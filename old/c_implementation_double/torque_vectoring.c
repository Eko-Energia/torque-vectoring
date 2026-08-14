#include "torque_vectoring.h"
#include "config.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

static bool vehicle_is_valid(const VehicleParameters *vehicle)
{
    return vehicle != NULL && vehicle->mass_kg > 0.0 &&
           vehicle->cg_height_m >= 0.0 && vehicle->wheelbase_m > 0.0 &&
           vehicle->track_width_m > 0.0 &&
           vehicle->friction_coefficient >= 0.0 &&
           vehicle->wheel_radius_m > 0.0 && vehicle->gravity_mps2 > 0.0 &&
           vehicle->command_min < vehicle->command_max;
}

VehicleParameters tv_default_vehicle(void)
{
    return (VehicleParameters){
        /* Measured mass of the complete vehicle [kg]. */
        .mass_kg = 850.0,

        /* CG is 0.511 m above ground. */
        .cg_height_m = 0.511,

        /* CG is 0.04 m ahead of the wheelbase midpoint. */
        .cg_offset_from_midpoint_m = -0.04,

        /* Axle spacing and rear track width [m]. */
        .wheelbase_m = 2.75,
        .track_width_m = 1.7,

        /* Assumed tyre-road friction coefficient [-]. */
        .friction_coefficient = 0.8,

        /* Effective driven-wheel radius used to convert force to torque [m]. */
        .wheel_radius_m = 0.35,

        /* Standard gravitational acceleration [m/s^2]. */
        .gravity_mps2 = 9.81,

        /* Valid pedal input and rear-wheel output range. */
        .command_min = TV_CONFIG_COMMAND_MIN,
        .command_max = TV_CONFIG_COMMAND_MAX,
    };
}

TorqueVectoringResult tv_calculate_max_rear_torque(
    const VehicleParameters *vehicle,
    double turn_radius_m,
    double vehicle_speed_mps)
{
    TorqueVectoringResult result = {.status = TV_INVALID_ARGUMENT};

    if (!vehicle_is_valid(vehicle) || !isfinite(turn_radius_m) ||
        !isfinite(vehicle_speed_mps) || turn_radius_m == 0.0 ||
        vehicle_speed_mps < 0.0) {
        return result;
    }

    const double radius_m = fabs(turn_radius_m);
    const double lateral_acceleration = vehicle_speed_mps * vehicle_speed_mps / radius_m;
    const double lateral_force = vehicle->mass_kg * lateral_acceleration;
    const double total_grip = vehicle->friction_coefficient * vehicle->mass_kg *
                              vehicle->gravity_mps2;

    result.lateral_acceleration_mps2 = lateral_acceleration;
    if (lateral_force > total_grip) {
        result.status = TV_LATERAL_GRIP_EXCEEDED;
        return result;
    }

    /* Same static front/rear distribution as the Python reference model. */
    const double weight = vehicle->mass_kg * vehicle->gravity_mps2;
    const double rear_load = weight * 0.5 +
        weight * vehicle->cg_offset_from_midpoint_m / vehicle->wheelbase_m;
    const double lateral_transfer = lateral_force * vehicle->cg_height_m /
                                    vehicle->track_width_m;
    const double inner_load = rear_load * 0.5 - lateral_transfer;
    const double outer_load = rear_load * 0.5 + lateral_transfer;

    result.rear_axle_normal_load_n = rear_load;
    result.inner_torque_nm = fmax(0.0, inner_load) *
        vehicle->friction_coefficient * vehicle->wheel_radius_m;
    result.outer_torque_nm = fmax(0.0, outer_load) *
        vehicle->friction_coefficient * vehicle->wheel_radius_m;
    result.status = TV_OK;
    return result;
}

static int command_from_offset(
    const VehicleParameters *vehicle,
    double command_offset)
{
    const long command = lround((double)vehicle->command_min + command_offset);
    if (command < vehicle->command_min) {
        return vehicle->command_min;
    }
    if (command > vehicle->command_max) {
        return vehicle->command_max;
    }
    return (int)command;
}

WheelCommands tv_calculate_rear_commands(
    const VehicleParameters *vehicle,
    double turn_radius_m,
    double vehicle_speed_mps,
    int pedal_command)
{
    WheelCommands commands = {.status = TV_INVALID_ARGUMENT};
    if (!vehicle_is_valid(vehicle)) {
        return commands;
    }
    commands.rear_left = vehicle->command_min;
    commands.rear_right = vehicle->command_min;
    if (pedal_command < vehicle->command_min ||
        pedal_command > vehicle->command_max) {
        return commands;
    }

    const TorqueVectoringResult limits =
        tv_calculate_max_rear_torque(vehicle, turn_radius_m, vehicle_speed_mps);
    commands.status = limits.status;
    if (limits.status != TV_OK) {
        return commands;
    }

    const double total_torque = limits.inner_torque_nm + limits.outer_torque_nm;
    if (total_torque <= 0.0 || pedal_command == vehicle->command_min) {
        commands.status = TV_OK;
        return commands;
    }

    /* Keep pedal_command as the straight-line value for each rear wheel. */
    const double inner_ratio = 2.0 * limits.inner_torque_nm / total_torque;
    const double outer_ratio = 2.0 * limits.outer_torque_nm / total_torque;
    const double pedal_offset = (double)(pedal_command - vehicle->command_min);
    double inner_offset = pedal_offset * inner_ratio;
    double outer_offset = pedal_offset * outer_ratio;

    /* Scale both commands together when either one reaches the configured max.
       This preserves the requested torque-vectoring ratio during saturation. */
    const double available_range =
        (double)(vehicle->command_max - vehicle->command_min);
    const double largest_offset = fmax(inner_offset, outer_offset);
    if (largest_offset > available_range) {
        const double saturation_scale = available_range / largest_offset;
        inner_offset *= saturation_scale;
        outer_offset *= saturation_scale;
    }

    const int inner_command = command_from_offset(vehicle, inner_offset);
    const int outer_command = command_from_offset(vehicle, outer_offset);

    /* Positive radius means a left turn; the left wheel is then the inner one. */
    if (turn_radius_m > 0.0) {
        commands.rear_left = inner_command;
        commands.rear_right = outer_command;
    } else {
        commands.rear_left = outer_command;
        commands.rear_right = inner_command;
    }
    commands.torque_vectoring_active = true;
    return commands;
}

double tv_rack_displacement_to_radius(double rack_displacement_mm)
{
    if (!isfinite(rack_displacement_mm)) {
        return NAN;
    }

    const double magnitude_mm = fabs(rack_displacement_mm);
    if (magnitude_mm == 0.0) {
        return copysign(INFINITY, rack_displacement_mm);
    }
    if (magnitude_mm < TV_CONFIG_RACK_MIN_MM ||
        magnitude_mm > TV_CONFIG_RACK_MAX_MM) {
        return NAN;
    }

    const double fitted_radius_m =
        TV_CONFIG_RACK_RADIUS_CONSTANT / magnitude_mm;
    const double corrected_radius_m =
        fitted_radius_m * TV_CONFIG_RADIUS_CORRECTION_SCALE +
        TV_CONFIG_RADIUS_CORRECTION_OFFSET_M;
    if (!isfinite(corrected_radius_m) || corrected_radius_m <= 0.0) {
        return NAN;
    }
    return copysign(corrected_radius_m, rack_displacement_mm);
}

WheelCommands tv_calculate_rear_commands_from_rack(
    const VehicleParameters *vehicle,
    bool rack_position_available,
    double rack_displacement_mm,
    double vehicle_speed_mps,
    int pedal_command)
{
    WheelCommands commands = {.status = TV_INVALID_ARGUMENT};
    if (!vehicle_is_valid(vehicle)) {
        return commands;
    }
    commands.rear_left = vehicle->command_min;
    commands.rear_right = vehicle->command_min;
    if (pedal_command < vehicle->command_min ||
        pedal_command > vehicle->command_max || !isfinite(vehicle_speed_mps) ||
        vehicle_speed_mps < 0.0) {
        return commands;
    }

    if (!rack_position_available || rack_displacement_mm == 0.0) {
        commands.rear_left = pedal_command;
        commands.rear_right = pedal_command;
        commands.status = TV_OK;
        return commands;
    }

    const double radius_m =
        tv_rack_displacement_to_radius(rack_displacement_mm);
    if (!isfinite(radius_m)) {
        commands.status = TV_RACK_OUT_OF_RANGE;
        return commands;
    }
    return tv_calculate_rear_commands(
        vehicle, radius_m, vehicle_speed_mps, pedal_command);
}

double tv_steering_angle_to_radius(
    const VehicleParameters *vehicle,
    double steering_angle_rad)
{
    if (!vehicle_is_valid(vehicle) || !isfinite(steering_angle_rad)) {
        return NAN;
    }
    if (fabs(steering_angle_rad) < 1e-12) {
        return copysign(INFINITY, steering_angle_rad);
    }
    return vehicle->wheelbase_m / tan(steering_angle_rad);
}

double tv_wheel_speeds_to_vehicle_speed(
    const VehicleParameters *vehicle,
    double speed_front_left_mps,
    double speed_front_right_mps,
    double speed_rear_left_mps,
    double speed_rear_right_mps,
    double rear_axle_turn_radius_m)
{
    if (!vehicle_is_valid(vehicle) || !isfinite(rear_axle_turn_radius_m) ||
        !isfinite(speed_front_left_mps) || !isfinite(speed_front_right_mps) ||
        !isfinite(speed_rear_left_mps) || !isfinite(speed_rear_right_mps)) {
        return NAN;
    }

    if (fabs(rear_axle_turn_radius_m) > 1e6) {
        return 0.25 * (speed_front_left_mps + speed_front_right_mps +
                       speed_rear_left_mps + speed_rear_right_mps);
    }

    const double half_track = vehicle->track_width_m * 0.5;
    const double left_offset = rear_axle_turn_radius_m - half_track;
    const double right_offset = rear_axle_turn_radius_m + half_track;
    const double radius_rear_left = fabs(left_offset);
    const double radius_rear_right = fabs(right_offset);
    const double radius_front_left = hypot(vehicle->wheelbase_m, left_offset);
    const double radius_front_right = hypot(vehicle->wheelbase_m, right_offset);

    if (radius_rear_left < DBL_EPSILON || radius_rear_right < DBL_EPSILON) {
        return NAN;
    }

    const double yaw_rate = 0.25 * (
        speed_front_left_mps / radius_front_left +
        speed_front_right_mps / radius_front_right +
        speed_rear_left_mps / radius_rear_left +
        speed_rear_right_mps / radius_rear_right);
    const double cg_from_rear = vehicle->wheelbase_m * 0.5 +
                                vehicle->cg_offset_from_midpoint_m;
    const double cg_radius = hypot(rear_axle_turn_radius_m, cg_from_rear);
    return yaw_rate * cg_radius;
}

const char *tv_status_string(TvStatus status)
{
    switch (status) {
    case TV_OK:
        return "ok";
    case TV_LATERAL_GRIP_EXCEEDED:
        return "lateral grip exceeded";
    case TV_RACK_OUT_OF_RANGE:
        return "rack displacement outside calibration range";
    case TV_INVALID_ARGUMENT:
        return "invalid argument";
    default:
        return "unknown status";
    }
}
