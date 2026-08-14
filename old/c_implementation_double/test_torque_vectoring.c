#include "torque_vectoring.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static bool nearly_equal(double actual, double expected, double tolerance)
{
    return fabs(actual - expected) <= tolerance;
}

int main(void)
{
    const VehicleParameters vehicle = tv_default_vehicle();
    const TorqueVectoringResult reference =
        tv_calculate_max_rear_torque(&vehicle, 6.5, 5.0);
    assert(reference.status == TV_OK);
    assert(nearly_equal(reference.inner_torque_nm, 291.560935664, 1e-6));
    assert(nearly_equal(reference.outer_torque_nm, 841.868627972, 1e-6));

    const TorqueVectoringResult opposite_turn =
        tv_calculate_max_rear_torque(&vehicle, -6.5, 5.0);
    assert(nearly_equal(opposite_turn.inner_torque_nm,
                        reference.inner_torque_nm, 1e-12));
    assert(nearly_equal(opposite_turn.outer_torque_nm,
                        reference.outer_torque_nm, 1e-12));

    const TorqueVectoringResult slip =
        tv_calculate_max_rear_torque(&vehicle, 6.5, 8.0);
    assert(slip.status == TV_LATERAL_GRIP_EXCEEDED);
    assert(slip.inner_torque_nm == 0.0 && slip.outer_torque_nm == 0.0);

    const int pedal_midpoint = vehicle.command_min +
        (vehicle.command_max - vehicle.command_min) / 2;
    const WheelCommands left_turn =
        tv_calculate_rear_commands(&vehicle, 6.5, 5.0, pedal_midpoint);
    assert(left_turn.status == TV_OK);
    assert(left_turn.rear_left >= vehicle.command_min);
    assert(left_turn.rear_right <= vehicle.command_max);
    assert(left_turn.rear_left < left_turn.rear_right);

    const WheelCommands right_turn =
        tv_calculate_rear_commands(&vehicle, -6.5, 5.0, pedal_midpoint);
    assert(right_turn.status == TV_OK);
    assert(right_turn.rear_left == left_turn.rear_right);
    assert(right_turn.rear_right == left_turn.rear_left);

    const WheelCommands no_pedal =
        tv_calculate_rear_commands(&vehicle, 6.5, 5.0, vehicle.command_min);
    assert(no_pedal.status == TV_OK);
    assert(no_pedal.rear_left == vehicle.command_min);
    assert(no_pedal.rear_right == vehicle.command_min);

    const WheelCommands full_pedal =
        tv_calculate_rear_commands(&vehicle, 6.5, 5.0, vehicle.command_max);
    assert(full_pedal.rear_left > vehicle.command_min);
    assert(full_pedal.rear_right == vehicle.command_max);
    const double command_ratio =
        (double)(full_pedal.rear_left - vehicle.command_min) /
        (double)(full_pedal.rear_right - vehicle.command_min);
    const double torque_ratio =
        reference.inner_torque_nm / reference.outer_torque_nm;
    assert(nearly_equal(command_ratio, torque_ratio, 0.02));

    const WheelCommands invalid_pedal =
        tv_calculate_rear_commands(&vehicle, 6.5, 5.0,
                                   vehicle.command_max + 1);
    assert(invalid_pedal.status == TV_INVALID_ARGUMENT);

    VehicleParameters custom_range = vehicle;
    custom_range.command_min = 0;
    custom_range.command_max = 100;
    const WheelCommands custom_commands =
        tv_calculate_rear_commands(&custom_range, 6.5, 5.0, 50);
    assert(custom_commands.status == TV_OK);
    assert(custom_commands.rear_left == 26);
    assert(custom_commands.rear_right == 74);
    const WheelCommands custom_limit =
        tv_calculate_rear_commands(&custom_range, 6.5, 5.0, 101);
    assert(custom_limit.status == TV_INVALID_ARGUMENT);

    assert(nearly_equal(tv_rack_displacement_to_radius(5.0),
                        101.4, 1e-12));
    assert(nearly_equal(tv_rack_displacement_to_radius(1.0),
                        507.0, 1e-12));
    assert(nearly_equal(tv_rack_displacement_to_radius(3.0),
                        169.0, 1e-12));
    assert(nearly_equal(tv_rack_displacement_to_radius(4.0),
                        126.75, 1e-12));
    assert(nearly_equal(tv_rack_displacement_to_radius(12.5),
                        40.56, 1e-12));
    assert(nearly_equal(tv_rack_displacement_to_radius(-70.0),
                        -7.242857142857, 1e-12));
    assert(isinf(tv_rack_displacement_to_radius(0.0)));
    assert(isnan(tv_rack_displacement_to_radius(71.0)));

    const WheelCommands rack_left = tv_calculate_rear_commands_from_rack(
        &vehicle, true, 70.0, 5.0, pedal_midpoint);
    assert(rack_left.status == TV_OK);
    assert(rack_left.torque_vectoring_active);
    assert(rack_left.rear_left < rack_left.rear_right);

    const WheelCommands rack_right = tv_calculate_rear_commands_from_rack(
        &vehicle, true, -70.0, 5.0, pedal_midpoint);
    assert(rack_right.status == TV_OK);
    assert(rack_right.rear_left == rack_left.rear_right);
    assert(rack_right.rear_right == rack_left.rear_left);

    const WheelCommands missing_rack = tv_calculate_rear_commands_from_rack(
        &vehicle, false, 0.0, 5.0, pedal_midpoint);
    assert(missing_rack.status == TV_OK);
    assert(!missing_rack.torque_vectoring_active);
    assert(missing_rack.rear_left == pedal_midpoint);
    assert(missing_rack.rear_right == pedal_midpoint);

    const WheelCommands zero_rack = tv_calculate_rear_commands_from_rack(
        &vehicle, true, 0.0, 5.0, pedal_midpoint);
    assert(zero_rack.status == TV_OK);
    assert(!zero_rack.torque_vectoring_active);
    assert(zero_rack.rear_left == pedal_midpoint);
    assert(zero_rack.rear_right == pedal_midpoint);

    const WheelCommands rack_out_of_range =
        tv_calculate_rear_commands_from_rack(
            &vehicle, true, 71.0, 5.0, pedal_midpoint);
    assert(rack_out_of_range.status == TV_RACK_OUT_OF_RANGE);

    assert(isinf(tv_steering_angle_to_radius(&vehicle, 0.0)));
    assert(nearly_equal(tv_steering_angle_to_radius(&vehicle, atan(2.75 / 10.0)),
                        10.0, 1e-12));

    const double radius = 10.0;
    const double half_track = vehicle.track_width_m * 0.5;
    const double yaw_rate = 0.5;
    const double measured_speed = tv_wheel_speeds_to_vehicle_speed(
        &vehicle,
        yaw_rate * hypot(vehicle.wheelbase_m, radius - half_track),
        yaw_rate * hypot(vehicle.wheelbase_m, radius + half_track),
        yaw_rate * (radius - half_track),
        yaw_rate * (radius + half_track),
        radius);
    const double cg_from_rear = vehicle.wheelbase_m * 0.5 +
                                vehicle.cg_offset_from_midpoint_m;
    assert(nearly_equal(measured_speed, yaw_rate * hypot(radius, cg_from_rear),
                        1e-12));

    puts("All tests passed.");
    return 0;
}
