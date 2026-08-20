#include "torque_vectoring.h"
#include "config.h"

#include <limits.h>
#include <stddef.h>

/** @brief Checks physical ranges required by the integer equations. */
static bool vehicle_is_valid(const VehicleParameters *vehicle)
{
    if (vehicle == NULL || vehicle->mass_kg == 0U ||
        vehicle->cg_height_mm == 0U || vehicle->wheelbase_mm == 0U ||
        vehicle->track_width_mm == 0U || vehicle->gravity_mmps2 == 0U ||
        vehicle->friction_permille == 0U ||
        vehicle->command_min >= vehicle->command_max ||
        (int64_t)vehicle->command_max - vehicle->command_min > UINT16_MAX) {
        return false;
    }

    const int32_t rear_numerator = (int32_t)vehicle->wheelbase_mm +
        2 * (int32_t)vehicle->cg_offset_from_midpoint_mm;
    return rear_numerator > 0;
}

/** @brief Absolute value for int32_t without overflow at INT32_MIN. */
static uint32_t absolute_i32(int32_t value)
{
    return value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

/** @brief Unsigned division rounded to the nearest integer. */
static uint64_t divide_rounded_u64(uint64_t numerator, uint64_t denominator)
{
    return (numerator + denominator / 2U) / denominator;
}

/**
 * @brief Floor of sqrt(value) for a 64-bit radicand.
 *
 * Digit-by-digit integer square root; the result always fits in 32 bits.
 */
static uint32_t isqrt_u64(uint64_t value)
{
    uint64_t remainder = value;
    uint64_t root = 0U;
    uint64_t bit = (uint64_t)1U << 62;

    while (bit > remainder) {
        bit >>= 2;
    }
    while (bit != 0U) {
        if (remainder >= root + bit) {
            remainder -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)root;
}

/** @brief sqrt(value) rounded to the nearest unsigned 32-bit integer. */
static uint32_t isqrt_rounded_u64(uint64_t value)
{
    const uint32_t root = isqrt_u64(value);
    if (root == UINT32_MAX) {
        return root;
    }
    const uint64_t remainder = value - (uint64_t)root * root;
    if (remainder > root) {
        return root + 1U;
    }
    return root;
}

/** @brief Distance from the rear axle to the CoM; zero if the CoM is on the axle. */
static uint32_t rear_axle_to_cg_mm(const VehicleParameters *vehicle)
{
    const int32_t lr_mm =
        ((int32_t)vehicle->wheelbase_mm -
         2 * (int32_t)vehicle->cg_offset_from_midpoint_mm) / 2;
    return absolute_i32(lr_mm);
}

/** @brief Absolute difference of two unsigned speeds. */
static uint32_t unsigned_delta_u32(uint32_t a, uint32_t b)
{
    return a >= b ? a - b : b - a;
}

/** @brief hypot(a, b) rounded to the nearest unsigned 32-bit integer. */
static uint32_t hypot_rounded_u32(uint32_t a, uint32_t b)
{
    const uint64_t a_squared = (uint64_t)a * a;
    const uint64_t b_squared = (uint64_t)b * b;
    const uint64_t sum =
        a_squared > UINT64_MAX - b_squared
            ? UINT64_MAX
            : a_squared + b_squared;
    return isqrt_rounded_u64(sum);
}

/** @brief Adds an unsigned offset and clamps it to the configured command range. */
static int32_t command_from_offset(
    const VehicleParameters *vehicle,
    uint64_t offset)
{
    const int64_t command = (int64_t)vehicle->command_min + (int64_t)offset;
    if (command < vehicle->command_min) {
        return vehicle->command_min;
    }
    if (command > vehicle->command_max) {
        return vehicle->command_max;
    }
    return (int32_t)command;
}

VehicleParameters tv_default_vehicle(void)
{
    return (VehicleParameters){
        /* Measured mass of the complete vehicle [kg]. */
        .mass_kg = 850U,

        /* CG is 511 mm above ground. */
        .cg_height_mm = 511U,

        /* CG is 40 mm ahead of the wheelbase midpoint. */
        .cg_offset_from_midpoint_mm = -40,

        /* Axle spacing, rear track width, and dynamic wheel radius [mm]. */
        .wheelbase_mm = 2750U,
        .track_width_mm = 1700U,
        .wheel_radius_mm = 350U,

        /* Tyre-road friction coefficient: 800 / 1000 = 0.8. */
        .friction_permille = 800U,

        /* Standard gravity expressed in integer STM32 units [mm/s^2]. */
        .gravity_mmps2 = 9810U,

        /* Valid pedal input and rear-wheel output range. */
        .command_min = TV_CONFIG_COMMAND_MIN,
        .command_max = TV_CONFIG_COMMAND_MAX,
    };
}

uint32_t tv_rack_displacement_to_radius_mm(int32_t rack_displacement_mm)
{
    const uint32_t magnitude_mm = absolute_i32(rack_displacement_mm);
    if (magnitude_mm < TV_CONFIG_RACK_MIN_MM ||
        magnitude_mm > TV_CONFIG_RACK_MAX_MM) {
        return 0U;
    }

    const uint64_t fitted_radius_mm = divide_rounded_u64(
        (uint64_t)TV_CONFIG_RACK_RADIUS_CONSTANT_M_MM * 1000U,
        magnitude_mm);
    const int64_t corrected_radius_mm =
        (int64_t)divide_rounded_u64(
            fitted_radius_mm * TV_CONFIG_RADIUS_CORRECTION_PERMILLE, 1000U) +
        TV_CONFIG_RADIUS_CORRECTION_OFFSET_MM;

    if (corrected_radius_mm <= 0 || corrected_radius_mm > UINT32_MAX) {
        return 0U;
    }
    return (uint32_t)corrected_radius_mm;
}

uint32_t tv_com_velocity_from_rear_wheels_mmps(
    const VehicleParameters *vehicle,
    uint32_t rear_left_speed_mmps,
    uint32_t rear_right_speed_mmps)
{
    if (vehicle == NULL || vehicle->track_width_mm == 0U) {
        return 0U;
    }

    const uint32_t vx_mmps = (uint32_t)divide_rounded_u64(
        (uint64_t)rear_left_speed_mmps + rear_right_speed_mmps, 2U);
    const uint32_t speed_diff_mmps =
        unsigned_delta_u32(rear_left_speed_mmps, rear_right_speed_mmps);
    const uint32_t vy_mmps = (uint32_t)divide_rounded_u64(
        (uint64_t)speed_diff_mmps * rear_axle_to_cg_mm(vehicle),
        vehicle->track_width_mm);
    return hypot_rounded_u32(vx_mmps, vy_mmps);
}

uint32_t tv_wheel_rpm_to_speed_mmps(
    const VehicleParameters *vehicle,
    uint32_t rpm)
{
    if (vehicle == NULL || vehicle->wheel_radius_mm == 0U) {
        return 0U;
    }

    /* v = RPM * r * π / 30 with π ≈ 355/113 → RPM * r * 355 / 3390. */
    const uint64_t speed_mmps = divide_rounded_u64(
        (uint64_t)rpm * vehicle->wheel_radius_mm * 355U, 3390U);
    return speed_mmps > UINT32_MAX ? UINT32_MAX : (uint32_t)speed_mmps;
}

uint32_t tv_wheel_angular_speed_to_linear_mmps(
    const VehicleParameters *vehicle,
    uint32_t angular_speed_mradps)
{
    if (vehicle == NULL || vehicle->wheel_radius_mm == 0U) {
        return 0U;
    }

    const uint64_t speed_mmps = divide_rounded_u64(
        (uint64_t)angular_speed_mradps * vehicle->wheel_radius_mm, 1000U);
    return speed_mmps > UINT32_MAX ? UINT32_MAX : (uint32_t)speed_mmps;
}

uint32_t tv_filter_wheel_speed_mmps(
    TvWheelSpeedFilter *filter,
    uint32_t sample_mmps)
{
    if (filter == NULL) {
        return 0U;
    }
    if (!filter->initialized) {
        filter->speed_mmps = sample_mmps;
        filter->initialized = true;
        return sample_mmps;
    }

    uint32_t alpha_permille = TV_CONFIG_EWMA_ALPHA_PERMILLE;
    if (alpha_permille > 1000U) {
        alpha_permille = 1000U;
    }
    const uint32_t filtered_mmps = (uint32_t)divide_rounded_u64(
        (uint64_t)alpha_permille * sample_mmps +
            (uint64_t)(1000U - alpha_permille) * filter->speed_mmps,
        1000U);
    filter->speed_mmps = filtered_mmps;
    return filtered_mmps;
}

uint32_t tv_com_velocity_from_wheel_speeds_mmps(
    const VehicleParameters *vehicle,
    uint32_t front_left_speed_mmps,
    uint32_t front_right_speed_mmps,
    uint32_t rear_left_speed_mmps,
    uint32_t rear_right_speed_mmps)
{
    if (vehicle == NULL || vehicle->track_width_mm == 0U) {
        return 0U;
    }

    const uint32_t yaw_mradps = (uint32_t)divide_rounded_u64(
        (uint64_t)unsigned_delta_u32(rear_left_speed_mmps, rear_right_speed_mmps) *
            1000U,
        vehicle->track_width_mm);
    if (yaw_mradps <= TV_CONFIG_STRAIGHT_YAW_MRADPS) {
        return (uint32_t)divide_rounded_u64(
            (uint64_t)front_left_speed_mmps + front_right_speed_mmps +
                rear_left_speed_mmps + rear_right_speed_mmps,
            4U);
    }
    return tv_com_velocity_from_rear_wheels_mmps(
        vehicle, rear_left_speed_mmps, rear_right_speed_mmps);
}

TvSlipCheck tv_check_wheel_slip(
    const VehicleParameters *vehicle,
    uint32_t front_left_speed_mmps,
    uint32_t front_right_speed_mmps,
    uint32_t rear_left_speed_mmps,
    uint32_t rear_right_speed_mmps)
{
    TvSlipCheck result = {0};
    if (vehicle == NULL || vehicle->track_width_mm == 0U) {
        return result;
    }

    result.rear_com_velocity_mmps = tv_com_velocity_from_rear_wheels_mmps(
        vehicle, rear_left_speed_mmps, rear_right_speed_mmps);

    const uint32_t vx_mmps = (uint32_t)divide_rounded_u64(
        (uint64_t)rear_left_speed_mmps + rear_right_speed_mmps, 2U);
    const uint32_t omega_times_wheelbase_mmps = (uint32_t)divide_rounded_u64(
        (uint64_t)unsigned_delta_u32(rear_left_speed_mmps, rear_right_speed_mmps) *
            vehicle->wheelbase_mm,
        vehicle->track_width_mm);
    result.front_projected_mmps =
        hypot_rounded_u32(vx_mmps, omega_times_wheelbase_mmps);
    result.front_measured_mmps = (uint32_t)divide_rounded_u64(
        (uint64_t)front_left_speed_mmps + front_right_speed_mmps, 2U);

    const uint32_t disagreement_mmps = unsigned_delta_u32(
        result.front_measured_mmps, result.front_projected_mmps);
    const uint32_t limit_mmps = (uint32_t)divide_rounded_u64(
        (uint64_t)result.front_projected_mmps * TV_CONFIG_SLIP_SPEED_PERMILLE,
        1000U);
    result.slip_detected = disagreement_mmps > 0U && disagreement_mmps >= limit_mmps;
    return result;
}

WheelCommands tv_calculate_rear_commands_from_rack(
    const VehicleParameters *vehicle,
    bool rack_position_available,
    int32_t rack_displacement_mm,
    uint32_t vehicle_speed_mmps,
    int32_t pedal_command)
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
    if (vehicle_speed_mmps > TV_CONFIG_MAX_SPEED_MMPS) {
        commands.status = TV_SPEED_OUT_OF_RANGE;
        return commands;
    }

    if (!rack_position_available || rack_displacement_mm == 0) {
        commands.rear_left = pedal_command;
        commands.rear_right = pedal_command;
        commands.status = TV_OK;
        return commands;
    }

    const uint32_t radius_mm =
        tv_rack_displacement_to_radius_mm(rack_displacement_mm);
    if (radius_mm == 0U) {
        commands.status = TV_RACK_OUT_OF_RANGE;
        return commands;
    }
    commands.turn_radius_mm = radius_mm;

    const uint64_t speed_squared =
        (uint64_t)vehicle_speed_mmps * vehicle_speed_mmps;
    const uint64_t grip_acceleration_mmps2 =
        (uint64_t)vehicle->friction_permille * vehicle->gravity_mmps2 / 1000U;
    if (speed_squared > (uint64_t)radius_mm * grip_acceleration_mmps2) {
        commands.status = TV_LATERAL_GRIP_EXCEEDED;
        return commands;
    }

    const uint32_t lateral_acceleration_mmps2 = (uint32_t)
        divide_rounded_u64(speed_squared, radius_mm);
    commands.lateral_acceleration_mmps2 = lateral_acceleration_mmps2;

    /* Common scale factors cancel in the inner/outer torque ratio.
       Proxies below are proportional to rear-wheel normal loads. */
    const int64_t rear_numerator = (int64_t)vehicle->wheelbase_mm +
        2 * (int64_t)vehicle->cg_offset_from_midpoint_mm;
    const int64_t static_proxy =
        (int64_t)vehicle->gravity_mmps2 * rear_numerator *
        vehicle->track_width_mm;
    const int64_t transfer_proxy =
        4 * (int64_t)vehicle->wheelbase_mm * lateral_acceleration_mmps2 *
        vehicle->cg_height_mm;
    int64_t inner_proxy = static_proxy - transfer_proxy;
    const int64_t outer_proxy = static_proxy + transfer_proxy;
    if (inner_proxy < 0) {
        inner_proxy = 0;
    }
    if (outer_proxy <= 0) {
        return commands;
    }

    const uint64_t total_proxy = (uint64_t)(inner_proxy + outer_proxy);
    const uint64_t inner_weight = (uint64_t)inner_proxy;
    const uint64_t outer_weight = (uint64_t)outer_proxy;
    const uint64_t pedal_offset =
        (uint64_t)((int64_t)pedal_command - vehicle->command_min);
    const uint64_t command_range =
        (uint64_t)((int64_t)vehicle->command_max - vehicle->command_min);

    uint64_t inner_offset;
    uint64_t outer_offset;
    if (pedal_offset * 2U * outer_weight > command_range * total_proxy) {
        /* Saturate both sides proportionally, preserving the torque split. */
        outer_offset = command_range;
        inner_offset = divide_rounded_u64(
            command_range * inner_weight, outer_weight);
    } else {
        inner_offset = divide_rounded_u64(
            pedal_offset * 2U * inner_weight, total_proxy);
        outer_offset = divide_rounded_u64(
            pedal_offset * 2U * outer_weight, total_proxy);
    }

    const int32_t inner_command = command_from_offset(vehicle, inner_offset);
    const int32_t outer_command = command_from_offset(vehicle, outer_offset);
    if (rack_displacement_mm > 0) {
        commands.rear_left = inner_command;
        commands.rear_right = outer_command;
    } else {
        commands.rear_left = outer_command;
        commands.rear_right = inner_command;
    }
    commands.torque_vectoring_active = true;
    commands.status = TV_OK;
    return commands;
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
    case TV_SPEED_OUT_OF_RANGE:
        return "vehicle speed outside configured range";
    case TV_INVALID_ARGUMENT:
        return "invalid argument";
    default:
        return "unknown status";
    }
}
