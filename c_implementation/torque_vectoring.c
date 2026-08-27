#include "torque_vectoring.h"
#include "config.h"

#include <stddef.h>

/*
 * All arithmetic in this file is 32-bit. On 32-bit MCU cores 64-bit
 * multiplication and division are emulated in software (for example
 * __aeabi_uldivmod on ARM), so every intermediate product is kept inside
 * uint32_t by the configuration guards below, the validated parameter
 * bounds, and rejection of out-of-range dynamic inputs.
 */

#if TV_CONFIG_MAX_SPEED_MMPS > 131070U
#error "TV_CONFIG_MAX_SPEED_MMPS above 131070 overflows 32-bit speed math"
#endif

#if TV_CONFIG_RACK_MIN_MM < 1U
#error "TV_CONFIG_RACK_MIN_MM must be at least 1"
#endif

#if TV_CONFIG_RACK_MIN_MM > TV_CONFIG_RACK_RADIUS_CONSTANT_M_MM * 1000U
#error "TV_CONFIG_RACK_MIN_MM must not exceed the fitted radius constant"
#endif

#if TV_CONFIG_RACK_RADIUS_CONSTANT_M_MM > 4294967U
#error "TV_CONFIG_RACK_RADIUS_CONSTANT_M_MM * 1000 must fit in 32 bits"
#endif

#if TV_CONFIG_RADIUS_CORRECTION_PERMILLE > \
    2147483648U / \
    (TV_CONFIG_RACK_RADIUS_CONSTANT_M_MM * 1000U / TV_CONFIG_RACK_MIN_MM)
#error "Radius correction permille overflows 32-bit rack-radius math"
#endif

#if TV_CONFIG_RADIUS_CORRECTION_OFFSET_MM > 1073741823 || \
    TV_CONFIG_RADIUS_CORRECTION_OFFSET_MM < -1073741823
#error "TV_CONFIG_RADIUS_CORRECTION_OFFSET_MM must fit in 31 bits"
#endif

#if TV_CONFIG_SLIP_SPEED_PERMILLE > 1000U
#error "TV_CONFIG_SLIP_SPEED_PERMILLE above 1000 overflows the slip limit"
#endif

/* Physical parameter bounds that keep every 32-bit product below 2^32. */
#define TV_MIN_AXLE_DISTANCE_MM 500U
#define TV_MAX_AXLE_DISTANCE_MM 5000U
#define TV_MAX_CG_HEIGHT_MM 2000U
#define TV_MAX_WHEEL_RADIUS_MM 1000U
#define TV_MIN_GRAVITY_MMPS2 1000U
#define TV_MAX_GRAVITY_MMPS2 20000U
#define TV_MAX_FRICTION_PERMILLE 2000U

/** @brief Checks the geometry bounds required by the 32-bit equations. */
static bool geometry_is_valid(const VehicleParameters *vehicle)
{
    if (vehicle == NULL || vehicle->cg_height_mm == 0U ||
        vehicle->cg_height_mm > TV_MAX_CG_HEIGHT_MM ||
        vehicle->wheelbase_mm < TV_MIN_AXLE_DISTANCE_MM ||
        vehicle->wheelbase_mm > TV_MAX_AXLE_DISTANCE_MM ||
        vehicle->track_width_mm < TV_MIN_AXLE_DISTANCE_MM ||
        vehicle->track_width_mm > TV_MAX_AXLE_DISTANCE_MM ||
        vehicle->wheel_radius_mm > TV_MAX_WHEEL_RADIUS_MM ||
        vehicle->gravity_mmps2 < TV_MIN_GRAVITY_MMPS2 ||
        vehicle->gravity_mmps2 > TV_MAX_GRAVITY_MMPS2 ||
        vehicle->friction_permille == 0U ||
        vehicle->friction_permille > TV_MAX_FRICTION_PERMILLE) {
        return false;
    }

    /* Both axle-to-CG distances must be positive: |2 * x_c| < L. */
    const int32_t doubled_offset_mm =
        2 * (int32_t)vehicle->cg_offset_from_midpoint_mm;
    return doubled_offset_mm > -(int32_t)vehicle->wheelbase_mm &&
           doubled_offset_mm < (int32_t)vehicle->wheelbase_mm;
}

/** @brief Checks all ranges required by the integer torque-split equations. */
static bool vehicle_is_valid(const VehicleParameters *vehicle)
{
    if (!geometry_is_valid(vehicle) || vehicle->mass_kg == 0U ||
        vehicle->command_min >= vehicle->command_max) {
        return false;
    }
    /* command_min < command_max, so the unsigned difference is exact. */
    return (uint32_t)vehicle->command_max - (uint32_t)vehicle->command_min <=
           UINT16_MAX;
}

/** @brief Absolute value for int32_t without overflow at INT32_MIN. */
static uint32_t absolute_i32(int32_t value)
{
    return value < 0 ? 0U - (uint32_t)value : (uint32_t)value;
}

/**
 * @brief Unsigned division rounded to the nearest integer.
 *
 * Rounds via the remainder instead of adding denominator / 2 to the
 * numerator, so no numerator close to UINT32_MAX can overflow.
 */
static uint32_t divide_rounded_u32(uint32_t numerator, uint32_t denominator)
{
    const uint32_t quotient = numerator / denominator;
    const uint32_t remainder = numerator - quotient * denominator;
    return remainder >= denominator - remainder ? quotient + 1U : quotient;
}

/**
 * @brief Floor of sqrt(value) for a 32-bit radicand.
 *
 * Digit-by-digit integer square root; the result always fits in 16 bits.
 */
static uint32_t isqrt_u32(uint32_t value)
{
    uint32_t remainder = value;
    uint32_t root = 0U;
    uint32_t bit = (uint32_t)1U << 30;

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
    return root;
}

/** @brief sqrt(value) rounded to the nearest unsigned 32-bit integer. */
static uint32_t isqrt_rounded_u32(uint32_t value)
{
    uint32_t root = isqrt_u32(value);
    const uint32_t remainder = value - root * root;
    if (remainder > root) {
        root += 1U;
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

/**
 * @brief True when both speeds are at most TV_CONFIG_MAX_SPEED_MMPS.
 *
 * Rejecting out-of-range sensor values here also keeps every intermediate
 * kinematic product within 32 bits.
 */
static bool speeds_in_range_mmps(uint32_t a_mmps, uint32_t b_mmps)
{
    return a_mmps <= TV_CONFIG_MAX_SPEED_MMPS &&
           b_mmps <= TV_CONFIG_MAX_SPEED_MMPS;
}

/** @brief hypot(a, b) rounded to the nearest unsigned 32-bit integer. */
static uint32_t hypot_rounded_u32(uint32_t a, uint32_t b)
{
    /* 46340 = floor(sqrt(2^31)): two squares of shifted inputs fit in
       32 bits. In-range speeds need at most a few shift steps, which cost
       under 0.01% of the result. */
    const uint32_t largest = a > b ? a : b;
    uint32_t shift = 0U;
    while ((largest >> shift) > 46340U) {
        shift += 1U;
    }

    const uint32_t a_scaled = a >> shift;
    const uint32_t b_scaled = b >> shift;
    const uint32_t root =
        isqrt_rounded_u32(a_scaled * a_scaled + b_scaled * b_scaled);
    if (shift != 0U && root > (UINT32_MAX >> shift)) {
        return UINT32_MAX;
    }
    return root << shift;
}

/** @brief Adds an unsigned offset and clamps it to the configured command range. */
static int32_t command_from_offset(
    const VehicleParameters *vehicle,
    uint32_t offset)
{
    const uint32_t command_range =
        (uint32_t)vehicle->command_max - (uint32_t)vehicle->command_min;
    if (offset >= command_range) {
        return vehicle->command_max;
    }
    return vehicle->command_min + (int32_t)offset;
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

    /* The configuration guards above keep both products below 2^31. */
    const uint32_t fitted_radius_mm = divide_rounded_u32(
        TV_CONFIG_RACK_RADIUS_CONSTANT_M_MM * 1000U, magnitude_mm);
    const int32_t corrected_radius_mm =
        (int32_t)divide_rounded_u32(
            fitted_radius_mm * TV_CONFIG_RADIUS_CORRECTION_PERMILLE, 1000U) +
        TV_CONFIG_RADIUS_CORRECTION_OFFSET_MM;

    return corrected_radius_mm <= 0 ? 0U : (uint32_t)corrected_radius_mm;
}

uint32_t tv_com_velocity_from_rear_wheels_mmps(
    const VehicleParameters *vehicle,
    uint32_t rear_left_speed_mmps,
    uint32_t rear_right_speed_mmps)
{
    if (!geometry_is_valid(vehicle) ||
        !speeds_in_range_mmps(rear_left_speed_mmps, rear_right_speed_mmps)) {
        return 0U;
    }

    const uint32_t vx_mmps = divide_rounded_u32(
        rear_left_speed_mmps + rear_right_speed_mmps, 2U);
    /* delta <= 131070 and l_r < 5000, so the product stays below 2^32. */
    const uint32_t vy_mmps = divide_rounded_u32(
        unsigned_delta_u32(rear_left_speed_mmps, rear_right_speed_mmps) *
            rear_axle_to_cg_mm(vehicle),
        vehicle->track_width_mm);
    return hypot_rounded_u32(vx_mmps, vy_mmps);
}

uint32_t tv_wheel_rpm_to_speed_mmps(
    const VehicleParameters *vehicle,
    uint32_t rpm)
{
    if (vehicle == NULL || vehicle->wheel_radius_mm == 0U ||
        vehicle->wheel_radius_mm > TV_MAX_WHEEL_RADIUS_MM) {
        return 0U;
    }

    /* v = RPM * r * π / 30 with π ≈ 355/113 → RPM * r * 71 / 678. */
    const uint32_t scaled_radius = 71U * vehicle->wheel_radius_mm;
    if (rpm > UINT32_MAX / scaled_radius) {
        return UINT32_MAX;
    }
    return divide_rounded_u32(rpm * scaled_radius, 678U);
}

uint32_t tv_wheel_angular_speed_to_linear_mmps(
    const VehicleParameters *vehicle,
    uint32_t angular_speed_mradps)
{
    if (vehicle == NULL || vehicle->wheel_radius_mm == 0U ||
        vehicle->wheel_radius_mm > TV_MAX_WHEEL_RADIUS_MM) {
        return 0U;
    }

    if (angular_speed_mradps > UINT32_MAX / vehicle->wheel_radius_mm) {
        return UINT32_MAX;
    }
    return divide_rounded_u32(
        angular_speed_mradps * vehicle->wheel_radius_mm, 1000U);
}

uint32_t tv_filter_wheel_speed_mmps(
    TvWheelSpeedFilter *filter,
    uint32_t sample_mmps)
{
    if (filter == NULL) {
        return 0U;
    }
    if (sample_mmps > TV_CONFIG_MAX_SPEED_MMPS) {
        /* Treat an implausible sample like a missing frame: keep the state. */
        return filter->speed_mmps;
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
    /* Both terms are bounded speeds, so the weighted sum is below 2^32. */
    const uint32_t previous_mmps =
        filter->speed_mmps > TV_CONFIG_MAX_SPEED_MMPS
            ? TV_CONFIG_MAX_SPEED_MMPS
            : filter->speed_mmps;
    const uint32_t filtered_mmps = divide_rounded_u32(
        alpha_permille * sample_mmps +
            (1000U - alpha_permille) * previous_mmps,
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
    if (!geometry_is_valid(vehicle) ||
        !speeds_in_range_mmps(front_left_speed_mmps, front_right_speed_mmps) ||
        !speeds_in_range_mmps(rear_left_speed_mmps, rear_right_speed_mmps)) {
        return 0U;
    }

    const uint32_t yaw_mradps = divide_rounded_u32(
        unsigned_delta_u32(rear_left_speed_mmps, rear_right_speed_mmps) *
            1000U,
        vehicle->track_width_mm);
    if (yaw_mradps <= TV_CONFIG_STRAIGHT_YAW_MRADPS) {
        return divide_rounded_u32(
            front_left_speed_mmps + front_right_speed_mmps +
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
    if (!geometry_is_valid(vehicle) ||
        !speeds_in_range_mmps(front_left_speed_mmps, front_right_speed_mmps) ||
        !speeds_in_range_mmps(rear_left_speed_mmps, rear_right_speed_mmps)) {
        return result;
    }

    result.rear_com_velocity_mmps = tv_com_velocity_from_rear_wheels_mmps(
        vehicle, rear_left_speed_mmps, rear_right_speed_mmps);

    const uint32_t vx_mmps = divide_rounded_u32(
        rear_left_speed_mmps + rear_right_speed_mmps, 2U);
    /* delta <= 131070 and L <= 5000, so the product stays below 2^32. */
    const uint32_t omega_times_wheelbase_mmps = divide_rounded_u32(
        unsigned_delta_u32(rear_left_speed_mmps, rear_right_speed_mmps) *
            vehicle->wheelbase_mm,
        vehicle->track_width_mm);
    result.front_projected_mmps =
        hypot_rounded_u32(vx_mmps, omega_times_wheelbase_mmps);
    result.front_measured_mmps = divide_rounded_u32(
        front_left_speed_mmps + front_right_speed_mmps, 2U);

    const uint32_t disagreement_mmps = unsigned_delta_u32(
        result.front_measured_mmps, result.front_projected_mmps);
    const uint32_t limit_mmps = divide_rounded_u32(
        result.front_projected_mmps * TV_CONFIG_SLIP_SPEED_PERMILLE, 1000U);
    result.slip_detected = disagreement_mmps >= TV_CONFIG_SLIP_MIN_MMPS &&
                           disagreement_mmps >= limit_mmps;
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

    const uint32_t grip_acceleration_mmps2 =
        (uint32_t)vehicle->friction_permille * vehicle->gravity_mmps2 / 1000U;
    uint32_t lateral_acceleration_mmps2;
    if (vehicle_speed_mmps <= UINT16_MAX) {
        lateral_acceleration_mmps2 = divide_rounded_u32(
            vehicle_speed_mmps * vehicle_speed_mmps, radius_mm);
    } else {
        /* Halve the speed so its square fits in 32 bits. Comparing the
           quarter-scale acceleration with grip / 4 is exactly equivalent
           to comparing 4 * quarter with grip. */
        const uint32_t half_speed_mmps =
            divide_rounded_u32(vehicle_speed_mmps, 2U);
        const uint32_t quarter_acceleration_mmps2 = divide_rounded_u32(
            half_speed_mmps * half_speed_mmps, radius_mm);
        if (quarter_acceleration_mmps2 > grip_acceleration_mmps2 / 4U) {
            commands.status = TV_LATERAL_GRIP_EXCEEDED;
            return commands;
        }
        lateral_acceleration_mmps2 = 4U * quarter_acceleration_mmps2;
    }
    if (lateral_acceleration_mmps2 > grip_acceleration_mmps2) {
        commands.status = TV_LATERAL_GRIP_EXCEEDED;
        return commands;
    }
    commands.lateral_acceleration_mmps2 = lateral_acceleration_mmps2;

    /* Common scale factors cancel in the inner/outer torque ratio. The
       full load proxies g*(L+2*xc)*t and 4*L*a*h are both divided by
       4*L*t, which keeps them proportional and below 2^16. */
    const int32_t rear_numerator_mm = (int32_t)vehicle->wheelbase_mm +
        2 * (int32_t)vehicle->cg_offset_from_midpoint_mm;
    const uint32_t static_proxy = divide_rounded_u32(
        (uint32_t)vehicle->gravity_mmps2 * (uint32_t)rear_numerator_mm,
        4U * vehicle->wheelbase_mm);
    if (static_proxy == 0U) {
        return commands;
    }
    uint32_t transfer_proxy = divide_rounded_u32(
        lateral_acceleration_mmps2 * vehicle->cg_height_mm,
        vehicle->track_width_mm);
    /* Beyond inner-wheel lift-off the split saturates at inner = 0. */
    if (transfer_proxy > static_proxy) {
        transfer_proxy = static_proxy;
    }

    const uint32_t inner_weight = static_proxy - transfer_proxy;
    const uint32_t outer_weight = static_proxy + transfer_proxy;
    const uint32_t total_proxy = 2U * static_proxy;
    const uint32_t pedal_offset =
        (uint32_t)pedal_command - (uint32_t)vehicle->command_min;
    const uint32_t command_range =
        (uint32_t)vehicle->command_max - (uint32_t)vehicle->command_min;
    const uint32_t pedal_scaled = 2U * pedal_offset;

    uint32_t inner_offset;
    uint32_t outer_offset;
    if (pedal_scaled * outer_weight > command_range * total_proxy) {
        /* Saturate both sides proportionally, preserving the torque split. */
        outer_offset = command_range;
        inner_offset = divide_rounded_u32(
            command_range * inner_weight, outer_weight);
    } else {
        inner_offset = divide_rounded_u32(
            pedal_scaled * inner_weight, total_proxy);
        outer_offset = divide_rounded_u32(
            pedal_scaled * outer_weight, total_proxy);
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
