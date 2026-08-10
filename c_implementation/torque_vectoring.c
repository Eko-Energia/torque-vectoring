#include "torque_vectoring.h"
#include "config.h"

#include <limits.h>
#include <stddef.h>

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

static uint32_t absolute_i32(int32_t value)
{
    return value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

static uint64_t divide_rounded_u64(uint64_t numerator, uint64_t denominator)
{
    return (numerator + denominator / 2U) / denominator;
}

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
        .mass_kg = 850U,
        .cg_height_mm = 511U,
        .cg_offset_from_midpoint_mm = -40,
        .wheelbase_mm = 2750U,
        .track_width_mm = 1700U,
        .friction_permille = 800U,
        .gravity_mmps2 = 9810U,
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
