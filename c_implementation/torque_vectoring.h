#ifndef TORQUE_VECTORING_H
#define TORQUE_VECTORING_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t mass_kg;
    uint16_t cg_height_mm;
    int16_t cg_offset_from_midpoint_mm;
    uint16_t wheelbase_mm;
    uint16_t track_width_mm;
    uint16_t friction_permille;
    uint16_t gravity_mmps2;
    int32_t command_min;
    int32_t command_max;
} VehicleParameters;

typedef enum {
    TV_OK = 0,
    TV_LATERAL_GRIP_EXCEEDED,
    TV_RACK_OUT_OF_RANGE,
    TV_SPEED_OUT_OF_RANGE,
    TV_INVALID_ARGUMENT
} TvStatus;

typedef struct {
    int32_t rear_left;
    int32_t rear_right;
    uint32_t turn_radius_mm;
    uint32_t lateral_acceleration_mmps2;
    bool torque_vectoring_active;
    TvStatus status;
} WheelCommands;

VehicleParameters tv_default_vehicle(void);

/* Returns the unsigned turn radius. Zero means fallback or invalid input. */
uint32_t tv_rack_displacement_to_radius_mm(int32_t rack_displacement_mm);

/* Integer-only real-time path intended for STM32. */
WheelCommands tv_calculate_rear_commands_from_rack(
    const VehicleParameters *vehicle,
    bool rack_position_available,
    int32_t rack_displacement_mm,
    uint32_t vehicle_speed_mmps,
    int32_t pedal_command
);

const char *tv_status_string(TvStatus status);

#endif
