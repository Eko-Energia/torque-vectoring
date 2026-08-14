/**
 * @file torque_vectoring.h
 * @brief Integer-only torque-vectoring API intended for STM32 targets.
 *
 * The real-time path uses fixed physical units and contains no floating-point
 * operations, dynamic allocation, or dependency on math.h/libm/arm_math.
 */

#ifndef TORQUE_VECTORING_H
#define TORQUE_VECTORING_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Physical parameters and command range of the vehicle Perła. */
typedef struct {
    /** Vehicle mass [kg]. */
    uint16_t mass_kg;

    /** Centre-of-mass height above ground, h [mm]. */
    uint16_t cg_height_mm;

    /**
     * Longitudinal centre-of-mass offset x_c [mm]. Positive points toward the
     * rear; -40 means 40 mm toward the front from the wheelbase midpoint.
     */
    int16_t cg_offset_from_midpoint_mm;

    /** Distance between front and rear axle centres, L [mm]. */
    uint16_t wheelbase_mm;

    /** Distance between left and right wheel centres, t [mm]. */
    uint16_t track_width_mm;

    /** Tyre friction coefficient [1/1000]; 800 represents 0.8. */
    uint16_t friction_permille;

    /** Gravitational acceleration [mm/s^2]. */
    uint16_t gravity_mmps2;

    /** Pedal/output value representing zero requested torque. */
    int32_t command_min;

    /** Maximum accepted pedal/output value. */
    int32_t command_max;
} VehicleParameters;

/** @brief Result code returned by the control calculation. */
typedef enum {
    /** Calculation succeeded; outputs may be applied. */
    TV_OK = 0,

    /** Requested speed and radius exceed mu*g; outputs equal command_min. */
    TV_LATERAL_GRIP_EXCEEDED,

    /** Non-zero rack value is outside the calibrated range. */
    TV_RACK_OUT_OF_RANGE,

    /** Speed exceeds TV_CONFIG_MAX_SPEED_MMPS. */
    TV_SPEED_OUT_OF_RANGE,

    /** Invalid pointer, vehicle parameters, or pedal command. */
    TV_INVALID_ARGUMENT
} TvStatus;

/** @brief Rear-wheel commands and diagnostic values for one control iteration. */
typedef struct {
    /** Rear-left motor command [configured command units]. */
    int32_t rear_left;

    /** Rear-right motor command [configured command units]. */
    int32_t rear_right;

    /** Unsigned fitted turn radius [mm]; 0 in fallback or error. */
    uint32_t turn_radius_mm;

    /** Calculated lateral acceleration [mm/s^2]. */
    uint32_t lateral_acceleration_mmps2;

    /** True when an unequal torque split was calculated. */
    bool torque_vectoring_active;

    /** Validity and status of all fields above. */
    TvStatus status;
} WheelCommands;

/**
 * @brief Returns the default parameters of Perła.
 * @return Vehicle parameters in integer STM32 units.
 *
 * The returned structure may be copied and adjusted before it is passed to the
 * controller. The function has no side effects and uses no global mutable state.
 */
VehicleParameters tv_default_vehicle(void);

/**
 * @brief Converts signed steering-rack displacement to an unsigned turn radius.
 * @param rack_displacement_mm Rack displacement from its centre position [mm].
 *        Its sign selects the turn direction; its magnitude selects the radius.
 * @return Turn radius [mm], or 0 if the value is zero/outside calibration.
 *
 * Uses the integer form R_mm = 507000 / abs(rack_mm), followed by configured
 * permille/offset correction for non-zero displacements from 1 to 70 mm.
 * The caller must use the input sign to determine left/right direction. A zero
 * result is not a valid finite radius.
 */
uint32_t tv_rack_displacement_to_radius_mm(int32_t rack_displacement_mm);

/**
 * @brief Calculates rear-left and rear-right commands using integer arithmetic.
 * @param vehicle Pointer to validated vehicle parameters; must remain valid for
 *        the duration of the call.
 * @param rack_position_available true if rack_displacement_mm is a fresh and
 *        valid sensor measurement; false activates equal-split fallback.
 * @param rack_displacement_mm Signed displacement from centred rack [mm].
 *        Positive means a left turn, negative a right turn, and zero activates
 *        equal-split fallback.
 * @param vehicle_speed_mmps Non-negative vehicle speed [mm/s]. For example,
 *        5000 means 5 m/s.
 * @param pedal_command Driver demand in [vehicle.command_min, vehicle.command_max].
 * @return Commands and diagnostics. Apply motor commands only when status is
 *         TV_OK. In fallback, both outputs equal pedal_command and
 *         torque_vectoring_active is false.
 *
 * @note Call this function once per control-loop iteration. It is re-entrant,
 * deterministic, allocation-free, and has no dependency on an FPU or DSP library.
 */
WheelCommands tv_calculate_rear_commands_from_rack(
    const VehicleParameters *vehicle,
    bool rack_position_available,
    int32_t rack_displacement_mm,
    uint32_t vehicle_speed_mmps,
    int32_t pedal_command
);

/**
 * @brief Returns a static English description of a TvStatus value.
 * @param status Status code to describe.
 * @return Pointer to a null-terminated static string. Never free this pointer.
 * @note Intended for logs/debug builds; the control algorithm does not require it.
 */
const char *tv_status_string(TvStatus status);

#endif
