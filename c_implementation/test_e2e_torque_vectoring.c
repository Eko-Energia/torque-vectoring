/**
 * @file test_e2e_torque_vectoring.c
 * @brief End-to-end tests of the full control path to the motor commands.
 *
 * Each scenario replays one STM32 control cycle exactly as the firmware
 * would: raw wheel RPM from CAN -> linear speed -> EWMA filter -> slip
 * check -> CoM velocity -> torque split, and asserts the exact commands
 * that would reach the rear motors.
 */

#include "torque_vectoring.h"

#include <assert.h>
#include <stdio.h>

/** Per-wheel filter state kept across control cycles, as on the target. */
typedef struct {
    TvWheelSpeedFilter front_left;
    TvWheelSpeedFilter front_right;
    TvWheelSpeedFilter rear_left;
    TvWheelSpeedFilter rear_right;
} WheelFilters;

/**
 * One full control cycle: CAN wheel RPM and driver inputs in, motor
 * commands out. When the rear axle slips it cannot be trusted for the
 * velocity estimate, so the cycle falls back to the measured front-axle
 * speed before calculating the torque split.
 */
static WheelCommands control_cycle(
    const VehicleParameters *vehicle,
    WheelFilters *filters,
    uint32_t front_left_rpm,
    uint32_t front_right_rpm,
    uint32_t rear_left_rpm,
    uint32_t rear_right_rpm,
    bool rack_position_available,
    int32_t rack_displacement_mm,
    int32_t pedal_command)
{
    const uint32_t front_left_mmps = tv_filter_wheel_speed_mmps(
        &filters->front_left,
        tv_wheel_rpm_to_speed_mmps(vehicle, front_left_rpm));
    const uint32_t front_right_mmps = tv_filter_wheel_speed_mmps(
        &filters->front_right,
        tv_wheel_rpm_to_speed_mmps(vehicle, front_right_rpm));
    const uint32_t rear_left_mmps = tv_filter_wheel_speed_mmps(
        &filters->rear_left,
        tv_wheel_rpm_to_speed_mmps(vehicle, rear_left_rpm));
    const uint32_t rear_right_mmps = tv_filter_wheel_speed_mmps(
        &filters->rear_right,
        tv_wheel_rpm_to_speed_mmps(vehicle, rear_right_rpm));

    const TvSlipCheck slip = tv_check_wheel_slip(
        vehicle, front_left_mmps, front_right_mmps,
        rear_left_mmps, rear_right_mmps);
    const uint32_t vehicle_speed_mmps =
        slip.slip_detected
            ? slip.front_measured_mmps
            : tv_com_velocity_from_wheel_speeds_mmps(
                  vehicle, front_left_mmps, front_right_mmps,
                  rear_left_mmps, rear_right_mmps);

    return tv_calculate_rear_commands_from_rack(
        vehicle, rack_position_available, rack_displacement_mm,
        vehicle_speed_mmps, pedal_command);
}

int main(void)
{
    const VehicleParameters vehicle = tv_default_vehicle();

    /* Scenario: straight-line cruise at 273 RPM = 10006 mm/s on every
       wheel with the rack centred. Both motors get the pedal verbatim. */
    {
        WheelFilters filters = {0};
        const WheelCommands commands = control_cycle(
            &vehicle, &filters, 273U, 273U, 273U, 273U, true, 0, 200);
        assert(commands.status == TV_OK);
        assert(!commands.torque_vectoring_active);
        assert(commands.rear_left == 200);
        assert(commands.rear_right == 200);
    }

    /* Scenario: steady left turn, rack +10 mm = 50.7 m radius at about
       8 m/s. Wheel RPMs are kinematically consistent (left side slower),
       so no slip is flagged. The inner (left) motor sheds the torque the
       outer gains: 168 + 232 = 2 * pedal. */
    {
        WheelFilters filters = {0};
        const WheelCommands commands = control_cycle(
            &vehicle, &filters, 216U, 223U, 215U, 222U, true, 10, 200);
        assert(commands.status == TV_OK);
        assert(commands.torque_vectoring_active);
        assert(commands.turn_radius_mm == 50700U);
        assert(commands.rear_left == 168);
        assert(commands.rear_right == 232);
        assert(commands.rear_left + commands.rear_right == 2 * 200);
    }

    /* Scenario: both rear (driven) wheels spin up to 12022 mm/s while the
       front axle measures 9016 mm/s in a rack +40 mm = 12.675 m turn. The
       rear axle cannot be trusted, so the cycle falls back to the front
       speed and still produces a valid, saturated split. */
    {
        WheelFilters filters = {0};
        const WheelCommands commands = control_cycle(
            &vehicle, &filters, 246U, 246U, 328U, 328U, true, 40, 180);
        assert(commands.status == TV_OK);
        assert(commands.torque_vectoring_active);
        assert(commands.turn_radius_mm == 12675U);
        assert(commands.rear_left == 27);
        assert(commands.rear_right == 256);
    }

    /* Same sensor picture without the slip fallback: averaging the
       spinning rear wheels into the estimate (10519 mm/s) would exceed
       the lateral grip limit and cut all torque. The fallback above is
       what keeps the car driving. */
    {
        const WheelCommands commands = tv_calculate_rear_commands_from_rack(
            &vehicle, true, 40, 10519U, 180);
        assert(commands.status == TV_LATERAL_GRIP_EXCEEDED);
        assert(commands.rear_left == vehicle.command_min);
        assert(commands.rear_right == vehicle.command_min);
    }

    /* Scenario: full lock (rack 70 mm = 7.243 m) at 10 m/s demands about
       13.8 m/s^2 of lateral grip, above mu * g = 7.85 m/s^2. Both motors
       drop to command_min. */
    {
        WheelFilters filters = {0};
        const WheelCommands commands = control_cycle(
            &vehicle, &filters, 273U, 273U, 273U, 273U, true, 70, 200);
        assert(commands.status == TV_LATERAL_GRIP_EXCEEDED);
        assert(commands.rear_left == vehicle.command_min);
        assert(commands.rear_right == vehicle.command_min);
    }

    /* Scenario: full pedal in the 50.7 m turn. The outer motor saturates
       at command_max and the inner keeps the calculated torque ratio
       instead of the torque sum. */
    {
        WheelFilters filters = {0};
        const WheelCommands commands = control_cycle(
            &vehicle, &filters, 216U, 223U, 215U, 222U, true, 10, 256);
        assert(commands.status == TV_OK);
        assert(commands.rear_left == 185);
        assert(commands.rear_right == 256);
    }

    /* Scenario: six cycles of noisy straight-line CAN frames. The EWMA
       settles near the true 10006 mm/s and the motor commands stay glued
       to the pedal for every cycle. */
    {
        WheelFilters filters = {0};
        const uint32_t noisy_rpm[6] = {273U, 281U, 265U, 277U, 269U, 273U};
        for (unsigned i = 0U; i < 6U; i++) {
            const WheelCommands commands = control_cycle(
                &vehicle, &filters, noisy_rpm[i], noisy_rpm[i],
                noisy_rpm[i], noisy_rpm[i], true, 0, 120);
            assert(commands.status == TV_OK);
            assert(commands.rear_left == 120);
            assert(commands.rear_right == 120);
        }
        assert(filters.rear_left.speed_mmps == 9996U);
    }

    /* Scenario: the rack sensor drops out mid-drive. The controller falls
       back to an even split of the pedal instead of guessing a radius. */
    {
        WheelFilters filters = {0};
        const WheelCommands commands = control_cycle(
            &vehicle, &filters, 216U, 223U, 215U, 222U, false, 10, 200);
        assert(commands.status == TV_OK);
        assert(!commands.torque_vectoring_active);
        assert(commands.rear_left == 200);
        assert(commands.rear_right == 200);
    }

    puts("All end-to-end tests passed.");
    return 0;
}
