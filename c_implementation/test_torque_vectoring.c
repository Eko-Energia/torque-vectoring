#include "torque_vectoring.h"

#include <assert.h>
#include <limits.h>
#include <stdio.h>

int main(void)
{
    const VehicleParameters vehicle = tv_default_vehicle();
    assert(vehicle.mass_kg == 850U);
    assert(vehicle.cg_height_mm == 511U);
    assert(vehicle.cg_offset_from_midpoint_mm == -40);
    assert(vehicle.wheel_radius_mm == 350U);

    assert(tv_rack_displacement_to_radius_mm(1) == 507000U);
    assert(tv_rack_displacement_to_radius_mm(3) == 169000U);
    assert(tv_rack_displacement_to_radius_mm(4) == 126750U);
    assert(tv_rack_displacement_to_radius_mm(5) == 101400U);
    assert(tv_rack_displacement_to_radius_mm(10) == 50700U);
    assert(tv_rack_displacement_to_radius_mm(70) == 7243U);
    assert(tv_rack_displacement_to_radius_mm(-70) == 7243U);
    assert(tv_rack_displacement_to_radius_mm(0) == 0U);
    assert(tv_rack_displacement_to_radius_mm(71) == 0U);
    assert(tv_rack_displacement_to_radius_mm(INT32_MIN) == 0U);

    const int32_t pedal_midpoint = vehicle.command_min +
        (vehicle.command_max - vehicle.command_min) / 2;
    const WheelCommands left = tv_calculate_rear_commands_from_rack(
        &vehicle, true, 70, 5000U, pedal_midpoint, 100U);
    assert(left.status == TV_OK);
    assert(left.torque_vectoring_active);
    assert(left.turn_radius_mm == 7243U);
    assert(left.lateral_acceleration_mmps2 == 3452U);
    assert(left.rear_left == 72);
    assert(left.rear_right == 184);

    const WheelCommands right = tv_calculate_rear_commands_from_rack(
        &vehicle, true, -70, 5000U, pedal_midpoint, 100U);
    assert(right.status == TV_OK);
    assert(right.rear_left == left.rear_right);
    assert(right.rear_right == left.rear_left);

    const WheelCommands full_pedal = tv_calculate_rear_commands_from_rack(
        &vehicle, true, 70, 5000U, vehicle.command_max, 100U);
    assert(full_pedal.status == TV_OK);
    assert(full_pedal.rear_left == 101);
    assert(full_pedal.rear_right == vehicle.command_max);

    const WheelCommands no_pedal = tv_calculate_rear_commands_from_rack(
        &vehicle, true, 70, 5000U, vehicle.command_min, 100U);
    assert(no_pedal.status == TV_OK);
    assert(no_pedal.rear_left == vehicle.command_min);
    assert(no_pedal.rear_right == vehicle.command_min);

    const WheelCommands missing_rack = tv_calculate_rear_commands_from_rack(
        &vehicle, false, 0, 5000U, pedal_midpoint, 100U);
    assert(missing_rack.status == TV_OK);
    assert(!missing_rack.torque_vectoring_active);
    assert(missing_rack.rear_left == pedal_midpoint);
    assert(missing_rack.rear_right == pedal_midpoint);

    const WheelCommands zero_rack = tv_calculate_rear_commands_from_rack(
        &vehicle, true, 0, 5000U, pedal_midpoint, 100U);
    assert(zero_rack.status == TV_OK);
    assert(!zero_rack.torque_vectoring_active);
    assert(zero_rack.rear_left == pedal_midpoint);
    assert(zero_rack.rear_right == pedal_midpoint);

    const WheelCommands small_rack = tv_calculate_rear_commands_from_rack(
        &vehicle, true, 3, 5000U, pedal_midpoint, 100U);
    assert(small_rack.status == TV_OK);
    assert(small_rack.torque_vectoring_active);
    assert(small_rack.rear_left == 126);
    assert(small_rack.rear_right == 130);

    const WheelCommands slip = tv_calculate_rear_commands_from_rack(
        &vehicle, true, 70, 8000U, pedal_midpoint, 100U);
    assert(slip.status == TV_LATERAL_GRIP_EXCEEDED);
    assert(slip.rear_left == vehicle.command_min);
    assert(slip.rear_right == vehicle.command_min);

    const WheelCommands bad_rack = tv_calculate_rear_commands_from_rack(
        &vehicle, true, 71, 5000U, pedal_midpoint, 100U);
    assert(bad_rack.status == TV_RACK_OUT_OF_RANGE);

    const WheelCommands bad_speed = tv_calculate_rear_commands_from_rack(
        &vehicle, true, 70, 100001U, pedal_midpoint, 100U);
    assert(bad_speed.status == TV_SPEED_OUT_OF_RANGE);

    const WheelCommands bad_pedal = tv_calculate_rear_commands_from_rack(
        &vehicle, true, 70, 5000U, vehicle.command_max + 1, 100U);
    assert(bad_pedal.status == TV_INVALID_ARGUMENT);

    VehicleParameters custom_range = vehicle;
    custom_range.command_min = 0;
    custom_range.command_max = 100;
    const WheelCommands custom = tv_calculate_rear_commands_from_rack(
        &custom_range, true, 70, 5000U, 50, 100U);
    assert(custom.status == TV_OK);
    assert(custom.rear_left == 28);
    assert(custom.rear_right == 72);

    /* tv_gain_percent blends the full split toward the equal-split fallback:
       0% must reproduce the fallback exactly, 100% must reproduce the
       unblended split from `left` above, and values above 100 clamp to it. */
    const WheelCommands open_diff = tv_calculate_rear_commands_from_rack(
        &vehicle, true, 70, 5000U, pedal_midpoint, 0U);
    assert(open_diff.status == TV_OK);
    assert(!open_diff.torque_vectoring_active);
    assert(open_diff.rear_left == pedal_midpoint);
    assert(open_diff.rear_right == pedal_midpoint);

    const WheelCommands half_gain = tv_calculate_rear_commands_from_rack(
        &vehicle, true, 70, 5000U, pedal_midpoint, 50U);
    assert(half_gain.status == TV_OK);
    assert(half_gain.torque_vectoring_active);
    assert(half_gain.rear_left == 100);
    assert(half_gain.rear_right == 156);
    assert(half_gain.rear_left + half_gain.rear_right == 2 * pedal_midpoint);

    const WheelCommands clamped_gain = tv_calculate_rear_commands_from_rack(
        &vehicle, true, 70, 5000U, pedal_midpoint, 500U);
    assert(clamped_gain.status == TV_OK);
    assert(clamped_gain.rear_left == left.rear_left);
    assert(clamped_gain.rear_right == left.rear_right);

    assert(tv_com_velocity_from_rear_wheels_mmps(&vehicle, 5000U, 5000U) ==
           5000U);
    assert(tv_com_velocity_from_rear_wheels_mmps(&vehicle, 0U, 0U) == 0U);
    assert(tv_com_velocity_from_rear_wheels_mmps(&vehicle, 4000U, 6000U) ==
           5270U);
    assert(tv_com_velocity_from_rear_wheels_mmps(&vehicle, 6000U, 4000U) ==
           5270U);
    assert(tv_com_velocity_from_rear_wheels_mmps(NULL, 5000U, 5000U) == 0U);
    assert(tv_com_velocity_from_rear_wheels_mmps(&vehicle, 100001U, 5000U) ==
           0U);

    VehicleParameters wide_track = vehicle;
    wide_track.track_width_mm = 2000U;
    assert(tv_com_velocity_from_rear_wheels_mmps(&wide_track, 4000U, 6000U) ==
           5196U);

    assert(tv_wheel_rpm_to_speed_mmps(&vehicle, 100U) == 3665U);
    assert(tv_wheel_rpm_to_speed_mmps(NULL, 100U) == 0U);
    assert(tv_wheel_angular_speed_to_linear_mmps(&vehicle, 10000U) == 3500U);
    VehicleParameters no_radius = vehicle;
    no_radius.wheel_radius_mm = 0U;
    assert(tv_wheel_rpm_to_speed_mmps(&no_radius, 100U) == 0U);

    TvWheelSpeedFilter wheel_filter = {0};
    assert(tv_filter_wheel_speed_mmps(&wheel_filter, 1000U) == 1000U);
    assert(tv_filter_wheel_speed_mmps(&wheel_filter, 2000U) == 1200U);
    assert(wheel_filter.speed_mmps == 1200U);
    assert(tv_filter_wheel_speed_mmps(NULL, 1000U) == 0U);

    assert(tv_com_velocity_from_wheel_speeds_mmps(
               &vehicle, 5100U, 4900U, 5000U, 5000U) == 5000U);
    assert(tv_com_velocity_from_wheel_speeds_mmps(
               &vehicle, 0U, 0U, 4000U, 6000U) == 5270U);
    assert(tv_com_velocity_from_wheel_speeds_mmps(
               NULL, 5000U, 5000U, 5000U, 5000U) == 0U);
    assert(tv_com_velocity_from_wheel_speeds_mmps(
               &vehicle, 100001U, 5000U, 5000U, 5000U) == 0U);

    const TvSlipCheck no_slip = tv_check_wheel_slip(
        &vehicle, 5955U, 5955U, 4000U, 6000U);
    assert(!no_slip.slip_detected);
    assert(no_slip.rear_com_velocity_mmps == 5270U);
    assert(no_slip.front_projected_mmps == 5955U);
    assert(no_slip.front_measured_mmps == 5955U);

    const TvSlipCheck locked_front = tv_check_wheel_slip(
        &vehicle, 0U, 0U, 4000U, 6000U);
    assert(locked_front.slip_detected);
    assert(locked_front.front_measured_mmps == 0U);

    const TvSlipCheck spinning_front = tv_check_wheel_slip(
        &vehicle, 8000U, 8000U, 4000U, 6000U);
    assert(spinning_front.slip_detected);

    const TvSlipCheck standing = tv_check_wheel_slip(
        &vehicle, 0U, 0U, 0U, 0U);
    assert(!standing.slip_detected);

    /* Sensor noise while crawling must stay below the absolute deadband. */
    const TvSlipCheck crawling = tv_check_wheel_slip(
        &vehicle, 250U, 250U, 0U, 0U);
    assert(!crawling.slip_detected);

    assert(tv_check_wheel_slip(NULL, 5000U, 5000U, 5000U, 5000U)
               .rear_com_velocity_mmps == 0U);
    assert(tv_check_wheel_slip(&vehicle, 5000U, 5000U, 100001U, 5000U)
               .rear_com_velocity_mmps == 0U);

    puts("All integer tests passed.");
    return 0;
}
