#ifndef TORQUE_VECTORING_H
#define TORQUE_VECTORING_H

#include <stdbool.h>

typedef struct {
    double mass_kg;
    double cg_height_m;
    double cg_offset_from_midpoint_m;
    double wheelbase_m;
    double track_width_m;
    double friction_coefficient;
    double wheel_radius_m;
    double gravity_mps2;
    int command_min;
    int command_max;
} VehicleParameters;

typedef enum {
    TV_OK = 0,
    TV_LATERAL_GRIP_EXCEEDED,
    TV_INVALID_ARGUMENT
} TvStatus;

typedef struct {
    double inner_torque_nm;
    double outer_torque_nm;
    double lateral_acceleration_mps2;
    double rear_axle_normal_load_n;
    TvStatus status;
} TorqueVectoringResult;

typedef struct {
    int rear_left;
    int rear_right;
    TvStatus status;
} WheelCommands;

VehicleParameters tv_default_vehicle(void);

TorqueVectoringResult tv_calculate_max_rear_torque(
    const VehicleParameters *vehicle,
    double turn_radius_m,
    double vehicle_speed_mps
);

WheelCommands tv_calculate_rear_commands(
    const VehicleParameters *vehicle,
    double turn_radius_m,
    double vehicle_speed_mps,
    int pedal_command
);

double tv_steering_angle_to_radius(
    const VehicleParameters *vehicle,
    double steering_angle_rad
);

double tv_wheel_speeds_to_vehicle_speed(
    const VehicleParameters *vehicle,
    double speed_front_left_mps,
    double speed_front_right_mps,
    double speed_rear_left_mps,
    double speed_rear_right_mps,
    double rear_axle_turn_radius_m
);

const char *tv_status_string(TvStatus status);

#endif
