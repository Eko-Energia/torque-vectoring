#include <math.h>

const int MASS = 700;                 // mass of the car (kg)
const double H = 0.511;               // height of the center of gravity (m)
const double XC = -0.04;              // longitudinal offset of the center of gravity (m)
const double WHEELBASE = 2.75;        // wheelbase (m)                
const double TRACK_WIDTH = 1.7;       // track width (m)
const double MU = 0.8;                // tire friction coefficient
const double WHEEL_DIAMETER = 0.7;    // wheel diameter (m)
const double G = 9.81;                // gravitational acceleration (m/s^2)
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0) // wheel radius (m)

// speed_*: prędkości liniowe kół [m/s]
// turn_radius: promień toru środka tylnej osi [m]
double speeds_to_car_speed(double speed_fl, double speed_fr, double speed_rl, double speed_rr, double turn_radius) {
    const double t2 = TRACK_WIDTH * 0.5;
    const double Rr = turn_radius; // promień środka tylnej osi

    // jazda prawie na wprost
    if (fabs(Rr) < 1e-9) {
        return 0.25 * (speed_fl + speed_fr + speed_rl + speed_rr);
    }

    // Odległość CoM od tylnej osi (uwzględnia XC)
    const double x_cg_from_rear = (WHEELBASE * 0.5) + XC;

    // Promienie toru kół:
    // tył: bez składowej wzdłużnej (oś tylna jako odniesienie skrętu)
    const double r_rl = fabs(Rr - t2);
    const double r_rr = fabs(Rr + t2);

    // przód: dochodzi wheelbase
    const double r_fl = hypot(WHEELBASE, Rr - t2);
    const double r_fr = hypot(WHEELBASE, Rr + t2);

    // Prędkość kątowa pojazdu z każdego koła
    const double omega_fl = speed_fl / r_fl;
    const double omega_fr = speed_fr / r_fr;
    const double omega_rl = speed_rl / r_rl;
    const double omega_rr = speed_rr / r_rr;

    // Uśrednienie (brak poślizgu -> powinny być bardzo zbliżone)
    const double omega = 0.25 * (omega_fl + omega_fr + omega_rl + omega_rr);

    // Promień toru CoM względem ICR
    const double R_cg = hypot(Rr, x_cg_from_rear);

    // Prędkość środka masy
    return omega * R_cg;
}

int radius_and_speed_to_max_torque(int turn_radius, int speed) {
    return 0;
}

int steering_angle_to_radius(int steering_angle) {
    return 0;
}

