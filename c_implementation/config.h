/**
 * @file config.h
 * @brief Compile-time calibration and safety limits for torque vectoring.
 */

#ifndef TV_CONFIG_H
#define TV_CONFIG_H

/** Minimum pedal and motor command; represents zero requested torque. */
#ifndef TV_CONFIG_COMMAND_MIN
#define TV_CONFIG_COMMAND_MIN 0
#endif

/** Maximum pedal and motor command. Value 256 requires at least 9 storage bits. */
#ifndef TV_CONFIG_COMMAND_MAX
#define TV_CONFIG_COMMAND_MAX 256
#endif

/** Smallest supported non-zero rack displacement magnitude [mm]. */
#define TV_CONFIG_RACK_MIN_MM 1U

/** Largest calibrated rack displacement magnitude [mm]. */
#define TV_CONFIG_RACK_MAX_MM 70U

/** Constant C [m*mm] in the fitted relation R_m = C / abs(rack_mm). */
#define TV_CONFIG_RACK_RADIUS_CONSTANT_M_MM 507U

/** Radius scale [permille]; 1000 = 1.000, 980 = 0.980. */
#define TV_CONFIG_RADIUS_CORRECTION_PERMILLE 1000U

/** Signed radius correction applied after scaling [mm]. */
#define TV_CONFIG_RADIUS_CORRECTION_OFFSET_MM 0

/** Maximum accepted speed sensor value [mm/s]; 100000 = 100 m/s. */
#define TV_CONFIG_MAX_SPEED_MMPS 100000U

/** EWMA weight for a new wheel-speed sample [permille]; 200 = 0.2. */
#ifndef TV_CONFIG_EWMA_ALPHA_PERMILLE
#define TV_CONFIG_EWMA_ALPHA_PERMILLE 200U
#endif

/**
 * Yaw-rate magnitude at or below which all four wheel speeds are averaged
 * [mrad/s]; 50 = 0.050 rad/s.
 */
#ifndef TV_CONFIG_STRAIGHT_YAW_MRADPS
#define TV_CONFIG_STRAIGHT_YAW_MRADPS 50U
#endif

/**
 * Relative front/rear disagreement that flags wheel slip [permille of the
 * rear-projected front-axle speed]; 100 = 10%.
 */
#ifndef TV_CONFIG_SLIP_SPEED_PERMILLE
#define TV_CONFIG_SLIP_SPEED_PERMILLE 100U
#endif

/**
 * Minimum absolute front/rear disagreement that can flag wheel slip [mm/s];
 * keeps sensor noise near standstill from raising false positives.
 */
#ifndef TV_CONFIG_SLIP_MIN_MMPS
#define TV_CONFIG_SLIP_MIN_MMPS 300U
#endif

#endif
