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

#endif
