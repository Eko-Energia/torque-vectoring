#ifndef TV_CONFIG_H
#define TV_CONFIG_H

#ifndef TV_CONFIG_COMMAND_MIN
#define TV_CONFIG_COMMAND_MIN 0
#endif

#ifndef TV_CONFIG_COMMAND_MAX
#define TV_CONFIG_COMMAND_MAX 256
#endif

#define TV_CONFIG_RACK_MIN_MM 5U
#define TV_CONFIG_RACK_MAX_MM 70U

/* R_m = 507 / |rack_mm|. The constant has units m*mm. */
#define TV_CONFIG_RACK_RADIUS_CONSTANT_M_MM 507U

/* Integer correction: radius = radius * permille / 1000 + offset_mm. */
#define TV_CONFIG_RADIUS_CORRECTION_PERMILLE 1000U
#define TV_CONFIG_RADIUS_CORRECTION_OFFSET_MM 0

/* Bounds intermediate products and rejects implausible sensor values. */
#define TV_CONFIG_MAX_SPEED_MMPS 100000U

#endif
