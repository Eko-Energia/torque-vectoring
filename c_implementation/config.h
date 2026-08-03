#ifndef TV_CONFIG_H
#define TV_CONFIG_H

/* Shared integer range for pedal input and rear-wheel output commands. */
#ifndef TV_CONFIG_COMMAND_MIN
#define TV_CONFIG_COMMAND_MIN 0
#endif

#ifndef TV_CONFIG_COMMAND_MAX
#define TV_CONFIG_COMMAND_MAX 256
#endif

/* Valid absolute rack displacement covered by the steering calibration. */
#define TV_CONFIG_RACK_MIN_MM 5.0
#define TV_CONFIG_RACK_MAX_MM 70.0

/* Fit from the measured rack-displacement/turn-radius curve:
   radius_mm = A * |rack_mm|^EXPONENT. */
#define TV_CONFIG_RACK_RADIUS_A 554462.0
#define TV_CONFIG_RACK_RADIUS_EXPONENT (-1.038)

/* Reserved for correction after vehicle testing. */
#define TV_CONFIG_RADIUS_CORRECTION_SCALE 1.0
#define TV_CONFIG_RADIUS_CORRECTION_OFFSET_M 0.0

#endif
