#include "torque_vectoring.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/** @brief Prints the desktop demonstration program syntax. */
static void print_usage(
    const char *program,
    int32_t command_min,
    int32_t command_max)
{
    fprintf(stderr,
            "Usage: %s [rack_mm speed_mmps pedal_%ld_to_%ld]\n"
            "       %s speed_mmps pedal_%ld_to_%ld  (rack unavailable: equal split)\n",
            program, (long)command_min, (long)command_max,
            program, (long)command_min, (long)command_max);
}

/**
 * @brief Desktop CLI demonstrating the same integer API used on STM32.
 * @return EXIT_SUCCESS on a valid calculation, otherwise EXIT_FAILURE.
 */
int main(int argc, char **argv)
{
    long rack_displacement_mm = 0;
    unsigned long speed_mmps = 5000U;
    const VehicleParameters vehicle = tv_default_vehicle();
    long pedal_command = vehicle.command_min +
                         (vehicle.command_max - vehicle.command_min) / 2;
    bool rack_position_available = false;

    if (argc == 4) {
        char *rack_end = NULL;
        char *speed_end = NULL;
        char *pedal_end = NULL;
        rack_displacement_mm = strtol(argv[1], &rack_end, 10);
        speed_mmps = strtoul(argv[2], &speed_end, 10);
        pedal_command = strtol(argv[3], &pedal_end, 10);
        rack_position_available = true;
        if (*argv[1] == '\0' || *rack_end != '\0' ||
            *argv[2] == '\0' || *speed_end != '\0' ||
            *argv[3] == '\0' || *pedal_end != '\0' ||
            rack_displacement_mm < INT32_MIN ||
            rack_displacement_mm > INT32_MAX || speed_mmps > UINT32_MAX ||
            pedal_command < vehicle.command_min ||
            pedal_command > vehicle.command_max) {
            print_usage(argv[0], vehicle.command_min, vehicle.command_max);
            return EXIT_FAILURE;
        }
    } else if (argc == 3) {
        char *speed_end = NULL;
        char *pedal_end = NULL;
        speed_mmps = strtoul(argv[1], &speed_end, 10);
        pedal_command = strtol(argv[2], &pedal_end, 10);
        if (*argv[1] == '\0' || *speed_end != '\0' ||
            *argv[2] == '\0' || *pedal_end != '\0' ||
            speed_mmps > UINT32_MAX ||
            pedal_command < vehicle.command_min ||
            pedal_command > vehicle.command_max) {
            print_usage(argv[0], vehicle.command_min, vehicle.command_max);
            return EXIT_FAILURE;
        }
    } else if (argc != 1) {
        print_usage(argv[0], vehicle.command_min, vehicle.command_max);
        return EXIT_FAILURE;
    }

    const WheelCommands commands = tv_calculate_rear_commands_from_rack(
        &vehicle, rack_position_available, (int32_t)rack_displacement_mm,
        (uint32_t)speed_mmps, (int32_t)pedal_command);

    if (commands.status == TV_LATERAL_GRIP_EXCEEDED) {
        puts("Lateral grip exceeded: reduce speed.");
        return EXIT_SUCCESS;
    }
    if (commands.status != TV_OK) {
        fprintf(stderr, "Calculation failed: %s.\n", tv_status_string(commands.status));
        return EXIT_FAILURE;
    }

    printf("Torque vectoring:   %s\n",
           commands.torque_vectoring_active ? "active" : "fallback (equal split)");
    printf("Rear left command:  %" PRId32 "\n", commands.rear_left);
    printf("Rear right command: %" PRId32 "\n", commands.rear_right);
    return EXIT_SUCCESS;
}
