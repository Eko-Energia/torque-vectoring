#include "torque_vectoring.h"

#include <stdio.h>
#include <stdlib.h>

static void print_usage(const char *program, int command_min, int command_max)
{
    fprintf(stderr,
            "Usage: %s [rack_mm speed_mps pedal_%d_to_%d]\n"
            "       %s speed_mps pedal_%d_to_%d  (rack unavailable: equal split)\n",
            program, command_min, command_max,
            program, command_min, command_max);
}

int main(int argc, char **argv)
{
    double rack_displacement_mm = 0.0;
    double speed_mps = 5.0;
    const VehicleParameters vehicle = tv_default_vehicle();
    long pedal_command = vehicle.command_min +
                         (vehicle.command_max - vehicle.command_min) / 2;
    bool rack_position_available = false;

    if (argc == 4) {
        char *rack_end = NULL;
        char *speed_end = NULL;
        char *pedal_end = NULL;
        rack_displacement_mm = strtod(argv[1], &rack_end);
        speed_mps = strtod(argv[2], &speed_end);
        pedal_command = strtol(argv[3], &pedal_end, 10);
        rack_position_available = true;
        if (*argv[1] == '\0' || *rack_end != '\0' ||
            *argv[2] == '\0' || *speed_end != '\0' ||
            *argv[3] == '\0' || *pedal_end != '\0' ||
            pedal_command < vehicle.command_min ||
            pedal_command > vehicle.command_max) {
            print_usage(argv[0], vehicle.command_min, vehicle.command_max);
            return EXIT_FAILURE;
        }
    } else if (argc == 3) {
        char *speed_end = NULL;
        char *pedal_end = NULL;
        speed_mps = strtod(argv[1], &speed_end);
        pedal_command = strtol(argv[2], &pedal_end, 10);
        if (*argv[1] == '\0' || *speed_end != '\0' ||
            *argv[2] == '\0' || *pedal_end != '\0' ||
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
        &vehicle, rack_position_available, rack_displacement_mm,
        speed_mps, (int)pedal_command);

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
    printf("Rear left command:  %d\n", commands.rear_left);
    printf("Rear right command: %d\n", commands.rear_right);
    return EXIT_SUCCESS;
}
