# Torque Vectoring in C

**English** · [Polski](README_PL.md)

The program distributes the driver's request between the left and right rear
wheels. Its inputs are:

- turn radius `[m]` — positive for a left turn and negative for a right turn,
- vehicle speed `[m/s]`,
- integer pedal input in the `0–256` range.

It returns two integer commands: rear-left and rear-right. The flow is simple:
read turn radius, speed, and pedal input; calculate lateral load distribution;
then scale the pedal request separately for both wheels.

## Command-range configuration

By default, pedal input and wheel commands share the `0–256` range. Change it in
`c_implementation/config.h`:

```c
#define TV_CONFIG_COMMAND_MIN 0
#define TV_CONFIG_COMMAND_MAX 100
```

After this change, pedal input and both outputs use `0–100`. `command_min`
represents no requested torque and `command_max` represents full demand. Values
outside the configured range return `TV_INVALID_ARGUMENT`.

## Run

```sh
make
./build/torque-vectoring 6.5 5.0 128
make test
```

Example output for a left turn:

```text
Rear left command:  66
Rear right command: 190
```

## The problem

![Vehicle cornering diagram and wheel-path radii](old/Screenshot%202026-03-09%20083548.png)

In a turn, the outer wheel travels farther and carries more vertical load. The
inner wheel is unloaded. The two wheels should therefore not always receive the
same command.

## Calculation

Lateral acceleration and force are:

```text
a_y = v² / |R|
F_y = m · a_y
```

Approximate lateral load transfer is:

```text
ΔF_z = F_y · h / t
F_z,inner = F_z,rear/2 - ΔF_z
F_z,outer = F_z,rear/2 + ΔF_z
```

The program derives each wheel's share from its available torque
`μ · F_z · r_w`. Pedal input is the baseline command for each wheel when driving
straight. In a turn, the inner command decreases and the outer command increases.
The result is rounded and clamped to the configured range. If `F_y > μmg`, both
commands are set to `command_min`.

If one command would exceed `command_max` under heavy pedal input, the program
scales **both** values by the same factor. This preserves the calculated ratio
between the wheels. Total requested torque is reduced because keeping the output
in range while preserving the torque split takes priority.

## Files

- `c_implementation/torque_vectoring.h` — types and public functions,
- `c_implementation/config.h` — input and output range,
- `c_implementation/torque_vectoring.c` — calculations,
- `c_implementation/main.c` — program input and output,
- `c_implementation/test_torque_vectoring.c` — tests.

This is a simplified educational model. It does not include the tyre friction
circle, transient dynamics, motor characteristics, or wheel slip.
