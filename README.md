# Torque Vectoring in C

**English** · [Polski](README_PL.md)

The program distributes the driver's request between the left and right rear
wheels. Its inputs are:

- steering-rack displacement `[mm]` — positive left and negative right,
- vehicle speed `[m/s]`,
- integer pedal input in the `0–256` range.

It returns two integer commands: rear-left and rear-right. The flow is simple:
convert rack displacement to turn radius, calculate lateral load distribution,
then scale the pedal request separately for both wheels.

If rack position is unavailable or exactly `0 mm`, fallback mode sends the pedal
value directly to both wheels. Torque vectoring is disabled in that case.

Base vehicle parameters are: mass `850 kg`, centre-of-mass height `0.511 m`,
longitudinal offset from the wheelbase midpoint `-0.040 m`, wheelbase `2.750 m`,
track width `1.700 m`, and tyre friction coefficient `0.8`.

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
The same file contains coefficients reserved for radius correction after testing:

```c
#define TV_CONFIG_RADIUS_CORRECTION_SCALE 1.0
#define TV_CONFIG_RADIUS_CORRECTION_OFFSET_M 0.0
```

## Run

```sh
make
./build/torque-vectoring 70 5.0 128
make test
```

When rack position is unavailable, provide only speed and pedal input:

```sh
./build/torque-vectoring 5.0 128
```

Example output for a left turn:

```text
Torque vectoring:   active
Rear left command:  68
Rear right command: 188
```

## The problem

![Vehicle cornering diagram and wheel-path radii](old/Screenshot%202026-03-09%20083548.png)

In a turn, the outer wheel travels farther and carries more vertical load. The
inner wheel is unloaded. The two wheels should therefore not always receive the
same command.

## Calculation

Rack displacement `x` is converted using a continuous function fitted to the
measured curve:

```text
R_mm = 554462 · |x_mm|⁻¹·⁰³⁸
```

The sign of `x` selects the turn direction. Input remains limited to `5–70 mm`;
values outside this range return `TV_RACK_OUT_OF_RANGE`. Source measurements:

| Displacement [mm] | Radius [mm] | Displacement [mm] | Radius [mm] |
|---:|---:|---:|---:|
| 5 | 102525 | 40 | 12167 |
| 10 | 50341 | 45 | 10819 |
| 15 | 33411 | 50 | 9660 |
| 20 | 24958 | 55 | 8687 |
| 25 | 19851 | 60 | 7858 |
| 30 | 16454 | 65 | 7120 |
| 35 | 14014 | 70 | 6492 |

The same function, measurements, and an executable example of the algorithm are
included in `old/analiz.ipynb`.

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
circle, transient dynamics, motor characteristics, suspension-geometry changes,
or a measured tyre model. Expected understeer is not modelled separately at this
stage. After vehicle testing, radius can be corrected with
`TV_CONFIG_RADIUS_CORRECTION_SCALE` and `TV_CONFIG_RADIUS_CORRECTION_OFFSET_M`.
