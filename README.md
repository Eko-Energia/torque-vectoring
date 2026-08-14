# Torque Vectoring in C

**English** · [Polski](README_PL.md)

The program distributes the driver's request between the left and right rear
wheels. It does not set the steering-rack position. On every call it **receives
the current steering-rack sensor reading** and information indicating whether
that reading is available. Its inputs are:

- steering-rack displacement `[mm]` — positive left and negative right,
- integer vehicle speed `[mm/s]`,
- integer pedal input in the `0–256` range.

It returns two integer commands: rear-left and rear-right. The flow is simple:
convert rack displacement to turn radius, calculate lateral load distribution,
then scale the pedal request separately for both wheels.

The rack sensor input is interpreted as follows:

| Data received from the sensor | Program behaviour |
|---|---|
| no current reading | send the pedal value to both wheels |
| exactly `0 mm` | straight driving: send the pedal value to both wheels |
| `+1` to `+70 mm` | left turn with active torque vectoring |
| `-1` to `-70 mm` | right turn with active torque vectoring |
| beyond `±70 mm` | reading outside the supported range; return an error |

Exactly `0 mm` is a separate, valid reading meaning that the rack is centred.
Values in the `±1–4 mm` range are also calculated. Because measured calibration
points begin at `5 mm`, this region uses an extrapolation of `R = 507 / |x|`.
This keeps the torque split continuous near straight-line driving.

Base vehicle parameters are: mass `850 kg`, centre-of-mass height `0.511 m`,
longitudinal offset from the wheelbase midpoint `-0.040 m`, wheelbase `2.750 m`,
track width `1.700 m`, and tyre friction coefficient `0.8`.

<p align="center">
  <img src="docs/vehicle_geometry.svg" width="900" alt="Perła geometry: wheelbase, track width, and centre-of-mass position">
</p>

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
#define TV_CONFIG_RADIUS_CORRECTION_PERMILLE 1000U
#define TV_CONFIG_RADIUS_CORRECTION_OFFSET_MM 0
```

`1000` means a `1.000` multiplier; for example, `980` means `0.980`.

## Run

```sh
make
./build/torque-vectoring 70 5000 128
make test
```

When rack position is unavailable, provide only speed and pedal input:

```sh
./build/torque-vectoring 5000 128
```

Example output for a left turn:

```text
Torque vectoring:   active
Rear left command:  72
Rear right command: 184
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
R_m = 507 / |x_mm|
```

The integer implementation uses the equivalent `R_mm = 507000 / |x_mm|`.

The sign of `x` selects the turn direction. The supported non-zero input range is
`1–70 mm`; larger values return `TV_RACK_OUT_OF_RANGE`. The curve is extrapolated
for `1–4 mm`. Source measurements begin at `5 mm`:

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

## STM32 implementation

The real-time control path uses no `float`, `double`, `pow()`, `hypot()`, or
`libm`. Inputs and outputs use 32-bit types, while intermediate products use
64-bit integers to prevent overflow. Units are `mm`, `mm/s`, `mm/s²`, and
permille. Division is rounded to the nearest integer.

Mass, friction coefficient, and wheel radius cancel when calculating the torque
ratio. Friction is still used to detect the lateral-grip limit. Maximum accepted
speed is configured with `TV_CONFIG_MAX_SPEED_MMPS`.

The Polish STM32CubeIDE integration guide, unit table, and control-loop example
are available in [`docs/STM32_INTEGRATION_PL.md`](docs/STM32_INTEGRATION_PL.md).
The current path uses neither `math.h` nor `arm_math`; CMSIS-DSP is not required.

Public types and functions include Doxygen comments. Generate HTML documentation
with `doxygen Doxyfile`.

The previous `double` implementation is archived in
`old/c_implementation_double/`. A comparison of `1,268,784` cases found no grip
status disagreements. Maximum output difference was one command unit, and the
maximum relative radius error was `0.00572%`. Full results are in
`old/c_implementation_double/COMPARISON.md`.

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
`TV_CONFIG_RADIUS_CORRECTION_PERMILLE` and `TV_CONFIG_RADIUS_CORRECTION_OFFSET_MM`.
