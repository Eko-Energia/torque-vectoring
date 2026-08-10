# Integer vs double comparison

Both compiled C implementations were compared over this grid:

- signed rack displacement: every integer from `5` to `70 mm`,
- speed: `0` to `8000 mm/s` in `10 mm/s` steps,
- pedal: `0, 1, 16, 32, 64, 96, 128, 160, 192, 224, 255, 256`.

Total: `1,268,784` control cases.

| Metric | Result |
|---|---:|
| Grip/status disagreements | 0 |
| Maximum wheel-command difference | 1 command unit |
| Cases with any command difference | 0.457% of valid cases |
| Mean absolute difference per wheel | 0.00332 command unit |
| Maximum radius rounding error | 0.5 mm |
| Maximum relative radius error | 0.00572% |
| Maximum lateral-acceleration error | 0.885 mm/s² |
| Mean lateral-acceleration error | 0.253 mm/s² |

The largest command error is one least-significant output unit, so the integer
implementation is effectively equivalent at the configured `0–256` resolution.
