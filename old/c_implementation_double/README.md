# Archived double-precision implementation

This is the last `double`-based implementation kept as a numerical reference.
The STM32-oriented implementation is in `../../c_implementation/` and uses only
integer arithmetic in its real-time control path.

Build and test this version independently:

```sh
make
make test
```

Its CLI accepts rack displacement in `mm`, speed in `m/s`, and the pedal command.
It requires `libm`.
