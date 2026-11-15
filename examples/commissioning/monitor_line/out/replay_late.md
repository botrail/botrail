# Monitor — `PLC1` from log line_io.csv

- model time: 19.73 s, tolerance 0.10 s
- drives the model (5): `auto.step`, `conv`, `feeder`, `lamp_lot`, `lamp_run`
- compared (4): `s1`, `s2`, `s_in`, `s_out`
- followed (6): `pb_jog`, `pb_reset`, `pb_start`, `pb_step`, `pb_stop`, `sw_step`

## Deviations (4)

| lane | kind | from | to | |
|---|---|---|---|---|
| `s2` | late | 4.36 | 4.86 | `s2` rose 0.50 s later on the machine than in the model (4.36 → 4.86 s) |
| `s2` | late | 5.52 | 6.02 | `s2` fell 0.50 s later on the machine than in the model (5.52 → 6.02 s) |
| `s2` | late | 11.05 | 11.55 | `s2` rose 0.50 s later on the machine than in the model (11.05 → 11.55 s) |
| `s2` | late | 12.21 | 12.71 | `s2` fell 0.50 s later on the machine than in the model (12.21 → 12.71 s) |
