# Exchange — host `PLC1`

Programs on this host: `common`, `auto`, `outputs`

## Inputs — the controller writes (6)

| point | variable | type | source | address | tag | wire | initial |
|---|---|---|---|---|---|---|---|
| `auto.step` | `auto_step` | INT | monitor | %QW100 |  |  | -1 |
| `conv` | `conv_run` | BOOL | device:run | %QX100.0 |  |  | False |
| `feeder` | `feeder_run` | BOOL | device:run | %QX100.1 |  |  | False |
| `lamp_lot` | `lamp_lot` | BOOL | signal:write-only | %QX100.2 |  |  | False |
| `lamp_run` | `lamp_run` | BOOL | signal:write-only | %QX100.3 |  |  | False |
| `run` | `run` | BOOL | signal:handshake | %QX100.4 |  |  | False |

## Outputs — the controller reads (10)

| point | variable | type | source | address | tag | wire | initial |
|---|---|---|---|---|---|---|---|
| `pb_jog` | `pb_jog` | BOOL | signal:handshake | %IX100.0 |  |  | False |
| `pb_reset` | `pb_reset` | BOOL | signal:handshake | %IX100.1 |  |  | False |
| `pb_start` | `pb_start` | BOOL | signal:handshake | %IX100.2 |  |  | False |
| `pb_step` | `pb_step` | BOOL | signal:handshake | %IX100.3 |  |  | False |
| `pb_stop` | `pb_stop` | BOOL | signal:handshake | %IX100.4 |  |  | False |
| `s1` | `s1` | BOOL | sensor | %IX100.5 |  |  | False |
| `s2` | `s2` | BOOL | sensor | %IX100.6 |  |  | False |
| `s_in` | `s_in` | BOOL | sensor | %IX100.7 |  |  | False |
| `s_out` | `s_out` | BOOL | sensor | %IX101.0 |  |  | False |
| `sw_step` | `sw_step` | BOOL | signal:handshake | %IX101.1 |  |  | False |

Internal relays (never exchanged): `advance`, `made`, `stop_done`, `stop_req`, `t_dwell1`, `t_dwell2`
