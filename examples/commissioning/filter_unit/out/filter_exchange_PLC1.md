# Exchange — host `PLC1`

Programs on this host: `filter`

## Inputs — the controller writes (5)

| point | variable | type | source | address | tag | wire | initial |
|---|---|---|---|---|---|---|---|
| `alarm` | `alarm` | BOOL | signal:handshake | %QX100.0 |  |  | False |
| `lamp_run` | `lamp_run` | BOOL | signal:write-only | %QX100.1 |  |  | False |
| `pump` | `pump` | BOOL | signal:write-only | %QX100.2 |  |  | False |
| `valve/extend` | `valve_extend` | BOOL | device:coil | %QX100.3 |  |  | False |
| `warn` | `warn` | BOOL | signal:handshake | %QX100.4 |  |  | False |

## Outputs — the controller reads (7)

| point | variable | type | source | address | tag | wire | initial |
|---|---|---|---|---|---|---|---|
| `dp` | `dp` | INT | analog | %IW100 | DPT1 |  | 0 |
| `dp_warn` | `dp_warn` | INT (0.1) | signal:handshake | %IW103 | DpWarn |  | 600 |
| `pb_reset` | `pb_reset` | BOOL | signal:handshake | %IX100.0 |  |  | False |
| `pb_start` | `pb_start` | BOOL | signal:handshake | %IX100.1 |  |  | False |
| `pb_stop` | `pb_stop` | BOOL | signal:handshake | %IX100.2 |  |  | False |
| `valve/extended` | `valve_extended` | BOOL | sensor | %IX100.3 |  |  | False |
| `valve_time` | `valve_time` | INT | signal:read-only | %IW102 | ValveTime |  | 600 |

- `dp_warn`: INT at a resolution of 0.1: 10 counts per unit

Internal relays (never exchanged): `alarm_dp`, `alarm_valve`, `alarm_wire`, `dp_limit`, `dp_peak`, `run`, `t_valve`
