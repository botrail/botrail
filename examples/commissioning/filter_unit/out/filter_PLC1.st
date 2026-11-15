(* botrail — Structured Text for host `PLC1`: `filter` *)
(* Variables are the I/O map's points of this host; addresses are the bindings on its PLC-family nodes. *)
(* Robot commands: raise `start` once `done` reads high, hold it until `done` reads low, drop it, then wait for `done`; program words select the exchange's numbered entries. *)

PROGRAM filter
  VAR_EXTERNAL
    alarm : BOOL;
    dp : INT;
    dp_warn : INT;
    lamp_run : BOOL;
    pb_reset : BOOL;
    pb_start : BOOL;
    pb_stop : BOOL;
    pump : BOOL;
    valve_extend : BOOL;
    valve_extended : BOOL;
    valve_time : INT;
    warn : BOOL;
  END_VAR
  VAR
    first : BOOL := TRUE;
    alarm_dp : BOOL; (* relay *)
    alarm_valve : BOOL; (* relay *)
    alarm_wire : BOOL; (* relay *)
    run : BOOL; (* relay *)
    dp_limit : REAL := 80.0; (* word *)
    dp_peak : REAL := 0.0; (* word *)
    t_valve : INT := 0; (* word *)
    dp_eu : REAL; (* scaled *)
  END_VAR

  (* analog inputs: the module's counts scaled to their unit *)
  dp_eu := 0.0 + (INT_TO_REAL(dp) - 0.0) * 100.0 / 4000.0; (* 0 … 100 kPa over 0 … 4000 counts *)
  (* first scan: enter the initial step *)
  IF first THEN
      first := FALSE;
  END_IF;
  (* transitions, in authored order *)
  (* rungs, every scan, top to bottom *)
  (* run (self-hold) *)
  run := ((pb_start OR run) AND NOT (pb_stop) AND NOT (alarm));
  (* outlet valve *)
  valve_extend := run;
  (* pump: behind the valve's open switch *)
  pump := (run AND valve_extended);
  (* change the element soon *)
  warn := (run AND (dp_eu >= (INT_TO_REAL(dp_warn) / 10.0)));
  (* element clogged *)
  IF (dp_eu >= dp_limit) THEN alarm_dp := TRUE; END_IF;
  (* transmitter wire broken *)
  IF (dp < -100) THEN alarm_wire := TRUE; END_IF;
  (* valve watchdog: 10 ms a scan *)
  IF (run AND NOT (valve_extended)) THEN t_valve := t_valve + 10; END_IF;
  (* valve watchdog cleared *)
  IF NOT ((run AND NOT (valve_extended))) THEN t_valve := 0; END_IF;
  (* valve did not open in time *)
  IF ((run AND NOT (valve_extended)) AND (t_valve >= valve_time)) THEN alarm_valve := TRUE; END_IF;
  (* alarm *)
  alarm := (alarm_dp OR alarm_valve OR alarm_wire);
  (* alarm reset *)
  IF pb_reset THEN alarm_dp := FALSE; END_IF;
  IF pb_reset THEN alarm_valve := FALSE; END_IF;
  IF pb_reset THEN alarm_wire := FALSE; END_IF;
  (* peak hold *)
  IF (dp_eu > dp_peak) THEN dp_peak := dp_eu; END_IF;
  (* running lamp *)
  lamp_run := run;
END_PROGRAM

CONFIGURATION Config0
  RESOURCE Res0 ON PLC
    VAR_GLOBAL
      alarm AT %QX100.0 : BOOL; (* alarm signal:handshake — the controller writes *)
      dp AT %IW100 : INT; (* dp analog — the controller reads *)
      dp_warn AT %IW103 : INT; (* dp_warn signal:handshake — the controller reads; INT at a resolution of 0.1: 10 counts per unit *)
      lamp_run AT %QX100.1 : BOOL; (* lamp_run signal:write-only — the controller writes *)
      pb_reset AT %IX100.0 : BOOL; (* pb_reset signal:handshake — the controller reads *)
      pb_start AT %IX100.1 : BOOL; (* pb_start signal:handshake — the controller reads *)
      pb_stop AT %IX100.2 : BOOL; (* pb_stop signal:handshake — the controller reads *)
      pump AT %QX100.2 : BOOL; (* pump signal:write-only — the controller writes *)
      valve_extend AT %QX100.3 : BOOL; (* valve/extend device:coil — the controller writes *)
      valve_extended AT %IX100.3 : BOOL; (* valve/extended sensor — the controller reads *)
      valve_time AT %IW102 : INT; (* valve_time signal:read-only — the controller reads *)
      warn AT %QX100.4 : BOOL; (* warn signal:handshake — the controller writes *)
    END_VAR
    TASK Main(INTERVAL := T#10ms, PRIORITY := 0);
    PROGRAM filter_inst WITH Main : filter;
  END_RESOURCE
END_CONFIGURATION
