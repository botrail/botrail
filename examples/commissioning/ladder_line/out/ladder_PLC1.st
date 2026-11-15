(* botrail — Structured Text for host `PLC1`: `common`, `auto`, `outputs` *)
(* Variables are the I/O map's points of this host; addresses are the bindings on its PLC-family nodes. *)
(* Robot commands: raise `start` once `done` reads high, hold it until `done` reads low, drop it, then wait for `done`; program words select the exchange's numbered entries. *)

PROGRAM common
  VAR_EXTERNAL
    advance : BOOL;
    auto_at_wait : BOOL;
    pb_start : BOOL;
    pb_step : BOOL;
    pb_stop : BOOL;
    run : BOOL;
    stop_done : BOOL;
    stop_req : BOOL;
    sw_step : BOOL;
  END_VAR
  VAR
    first : BOOL := TRUE;
    r_pb_step : R_TRIG;
  END_VAR

  (* edges, sampled every scan *)
  r_pb_step(CLK := pb_step);
  (* first scan: enter the initial step *)
  IF first THEN
      first := FALSE;
  END_IF;
  (* transitions, in authored order *)
  (* rungs, every scan, top to bottom *)
  (* cycle stop: at the end of the cycle *)
  stop_done := (stop_req AND auto_at_wait);
  (* run (self-hold) *)
  run := ((pb_start OR run) AND NOT (stop_done));
  (* cycle stop asked *)
  IF (pb_stop AND run) THEN stop_req := TRUE; END_IF;
  (* stop taken *)
  IF NOT (run) THEN stop_req := FALSE; END_IF;
  (* advance (step mode: one step per press) *)
  advance := (run AND (NOT (sw_step) OR r_pb_step.Q));
END_PROGRAM

PROGRAM auto
  VAR_EXTERNAL
    advance : BOOL;
    auto_at_clear : BOOL;
    auto_at_discharge : BOOL;
    auto_at_dwell_1 : BOOL;
    auto_at_dwell_2 : BOOL;
    auto_at_feed_in : BOOL;
    auto_at_to_st1 : BOOL;
    auto_at_to_st2 : BOOL;
    auto_at_wait : BOOL;
    auto_step : INT;
    pb_reset : BOOL;
    run : BOOL;
    s1 : BOOL;
    s2 : BOOL;
    s_in : BOOL;
    s_out : BOOL;
    stop_req : BOOL;
    t_dwell1 : BOOL;
    t_dwell2 : BOOL;
  END_VAR
  VAR
    first : BOOL := TRUE;
    s_wait : BOOL := TRUE; (* initial step *)
    s_feed_in : BOOL;
    s_to_st1 : BOOL;
    s_dwell_1 : BOOL;
    s_to_st2 : BOOL;
    s_dwell_2 : BOOL;
    s_discharge : BOOL;
    s_clear : BOOL;
  END_VAR

  (* first scan: enter the initial step *)
  IF first THEN (* 2 *)
      first := FALSE;
      auto_step := 0;
  END_IF;
  (* reset: back to — and held in — the first step; else the transitions, in authored order *)
  IF pb_reset THEN
      IF NOT s_wait THEN
          s_feed_in := FALSE;
          s_to_st1 := FALSE;
          s_dwell_1 := FALSE;
          s_to_st2 := FALSE;
          s_dwell_2 := FALSE;
          s_discharge := FALSE;
          s_clear := FALSE;
          s_wait := TRUE;
          auto_step := 0;
      END_IF;
  ELSE
      IF s_wait AND (run AND NOT (stop_req)) AND advance THEN
          s_wait := FALSE;
          s_feed_in := TRUE;
          auto_step := 1;
      END_IF;
      IF s_feed_in AND s_in AND advance THEN
          s_feed_in := FALSE;
          s_to_st1 := TRUE;
          auto_step := 2;
      END_IF;
      IF s_to_st1 AND s1 AND advance THEN
          s_to_st1 := FALSE;
          s_dwell_1 := TRUE;
          auto_step := 3;
      END_IF;
      IF s_dwell_1 AND t_dwell1 AND advance THEN
          s_dwell_1 := FALSE;
          s_to_st2 := TRUE;
          auto_step := 4;
      END_IF;
      IF s_to_st2 AND s2 AND advance THEN
          s_to_st2 := FALSE;
          s_dwell_2 := TRUE;
          auto_step := 5;
      END_IF;
      IF s_dwell_2 AND t_dwell2 AND advance THEN
          s_dwell_2 := FALSE;
          s_discharge := TRUE;
          auto_step := 6;
      END_IF;
      IF s_discharge AND s_out AND advance THEN
          s_discharge := FALSE;
          s_clear := TRUE;
          auto_step := 7;
      END_IF;
      IF s_clear AND NOT (s_out) AND advance THEN
          s_clear := FALSE;
          s_wait := TRUE; (* cycle *)
          auto_step := 0;
      END_IF;
  END_IF;
  auto_at_wait := s_wait; (* read by another program *)
  auto_at_to_st1 := s_to_st1; (* read by another program *)
  auto_at_to_st2 := s_to_st2; (* read by another program *)
  auto_at_discharge := s_discharge; (* read by another program *)
  auto_at_clear := s_clear; (* read by another program *)
  auto_at_feed_in := s_feed_in; (* read by another program *)
  auto_at_dwell_1 := s_dwell_1; (* read by another program *)
  auto_at_dwell_2 := s_dwell_2; (* read by another program *)
END_PROGRAM

PROGRAM outputs
  VAR_EXTERNAL
    auto_at_clear : BOOL;
    auto_at_discharge : BOOL;
    auto_at_dwell_1 : BOOL;
    auto_at_dwell_2 : BOOL;
    auto_at_feed_in : BOOL;
    auto_at_to_st1 : BOOL;
    auto_at_to_st2 : BOOL;
    conv_run : BOOL;
    feeder_run : BOOL;
    lamp_lot : BOOL;
    lamp_run : BOOL;
    made : BOOL;
    pb_jog : BOOL;
    pb_reset : BOOL;
    run : BOOL;
    s1 : BOOL;
    s2 : BOOL;
    t_dwell1 : BOOL;
    t_dwell2 : BOOL;
  END_VAR
  VAR
    first : BOOL := TRUE;
    t_t_dwell1 : TON;
    t_t_dwell2 : TON;
    c_made : CTU;
  END_VAR

  (* first scan: enter the initial step *)
  IF first THEN (* 3 *)
      first := FALSE;
  END_IF;
  (* transitions, in authored order *)
  (* rungs, every scan, top to bottom *)
  (* belt: the travel steps, or the manual jog *)
  conv_run := ((run AND ((auto_at_to_st1 AND NOT (s1)) OR (auto_at_to_st2 AND NOT (s2)) OR auto_at_discharge OR auto_at_clear)) OR (NOT (run) AND pb_jog));
  (* feeder: one work per cycle *)
  feeder_run := auto_at_feed_in;
  (* station 1 dwell *)
  t_t_dwell1(IN := auto_at_dwell_1, PT := T#800ms);
  t_dwell1 := t_t_dwell1.Q;
  (* station 2 dwell *)
  t_t_dwell2(IN := auto_at_dwell_2, PT := T#800ms);
  t_dwell2 := t_t_dwell2.Q;
  (* running lamp *)
  lamp_run := run;
  (* works made *)
  c_made(CU := auto_at_clear, R := pb_reset, PV := 2);
  made := c_made.Q;
  (* lot of two made *)
  lamp_lot := made;
  (* lot counter cleared with the steps *)
  (* resets counter c_made *)
END_PROGRAM

CONFIGURATION Config0
  RESOURCE Res0 ON PLC
    VAR_GLOBAL
      auto_step AT %QW100 : INT; (* auto.step monitor — the controller writes *)
      conv_run AT %QX100.0 : BOOL; (* conv device:run — the controller writes *)
      feeder_run AT %QX100.1 : BOOL; (* feeder device:run — the controller writes *)
      lamp_lot AT %QX100.2 : BOOL; (* lamp_lot signal:write-only — the controller writes *)
      lamp_run AT %QX100.3 : BOOL; (* lamp_run signal:write-only — the controller writes *)
      pb_jog AT %IX100.0 : BOOL; (* pb_jog signal:handshake — the controller reads *)
      pb_reset AT %IX100.1 : BOOL; (* pb_reset signal:handshake — the controller reads *)
      pb_start AT %IX100.2 : BOOL; (* pb_start signal:handshake — the controller reads *)
      pb_step AT %IX100.3 : BOOL; (* pb_step signal:handshake — the controller reads *)
      pb_stop AT %IX100.4 : BOOL; (* pb_stop signal:handshake — the controller reads *)
      run AT %QX100.4 : BOOL; (* run signal:handshake — the controller writes *)
      s1 AT %IX100.5 : BOOL; (* s1 sensor — the controller reads *)
      s2 AT %IX100.6 : BOOL; (* s2 sensor — the controller reads *)
      s_in AT %IX100.7 : BOOL; (* s_in sensor — the controller reads *)
      s_out AT %IX101.0 : BOOL; (* s_out sensor — the controller reads *)
      sw_step AT %IX101.1 : BOOL; (* sw_step signal:handshake — the controller reads *)
      advance : BOOL; (* relay *)
      auto_at_clear : BOOL; (* relay *)
      auto_at_discharge : BOOL; (* relay *)
      auto_at_dwell_1 : BOOL; (* relay *)
      auto_at_dwell_2 : BOOL; (* relay *)
      auto_at_feed_in : BOOL; (* relay *)
      auto_at_to_st1 : BOOL; (* relay *)
      auto_at_to_st2 : BOOL; (* relay *)
      auto_at_wait : BOOL; (* relay *)
      made : BOOL; (* relay *)
      stop_done : BOOL; (* relay *)
      stop_req : BOOL; (* relay *)
      t_dwell1 : BOOL; (* relay *)
      t_dwell2 : BOOL; (* relay *)
    END_VAR
    TASK Main(INTERVAL := T#10ms, PRIORITY := 0);
    PROGRAM common_inst WITH Main : common;
    PROGRAM auto_inst WITH Main : auto;
    PROGRAM outputs_inst WITH Main : outputs;
  END_RESOURCE
END_CONFIGURATION
