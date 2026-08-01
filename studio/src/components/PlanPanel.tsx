import { samplePlayback } from "../playback";
import { robotByName, useStudioStore } from "../store";
import { sendPlanRequest } from "../ws";

/** Goal capture, plan request, and trajectory preview playback. */
export function PlanPanel() {
  const robot = useStudioStore((s) => robotByName(s.robots, s.selectedRobot));
  const connected = useStudioStore((s) => s.connection === "connected");
  const goal = useStudioStore((s) => s.goal);
  const planning = useStudioStore((s) => s.planning);
  const planError = useStudioStore((s) => s.planError);
  const planStats = useStudioStore((s) => s.planStats);
  const playback = useStudioStore((s) => s.playback);
  const segmentEnds = useStudioStore((s) => s.segmentEnds);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const playing = useStudioStore((s) => s.playing);
  const setGoalFromCurrent = useStudioStore((s) => s.setGoalFromCurrent);
  const clearGoal = useStudioStore((s) => s.clearGoal);
  const beginPlanning = useStudioStore((s) => s.beginPlanning);
  const setPlayback = useStudioStore((s) => s.setPlayback);
  const setPlaying = useStudioStore((s) => s.setPlaying);

  if (!robot) return null;

  const onPlan = () => {
    if (!goal) return;
    beginPlanning();
    sendPlanRequest(goal.robot, goal.positions);
  };

  const onTogglePlay = () => {
    if (!playback) return;
    // "At the end" with a slider-step tolerance: scrubbing snaps the time
    // to a 0.01s grid, so an exact comparison would replay ~one frame.
    if (!playing && playbackTime >= playback.duration - 0.02) {
      setPlayback(0, samplePlayback(playback, 0));
    }
    setPlaying(!playing);
  };

  const onScrub = (t: number) => {
    if (!playback) return;
    setPlaying(false);
    setPlayback(t, samplePlayback(playback, t));
  };

  return (
    <section className="panel-section">
      <div className="panel-head">
        <h2>Plan</h2>
        {planning && <span className="badge muted">planning…</span>}
        {!planning && planStats && playback && (
          <span className="badge ok">
            {playback.duration.toFixed(2)}s · {planStats.waypoints} wp ·{" "}
            {planStats.planning_time_ms.toFixed(0)}ms
          </span>
        )}
      </div>
      <div className="plan-controls">
        <div className="seg">
          <button onClick={setGoalFromCurrent} disabled={!connected}>
            Set goal
          </button>
          <button onClick={clearGoal} disabled={!goal}>
            Clear
          </button>
          <button
            className="plan-go"
            onClick={onPlan}
            disabled={!goal || planning || !connected}
          >
            Plan
          </button>
        </div>
        {planError && <div className="plan-error">{planError}</div>}
        {goal && !playback && !planning && !planError && (
          <div className="plan-hint">
            goal set on {goal.robot} — pose the robot at the start, then Plan
          </div>
        )}
        {playback && (
          <div className="playback-row">
            <button onClick={onTogglePlay}>{playing ? "❚❚" : "▶"}</button>
            <div className="seek">
              <input
                type="range"
                min={0}
                max={playback.duration}
                step={0.01}
                value={playbackTime}
                onChange={(e) => onScrub(parseFloat(e.target.value))}
              />
              {segmentEnds.length > 0 && playback.duration > 0 && (
                <div className="seek-marks">
                  {segmentEnds.map((t, i) => (
                    <div
                      key={i}
                      className="seek-mark"
                      style={{ left: `${(t / playback.duration) * 100}%` }}
                    />
                  ))}
                </div>
              )}
            </div>
            <span className="playback-time">
              {playbackTime.toFixed(2)}/{playback.duration.toFixed(2)}s
            </span>
          </div>
        )}
      </div>
    </section>
  );
}
