import { samplePoses } from "../playback";
import { useStudioStore } from "../store";
import { sendPlanRequest } from "../ws";

/** Goal capture, plan request, and trajectory preview playback. */
export function PlanPanel() {
  const sceneDesc = useStudioStore((s) => s.sceneDesc);
  const connected = useStudioStore((s) => s.connection === "connected");
  const goal = useStudioStore((s) => s.goal);
  const planning = useStudioStore((s) => s.planning);
  const planError = useStudioStore((s) => s.planError);
  const planStats = useStudioStore((s) => s.planStats);
  const trajectory = useStudioStore((s) => s.trajectory);
  const segmentEnds = useStudioStore((s) => s.segmentEnds);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const playing = useStudioStore((s) => s.playing);
  const setGoalFromCurrent = useStudioStore((s) => s.setGoalFromCurrent);
  const clearGoal = useStudioStore((s) => s.clearGoal);
  const beginPlanning = useStudioStore((s) => s.beginPlanning);
  const setPlayback = useStudioStore((s) => s.setPlayback);
  const setPlaying = useStudioStore((s) => s.setPlaying);

  if (!sceneDesc) return null;

  const onPlan = () => {
    if (!goal) return;
    beginPlanning();
    sendPlanRequest(goal.positions);
  };

  const onTogglePlay = () => {
    if (!trajectory) return;
    // "At the end" with a slider-step tolerance: scrubbing snaps the time
    // to a 0.01s grid, so an exact comparison would replay ~one frame.
    if (!playing && playbackTime >= trajectory.duration - 0.02) {
      setPlayback(0, samplePoses(trajectory, 0));
    }
    setPlaying(!playing);
  };

  const onScrub = (t: number) => {
    if (!trajectory) return;
    setPlaying(false);
    setPlayback(t, samplePoses(trajectory, t));
  };

  return (
    <section className="panel-section">
      <div className="panel-head">
        <h2>Plan</h2>
        {planning && <span className="badge muted">planning…</span>}
        {!planning && planStats && trajectory && (
          <span className="badge ok">
            {trajectory.duration.toFixed(2)}s · {planStats.waypoints} wp ·{" "}
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
        {goal && !trajectory && !planning && !planError && (
          <div className="plan-hint">
            goal set — pose the robot at the start, then Plan
          </div>
        )}
        {trajectory && (
          <div className="playback-row">
            <button onClick={onTogglePlay}>{playing ? "❚❚" : "▶"}</button>
            <div className="seek">
              <input
                type="range"
                min={0}
                max={trajectory.duration}
                step={0.01}
                value={playbackTime}
                onChange={(e) => onScrub(parseFloat(e.target.value))}
              />
              {segmentEnds.length > 0 && trajectory.duration > 0 && (
                <div className="seek-marks">
                  {segmentEnds.map((t, i) => (
                    <div
                      key={i}
                      className="seek-mark"
                      style={{ left: `${(t / trajectory.duration) * 100}%` }}
                    />
                  ))}
                </div>
              )}
            </div>
            <span className="playback-time">
              {playbackTime.toFixed(2)}/{trajectory.duration.toFixed(2)}s
            </span>
          </div>
        )}
      </div>
    </section>
  );
}
