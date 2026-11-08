import { motionTarget, waypointSegment } from "../motions";
import { robotArms, robotByName, useStudioStore } from "../store";
import { sendAddSegment } from "../ws";
import { applyPlacement, stancePlacement } from "./locomotion";
import { type PanelAction, nextSpeed } from "./panel";
import { headPose, xrRig } from "./rig";
import { exitVr } from "./xrStore";

/**
 * Everything the headset's input does to the studio, in one place
 * (`.internal/design-vr.md` §3.1): the buttons and the panel both call
 * these, and a later mediator (design-studio-architecture.md) replaces
 * this file rather than the input code. Each is a store action the
 * desktop UI already has, under the same rules (no play without a bake).
 */
export const xrActions = {
  /** A / the panel's first button: play ⇄ pause the baked cycle. */
  togglePlay(): void {
    const s = useStudioStore.getState();
    if (!s.playback) return;
    s.setPlaying(!s.playing);
  },
  /** B / the speed button: 1× → 2× → 4× → 8× → 1×. */
  cycleSpeed(): void {
    const s = useStudioStore.getState();
    s.setPlaybackSpeed(nextSpeed(s.playbackSpeed));
  },
  /** X / the recenter button: back to the launch stance, facing the cell. */
  recenter(): void {
    const { origin, stance } = xrRig;
    const head = headPose();
    if (!origin || !head || !stance) return;
    applyPlacement(origin, stancePlacement(stance, head));
  },
  /** Left Y / the panel's `+ point`: the focused robot's current
   * configuration becomes a joint waypoint of the motion the MOTION panel
   * shows (its upright toggle applies). Explicit, never on letting go of
   * the handle: a release is rarely the pose one meant. */
  addWaypoint(): void {
    const s = useStudioStore.getState();
    const name = s.selection.type === "tcp" ? s.selection.robot : s.selectedRobot;
    const robot = robotByName(s.robots, name);
    if (!robot) return;
    const { motionName, arm } = motionTarget(
      s.motions,
      s.selectedMotion,
      {
        name: robot.desc.name,
        dualArm: robotArms(robot.desc).length > 0,
        selectedGroup: robot.selectedGroup,
      },
      s.robots[0]?.desc.name,
    );
    sendAddSegment(
      motionName,
      robot.desc.name,
      waypointSegment(robot.jointPositions, "joint", s.motionUpright),
      arm,
    );
  },
  /** The exit button: ends the session (the desktop view returns). */
  exit(): void {
    exitVr();
  },
  run(action: PanelAction): void {
    switch (action) {
      case "play":
        return xrActions.togglePlay();
      case "speed":
        return xrActions.cycleSpeed();
      case "waypoint":
        return xrActions.addWaypoint();
      case "recenter":
        return xrActions.recenter();
      case "exit":
        return xrActions.exit();
    }
  },
};
