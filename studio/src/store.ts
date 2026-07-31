import { create } from "zustand";
import type {
  CollisionPairMsg,
  FrameMsg,
  IkStatusMsg,
  MotionMsg,
  ObstacleMsg,
  PlanStatsMsg,
  PoseMsg,
  SceneDescriptionMsg,
  DeviceMsg,
  SensorMsg,
  SequenceMsg,
  ServerMessage,
  SignalDefMsg,
  SignalTrackMsg,
  StepSpanMsg,
  TrajectoryMsg,
} from "./protocol";

export type ConnectionStatus = "connecting" | "connected" | "disconnected";
export type GizmoMode = "translate" | "rotate";

/** What the viewport gizmo is currently editing. */
export type Selection =
  | { type: "tcp" }
  | { type: "obstacle"; name: string }
  | { type: "robot" };

/** Actuated joints (those with a q_index), in q_index order. */
function actuatedDof(scene: SceneDescriptionMsg): number {
  return scene.joints.filter((j) => j.q_index !== null).length;
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.min(Math.max(v, lo), hi);
}

/** Names of links involved in at least one collision pair. */
export function collidingLinkNames(collisions: CollisionPairMsg[]): Set<string> {
  const set = new Set<string>();
  for (const pair of collisions) {
    for (const ref of [pair.a, pair.b]) {
      if (ref.kind === "link") set.add(ref.name);
    }
  }
  return set;
}

/** Names of obstacles involved in at least one collision pair. */
export function collidingObstacleNames(
  collisions: CollisionPairMsg[],
): Set<string> {
  const set = new Set<string>();
  for (const pair of collisions) {
    for (const ref of [pair.a, pair.b]) {
      if (ref.kind === "obstacle") set.add(ref.name);
    }
  }
  return set;
}

/** First-sample poses of every attached-object track (playback seeding). */
function firstObjectPoses(traj: TrajectoryMsg): Record<string, PoseMsg> | null {
  if (!traj.object_tracks || traj.object_tracks.length === 0) return null;
  const out: Record<string, PoseMsg> = {};
  for (const track of traj.object_tracks) out[track.name] = track.poses[0];
  return out;
}

interface StudioState {
  sceneDesc: SceneDescriptionMsg | null;
  /** Joint position vector, indexed by q_index (length = DOF). */
  jointPositions: number[];
  /** World pose of the robot's root link. */
  basePose: PoseMsg | null;
  /** World pose per link, aligned with sceneDesc.links. */
  linkPoses: PoseMsg[];
  connection: ConnectionStatus;
  /** Outcome of the last server-side IK solve (null after non-IK updates). */
  ikStatus: IkStatusMsg | null;
  /** Link the TCP gizmo is attached to. */
  tcpLink: string | null;
  gizmoMode: GizmoMode;
  /** Obstacles in the scene; re-sent in full by the server on every change. */
  obstacles: ObstacleMsg[];
  /** Named world frames (mount points); re-sent in full on every change. */
  frames: FrameMsg[];
  /** Colliding pairs at the current configuration (empty when clear). */
  collisions: CollisionPairMsg[];
  /** Minimum robot-obstacle distance; null without obstacles. */
  minDistance: number | null;
  /** Which object the viewport gizmo edits; defaults to the TCP. */
  selection: Selection;
  /** Snapshot of a goal configuration (joints + link poses for the ghost). */
  goal: { positions: number[]; linkPoses: PoseMsg[] } | null;
  /** True while a plan request is in flight. */
  planning: boolean;
  planError: string | null;
  planStats: PlanStatsMsg | null;
  /** Last planned trajectory (for preview playback). */
  trajectory: TrajectoryMsg | null;
  playbackTime: number;
  playing: boolean;
  /** When set, the robot renders at these poses instead of the live state. */
  overridePoses: PoseMsg[] | null;
  /** Joint-space playback override (USD-rendered robots do FK client-side). */
  overrideJoints: number[] | null;
  /** Playback poses of attached objects, keyed by obstacle name. */
  overrideObstaclePoses: Record<string, PoseMsg> | null;
  /** All motions in the scene; re-sent in full by the server on every change. */
  motions: MotionMsg[];
  /** PLC-style sequences; re-sent in full by the server on every change. */
  sequences: SequenceMsg[];
  /** Declared internal signals. */
  signalDefs: SignalDefMsg[];
  /** Pseudo-sensors; re-sent in full by the server on every change. */
  sensors: SensorMsg[];
  /** Auxiliary devices; re-sent in full by the server on every change. */
  devices: DeviceMsg[];
  /** True while a sequence rollout is in flight. */
  sequenceSimulating: boolean;
  sequenceError: string | null;
  /** Step bands + signal lanes of the last baked timeline (the dock). */
  timeline: {
    duration: number;
    stepSpans: StepSpanMsg[];
    signals: SignalTrackMsg[];
  } | null;
  /** The USD recording behind the current playback, when there is one. */
  recording: { source: string; mode: string; warnings: string[] } | null;
  recordingError: string | null;
  /** True while a motion plan request is in flight. */
  motionPlanning: boolean;
  motionError: string | null;
  /** Time at which each planned segment ends (playback boundary markers). */
  segmentEnds: number[];
  /** Timing of the last successful motion plan. */
  motionStats: { planningTimeMs: number } | null;
  /** A USD file dropped in wasm mode, rendered client-side as the stage. */
  droppedStage: { data: ArrayBuffer; name: string } | null;
  /** Obstacles hidden in the viewport (display only; collision unaffected). */
  hiddenObstacles: Set<string>;

  setConnection: (c: ConnectionStatus) => void;
  applyServerMessage: (msg: ServerMessage) => void;
  /** Optimistic local update of a single DOF. */
  setJointPosition: (qIndex: number, value: number) => void;
  /** Reset every DOF to 0 (clamped into limits when present). */
  resetJoints: () => void;
  setTcpLink: (link: string) => void;
  setGizmoMode: (mode: GizmoMode) => void;
  selectTcp: () => void;
  selectObstacle: (name: string) => void;
  selectRobot: () => void;
  beginSequenceSim: () => void;
  setGoalFromCurrent: () => void;
  clearGoal: () => void;
  beginPlanning: () => void;
  beginMotionPlanning: () => void;
  /** Scrub/advance playback; poses or joints become the display override. */
  setPlayback: (
    t: number,
    poses: PoseMsg[] | null,
    joints: number[] | null,
    obstaclePoses: Record<string, PoseMsg> | null,
  ) => void;
  setPlaying: (playing: boolean) => void;
  /** Ends playback and returns the display to the live state. */
  stopPlayback: () => void;
  setDroppedStage: (stage: { data: ArrayBuffer; name: string } | null) => void;
  toggleObstacleHidden: (name: string) => void;
}

export const useStudioStore = create<StudioState>((set) => ({
  sceneDesc: null,
  jointPositions: [],
  basePose: null,
  linkPoses: [],
  connection: "connecting",
  ikStatus: null,
  tcpLink: null,
  gizmoMode: "translate",
  obstacles: [],
  frames: [],
  collisions: [],
  minDistance: null,
  selection: { type: "tcp" },
  goal: null,
  planning: false,
  planError: null,
  planStats: null,
  trajectory: null,
  playbackTime: 0,
  playing: false,
  overridePoses: null,
  overrideJoints: null,
  overrideObstaclePoses: null,
  motions: [],
  sequences: [],
  signalDefs: [],
  sensors: [],
  devices: [],
  sequenceSimulating: false,
  sequenceError: null,
  timeline: null,
  recording: null,
  recordingError: null,
  motionPlanning: false,
  motionError: null,
  segmentEnds: [],
  motionStats: null,
  droppedStage: null,
  hiddenObstacles: new Set(),

  setConnection: (c) => set({ connection: c }),

  applyServerMessage: (msg) => {
    if (msg.type === "scene_init") {
      set({
        sceneDesc: msg.scene,
        jointPositions: new Array(actuatedDof(msg.scene)).fill(0),
        basePose: msg.scene.base_pose,
        linkPoses: [],
        ikStatus: null,
        tcpLink: msg.scene.tcp_link,
        obstacles: [],
        frames: [],
        collisions: [],
        minDistance: null,
        selection: { type: "tcp" },
        goal: null,
        planning: false,
        planError: null,
        planStats: null,
        trajectory: null,
        playbackTime: 0,
        playing: false,
        overridePoses: null,
        overrideJoints: null,
        overrideObstaclePoses: null,
        motions: [],
        sequences: [],
        signalDefs: [],
        sensors: [],
        devices: [],
        sequenceSimulating: false,
        sequenceError: null,
        timeline: null,
        motionPlanning: false,
        motionError: null,
        segmentEnds: [],
        motionStats: null,
      });
    } else if (msg.type === "obstacles") {
      set((s) => {
        // Drop the selection if the obstacle it pointed at is gone.
        const sel = s.selection;
        const gone =
          sel.type === "obstacle" &&
          !msg.obstacles.some((o) => o.name === sel.name);
        return {
          obstacles: msg.obstacles,
          selection: gone ? { type: "tcp" } : sel,
        };
      });
    } else if (msg.type === "plan_result") {
      if (msg.ok && msg.trajectory) {
        // Auto-start the preview. The trajectory is shared with motion
        // playback, so drop any stale motion badge/markers it left behind.
        set({
          planning: false,
          planError: null,
          planStats: msg.stats,
          trajectory: msg.trajectory,
          playbackTime: 0,
          playing: true,
          overridePoses: msg.trajectory.link_poses?.[0] ?? null,
          overrideJoints: msg.trajectory.link_poses
            ? null
            : (msg.trajectory.joint_positions[0] ?? null),
          overrideObstaclePoses: firstObjectPoses(msg.trajectory),
          segmentEnds: [],
          motionStats: null,
          timeline: null,
          recording: null,
        });
      } else {
        set({ planning: false, planError: msg.error ?? "planning failed" });
      }
    } else if (msg.type === "frames") {
      set({ frames: msg.frames });
    } else if (msg.type === "motions") {
      set({ motions: msg.motions });
    } else if (msg.type === "sequences") {
      set({ sequences: msg.sequences, signalDefs: msg.signals });
    } else if (msg.type === "sensors") {
      set({ sensors: msg.sensors });
    } else if (msg.type === "devices") {
      set({ devices: msg.devices });
    } else if (msg.type === "sequence_result") {
      if (msg.ok && msg.timeline) {
        const traj = msg.timeline.trajectory;
        // The baked cycle plays through the shared trajectory machinery;
        // step ends double as the seek-bar tick marks.
        set({
          sequenceSimulating: false,
          sequenceError: null,
          timeline: {
            duration: msg.timeline.duration,
            stepSpans: msg.timeline.step_spans,
            signals: msg.timeline.signals,
          },
          segmentEnds: msg.timeline.step_spans.map((s) => s.end),
          trajectory: traj,
          playbackTime: 0,
          playing: true,
          overridePoses: traj.link_poses?.[0] ?? null,
          overrideJoints: traj.link_poses ? null : (traj.joint_positions[0] ?? null),
          overrideObstaclePoses: firstObjectPoses(traj),
          planStats: null,
          planError: null,
          motionStats: null,
          motionError: null,
          recording: null,
        });
      } else {
        set({
          sequenceSimulating: false,
          sequenceError: msg.error ?? "simulation failed",
        });
      }
    } else if (msg.type === "recording_result") {
      if (msg.ok && msg.timeline) {
        const traj = msg.timeline.trajectory;
        // A baked USD recording (Isaac capture or botrail export) plays
        // through the shared trajectory machinery. Joint-state recordings
        // carry joint_positions; transform recordings carry link_poses,
        // which USD robots apply via setLinkTransforms (baked mode).
        set({
          recording: {
            source: msg.source,
            mode: msg.mode ?? "",
            warnings: msg.warnings,
          },
          recordingError: null,
          timeline: {
            duration: msg.timeline.duration,
            stepSpans: msg.timeline.step_spans,
            signals: msg.timeline.signals,
          },
          segmentEnds: [],
          trajectory: traj,
          playbackTime: 0,
          playing: true,
          overridePoses: traj.link_poses?.[0] ?? null,
          overrideJoints: traj.link_poses ? null : (traj.joint_positions[0] ?? null),
          overrideObstaclePoses: firstObjectPoses(traj),
          planStats: null,
          planError: null,
          motionStats: null,
          motionError: null,
        });
      } else {
        set({
          recording: null,
          recordingError: msg.error ?? "recording import failed",
        });
      }
    } else if (msg.type === "motion_result") {
      if (msg.ok && msg.trajectory) {
        // Auto-start the preview through the same shared playback state the
        // plan panel uses; clear its badge/error since we now own the preview.
        set({
          motionPlanning: false,
          motionError: null,
          motionStats: { planningTimeMs: msg.planning_time_ms ?? 0 },
          segmentEnds: msg.segment_ends,
          trajectory: msg.trajectory,
          playbackTime: 0,
          playing: true,
          overridePoses: msg.trajectory.link_poses?.[0] ?? null,
          overrideJoints: msg.trajectory.link_poses
            ? null
            : (msg.trajectory.joint_positions[0] ?? null),
          overrideObstaclePoses: firstObjectPoses(msg.trajectory),
          planStats: null,
          planError: null,
          timeline: null,
          recording: null,
        });
      } else {
        set({ motionPlanning: false, motionError: msg.error ?? "planning failed" });
      }
    } else {
      set({
        jointPositions: msg.joint_positions,
        basePose: msg.base_pose,
        linkPoses: msg.link_poses,
        ikStatus: msg.ik_status,
        collisions: msg.collisions,
        minDistance: msg.min_distance,
      });
    }
  },

  setJointPosition: (qIndex, value) =>
    set((s) => {
      const next = s.jointPositions.slice();
      next[qIndex] = value;
      return { jointPositions: next };
    }),

  resetJoints: () =>
    set((s) => {
      const desc = s.sceneDesc;
      const next = s.jointPositions.slice();
      if (!desc) {
        return { jointPositions: next.map(() => 0) };
      }
      for (const j of desc.joints) {
        if (j.q_index === null) continue;
        next[j.q_index] = j.limits ? clamp(0, j.limits[0], j.limits[1]) : 0;
      }
      return { jointPositions: next };
    }),

  setTcpLink: (link) => set({ tcpLink: link, ikStatus: null }),
  setGizmoMode: (mode) => set({ gizmoMode: mode }),
  selectTcp: () => set({ selection: { type: "tcp" } }),
  selectObstacle: (name) => set({ selection: { type: "obstacle", name } }),
  selectRobot: () => set({ selection: { type: "robot" } }),

  beginSequenceSim: () => set({ sequenceSimulating: true, sequenceError: null }),
  setGoalFromCurrent: () =>
    set((s) => ({
      goal:
        s.linkPoses.length > 0
          ? { positions: s.jointPositions.slice(), linkPoses: s.linkPoses }
          : null,
      planError: null,
    })),
  clearGoal: () => set({ goal: null, planError: null }),
  beginPlanning: () => set({ planning: true, planError: null }),
  beginMotionPlanning: () => set({ motionPlanning: true, motionError: null }),

  setPlayback: (t, poses, joints, obstaclePoses) =>
    set({
      playbackTime: t,
      overridePoses: poses,
      overrideJoints: joints,
      overrideObstaclePoses: obstaclePoses,
    }),
  setPlaying: (playing) => set({ playing }),
  stopPlayback: () =>
    set({
      playing: false,
      overridePoses: null,
      overrideJoints: null,
      overrideObstaclePoses: null,
    }),
  setDroppedStage: (stage) => set({ droppedStage: stage }),
  toggleObstacleHidden: (name) =>
    set((s) => {
      const next = new Set(s.hiddenObstacles);
      if (next.has(name)) {
        next.delete(name);
      } else {
        next.add(name);
      }
      return { hiddenObstacles: next };
    }),
}));
