import { create } from "zustand";
import type {
  CollisionPairMsg,
  FrameMsg,
  IkStatusMsg,
  MotionMsg,
  ObstacleMsg,
  PlanStatsMsg,
  PoseMsg,
  RobotDescMsg,
  DeviceMsg,
  SensorMsg,
  SequenceMsg,
  ServerMessage,
  SignalDefMsg,
  SignalTrackMsg,
  StepSpanMsg,
} from "./protocol";
import {
  samplePlayback,
  tracksFromTimeline,
  tracksFromTrajectory,
  type PlaybackSample,
  type PlaybackTracks,
} from "./playback";

export type ConnectionStatus = "connecting" | "connected" | "disconnected";
export type GizmoMode = "translate" | "rotate";

/** What the viewport gizmo is currently editing. */
export type Selection =
  | { type: "tcp"; robot: string }
  | { type: "obstacle"; name: string }
  | { type: "robot"; robot: string };

/** Per-robot UI state: the description plus the live server state. */
export interface RobotUiState {
  desc: RobotDescMsg;
  /** Joint position vector, indexed by q_index (length = robot DOF). */
  jointPositions: number[];
  /** World pose of the robot's root link. */
  basePose: PoseMsg;
  /** World pose per link, aligned with desc.links. */
  linkPoses: PoseMsg[];
  /** Outcome of the last server-side IK solve for this robot. */
  ikStatus: IkStatusMsg | null;
  /** Link the TCP gizmo is attached to. */
  tcpLink: string | null;
}

/** Actuated joints (those with a q_index), in q_index order. */
function actuatedDof(desc: RobotDescMsg): number {
  return desc.joints.filter((j) => j.q_index !== null).length;
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.min(Math.max(v, lo), hi);
}

/** The robot named `name`, or null. */
export function robotByName(
  robots: RobotUiState[],
  name: string | null,
): RobotUiState | null {
  if (name === null) return null;
  return robots.find((r) => r.desc.name === name) ?? null;
}

/** Names of `robot`'s links involved in at least one collision pair. */
export function collidingLinkNames(
  collisions: CollisionPairMsg[],
  robot: string,
): Set<string> {
  const set = new Set<string>();
  for (const pair of collisions) {
    for (const ref of [pair.a, pair.b]) {
      if (ref.kind === "link" && ref.robot === robot) set.add(ref.name);
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

/** Playback + first-sample overrides, spread into a store update. */
function startPlayback(tracks: PlaybackTracks) {
  const sample = samplePlayback(tracks, 0);
  return {
    playback: tracks,
    playbackTime: 0,
    playing: true,
    overridePoses: sample.poses,
    overrideJoints: sample.joints,
    overrideObstaclePoses: sample.objects,
  };
}

interface StudioState {
  /** Robot instances, in server (scene) order. */
  robots: RobotUiState[];
  /** Robot the panels operate on (instance name). */
  selectedRobot: string | null;
  connection: ConnectionStatus;
  gizmoMode: GizmoMode;
  /** Obstacles in the scene; re-sent in full by the server on every change. */
  obstacles: ObstacleMsg[];
  /** Named world frames (mount points); re-sent in full on every change. */
  frames: FrameMsg[];
  /** Colliding pairs at the current configuration (empty when clear). */
  collisions: CollisionPairMsg[];
  /** Minimum robot-obstacle distance; null without obstacles. */
  minDistance: number | null;
  /** Which object the viewport gizmo edits; defaults to the first TCP. */
  selection: Selection;
  /** Snapshot of a goal configuration on one robot (ghost display). */
  goal: { robot: string; positions: number[]; linkPoses: PoseMsg[] } | null;
  /** True while a plan request is in flight. */
  planning: boolean;
  planError: string | null;
  planStats: PlanStatsMsg | null;
  /** Last playable result (plan/motion/sequence/recording tracks). */
  playback: PlaybackTracks | null;
  playbackTime: number;
  playing: boolean;
  /** Robot name -> link poses shown instead of the live state (playback). */
  overridePoses: Record<string, PoseMsg[]> | null;
  /** Robot name -> joint-space playback override (USD robots, client FK). */
  overrideJoints: Record<string, number[]> | null;
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
  /** Step bands, robot lanes + signal lanes of the last baked timeline. */
  timeline: {
    duration: number;
    stepSpans: StepSpanMsg[];
    /** Per-robot move intervals (motion/ramp bands), in scene order. */
    robots: { name: string; moves: StepSpanMsg[] }[];
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
  /** Optimistic local update of a single DOF on one robot. */
  setJointPosition: (robot: string, qIndex: number, value: number) => void;
  /** Reset a robot's DOFs to 0 (clamped into limits when present). */
  resetJoints: (robot: string) => void;
  setTcpLink: (robot: string, link: string) => void;
  setGizmoMode: (mode: GizmoMode) => void;
  /** Focus a robot's TCP (also makes it the panel-selected robot). */
  selectTcp: (robot?: string) => void;
  selectObstacle: (name: string) => void;
  /** Focus a robot's base for placement (also panel-selects it). */
  selectRobot: (robot: string) => void;
  /** Panel robot selector; retargets a robot-scoped gizmo selection. */
  setSelectedRobot: (robot: string) => void;
  beginSequenceSim: () => void;
  setGoalFromCurrent: () => void;
  clearGoal: () => void;
  beginPlanning: () => void;
  beginMotionPlanning: () => void;
  /** Scrub/advance playback; the sample becomes the display override. */
  setPlayback: (t: number, sample: PlaybackSample) => void;
  setPlaying: (playing: boolean) => void;
  /** Ends playback and returns the display to the live state. */
  stopPlayback: () => void;
  setDroppedStage: (stage: { data: ArrayBuffer; name: string } | null) => void;
  toggleObstacleHidden: (name: string) => void;
}

export const useStudioStore = create<StudioState>((set) => ({
  robots: [],
  selectedRobot: null,
  connection: "connecting",
  gizmoMode: "translate",
  obstacles: [],
  frames: [],
  collisions: [],
  minDistance: null,
  selection: { type: "tcp", robot: "" },
  goal: null,
  planning: false,
  planError: null,
  planStats: null,
  playback: null,
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
      set((s) => {
        // A re-handshake (e.g. a robot was added) keeps the user's TCP link
        // and selection for robots that survive by name.
        const prev = new Map(s.robots.map((r) => [r.desc.name, r]));
        const robots = msg.scene.robots.map((desc) => ({
          desc,
          jointPositions: new Array(actuatedDof(desc)).fill(0),
          basePose: desc.base_pose,
          linkPoses: [],
          ikStatus: null,
          tcpLink: prev.get(desc.name)?.tcpLink ?? desc.tcp_link,
        }));
        const surviving =
          s.selectedRobot !== null &&
          robots.some((r) => r.desc.name === s.selectedRobot);
        const selected = surviving
          ? s.selectedRobot
          : (robots[0]?.desc.name ?? null);
        return {
          robots,
          selectedRobot: selected,
          obstacles: [],
          frames: [],
          collisions: [],
          minDistance: null,
          selection: { type: "tcp", robot: selected ?? "" },
          goal: null,
          planning: false,
          planError: null,
          planStats: null,
          playback: null,
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
        };
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
          selection: gone
            ? { type: "tcp", robot: s.selectedRobot ?? "" }
            : sel,
        };
      });
    } else if (msg.type === "plan_result") {
      if (msg.ok && msg.trajectory) {
        // Auto-start the preview. The playback is shared with motion
        // playback, so drop any stale motion badge/markers it left behind.
        set({
          planning: false,
          planError: null,
          planStats: msg.stats,
          ...startPlayback(tracksFromTrajectory(msg.robot, msg.trajectory)),
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
        // The baked cycle plays through the shared playback machinery;
        // step ends double as the seek-bar tick marks.
        set({
          sequenceSimulating: false,
          sequenceError: null,
          timeline: {
            duration: msg.timeline.duration,
            stepSpans: msg.timeline.step_spans,
            robots: msg.timeline.robots.map((r) => ({
              name: r.name,
              moves: r.moves,
            })),
            signals: msg.timeline.signals,
          },
          segmentEnds: msg.timeline.step_spans.map((s) => s.end),
          ...startPlayback(tracksFromTimeline(msg.timeline)),
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
        // A baked USD recording (Isaac capture or botrail export) plays
        // through the shared playback machinery. Joint-state recordings
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
            robots: msg.timeline.robots.map((r) => ({
              name: r.name,
              moves: r.moves,
            })),
            signals: msg.timeline.signals,
          },
          segmentEnds: [],
          ...startPlayback(tracksFromTimeline(msg.timeline)),
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
          ...startPlayback(tracksFromTrajectory(msg.robot, msg.trajectory)),
          planStats: null,
          planError: null,
          timeline: null,
          recording: null,
        });
      } else {
        set({
          motionPlanning: false,
          motionError: msg.error ?? "planning failed",
        });
      }
    } else if (msg.type === "state") {
      set((s) => ({
        robots: s.robots.map((r) => {
          const st = msg.robots.find((x) => x.name === r.desc.name);
          return st
            ? {
                ...r,
                jointPositions: st.joint_positions,
                basePose: st.base_pose,
                linkPoses: st.link_poses,
                ikStatus: st.ik_status,
              }
            : r;
        }),
        collisions: msg.collisions,
        minDistance: msg.min_distance,
      }));
    }
    // Message types this build doesn't know (a newer server) fall through
    // untouched — a stale bundle must degrade, not crash to a black screen.
  },

  setJointPosition: (robot, qIndex, value) =>
    set((s) => ({
      robots: s.robots.map((r) => {
        if (r.desc.name !== robot) return r;
        const next = r.jointPositions.slice();
        next[qIndex] = value;
        return { ...r, jointPositions: next };
      }),
    })),

  resetJoints: (robot) =>
    set((s) => ({
      robots: s.robots.map((r) => {
        if (r.desc.name !== robot) return r;
        const next = r.jointPositions.slice();
        for (const j of r.desc.joints) {
          if (j.q_index === null) continue;
          next[j.q_index] = j.limits ? clamp(0, j.limits[0], j.limits[1]) : 0;
        }
        return { ...r, jointPositions: next };
      }),
    })),

  setTcpLink: (robot, link) =>
    set((s) => ({
      robots: s.robots.map((r) =>
        r.desc.name === robot ? { ...r, tcpLink: link, ikStatus: null } : r,
      ),
    })),
  setGizmoMode: (mode) => set({ gizmoMode: mode }),
  selectTcp: (robot) =>
    set((s) => {
      const name = robot ?? s.selectedRobot ?? s.robots[0]?.desc.name ?? "";
      return { selection: { type: "tcp", robot: name }, selectedRobot: name };
    }),
  selectObstacle: (name) => set({ selection: { type: "obstacle", name } }),
  selectRobot: (robot) =>
    set({ selection: { type: "robot", robot }, selectedRobot: robot }),
  setSelectedRobot: (robot) =>
    set((s) => ({
      selectedRobot: robot,
      selection:
        s.selection.type === "obstacle"
          ? s.selection
          : s.selection.type === "robot"
            ? { type: "robot", robot }
            : { type: "tcp", robot },
    })),

  beginSequenceSim: () => set({ sequenceSimulating: true, sequenceError: null }),
  setGoalFromCurrent: () =>
    set((s) => {
      const r = robotByName(s.robots, s.selectedRobot);
      return {
        goal:
          r && r.linkPoses.length > 0
            ? {
                robot: r.desc.name,
                positions: r.jointPositions.slice(),
                linkPoses: r.linkPoses,
              }
            : null,
        planError: null,
      };
    }),
  clearGoal: () => set({ goal: null, planError: null }),
  beginPlanning: () => set({ planning: true, planError: null }),
  beginMotionPlanning: () => set({ motionPlanning: true, motionError: null }),

  setPlayback: (t, sample) =>
    set({
      playbackTime: t,
      overridePoses: sample.poses,
      overrideJoints: sample.joints,
      overrideObstaclePoses: sample.objects,
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
