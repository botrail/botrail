import { create } from "zustand";
import type {
  BranchTakenMsg,
  CollisionPairMsg,
  FlashMsg,
  FrameMsg,
  ToolpathOverlayMsg,
  IkStatusMsg,
  MotionMsg,
  ObstacleMsg,
  PoseMsg,
  RobotDescMsg,
  DeviceMsg,
  ScenarioMsg,
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
import { attributionIssues } from "./sfc";

export type ConnectionStatus = "connecting" | "connected" | "disconnected";
export type GizmoMode = "translate" | "rotate";
export type SidebarTab = "layout" | "motion" | "sequence";

const TAB_KEY = "botrail-studio.tab";

function initialTab(): SidebarTab {
  try {
    const v = localStorage.getItem(TAB_KEY);
    if (v === "layout" || v === "motion" || v === "sequence") return v;
  } catch {
    // Private-mode storage failures only cost persistence.
  }
  return "layout";
}

function persistTab(tab: SidebarTab): SidebarTab {
  try {
    localStorage.setItem(TAB_KEY, tab);
  } catch {
    // Persistence only.
  }
  return tab;
}

const SFC_KEY = "botrail-studio.sfc";

function initialSfcOpen(): boolean {
  try {
    return localStorage.getItem(SFC_KEY) === "1";
  } catch {
    return false;
  }
}

function persistSfcOpen(open: boolean): boolean {
  try {
    localStorage.setItem(SFC_KEY, open ? "1" : "0");
  } catch {
    // Persistence only.
  }
  return open;
}


/**
 * Selection steps for a viewport click on `name`, outermost first, ending
 * at the obstacle itself: `/World/Pedestal` then `/World/Pedestal/Column`.
 *
 * The stage root is skipped when every obstacle sits under it — selecting
 * "everything" is never what a click on one machine means. A scene with
 * several roots keeps them all, since there each root *is* a machine.
 */
export function drillChain(name: string, allNames: string[]): string[] {
  const parts = name.split("/").filter(Boolean);
  const chain: string[] = [];
  for (let i = 1; i < parts.length; i++) {
    const path = `/${parts.slice(0, i).join("/")}`;
    if (i === 1 && allNames.every((n) => n.startsWith(`${path}/`))) continue;
    chain.push(path);
  }
  chain.push(name);
  return chain;
}

/** What the viewport gizmo (or the Layout inspector) is currently editing. */
export type Selection =
  | { type: "tcp"; robot: string }
  | { type: "obstacle"; name: string }
  /** An imported subtree (`/World/Pedestal`), moved as one rigid body. */
  | { type: "group"; path: string }
  | { type: "robot"; robot: string }
  /** Fixtures have no gizmo; selecting one opens its form. */
  | { type: "sensor"; name: string }
  | { type: "device"; name: string };

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

/** Robot-scoped selections (tcp, base) retarget on a robot switch;
 * object-scoped ones (obstacle, group, sensor, device) persist. */
function retargetSelection(sel: Selection, robot: string): Selection {
  if (sel.type === "tcp") return { type: "tcp", robot };
  if (sel.type === "robot") return { type: "robot", robot };
  return sel;
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
    overrideBases: sample.bases,
    overrideObstaclePoses: sample.objects,
    stowedObstacles: sample.stowed,
  };
}

export interface StudioState {
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
  /** Toolpath overlays (world-resolved polylines); re-sent in full on
   * every toolpath or part-frame change. */
  toolpaths: ToolpathOverlayMsg[];
  /** Colliding pairs at the current configuration (empty when clear). */
  collisions: CollisionPairMsg[];
  /** Minimum robot-obstacle distance; null without obstacles. */
  minDistance: number | null;
  /** Which object the viewport gizmo edits; defaults to the first TCP. */
  selection: Selection;
  /** Which sidebar tab is up. */
  activeTab: SidebarTab;
  /** Motion the Motion tab edits; null adopts the selected robot's first
   * motion (or the conventional fresh name when it has none yet). */
  selectedMotion: string | null;
  /** Last playable result (plan/motion/sequence/recording tracks). */
  playback: PlaybackTracks | null;
  playbackTime: number;
  playing: boolean;
  /** Playback rate multiplier (1, 2, 4, 8). */
  playbackSpeed: number;
  /** Restart from 0 when the end is reached. */
  playbackLoop: boolean;
  /** Robot name -> link poses shown instead of the live state (playback). */
  overridePoses: Record<string, PoseMsg[]> | null;
  /** Robot name -> joint-space playback override (USD robots, client FK). */
  overrideJoints: Record<string, number[]> | null;
  /** Robot name -> base pose during playback, for robots riding a vehicle.
   * Their base is not a scene constant, so the views cannot read it from
   * the description while a timeline plays. */
  overrideBases: Record<string, PoseMsg> | null;
  /** Playback poses of attached objects, keyed by obstacle name. */
  overrideObstaclePoses: Record<string, PoseMsg> | null;
  /** Objects stowed at the current playback instant — waiting in a
   * magazine or off the line — and therefore not drawn. */
  stowedObstacles: Set<string>;
  /** All motions in the scene; re-sent in full by the server on every change. */
  motions: MotionMsg[];
  /** PLC-style sequences; re-sent in full by the server on every change. */
  sequences: SequenceMsg[];
  /** Declared internal signals. */
  signalDefs: SignalDefMsg[];
  /** Pseudo-sensors; re-sent in full by the server on every change. */
  sensors: SensorMsg[];
  /** Weld-flash bindings (signal -> flash at a robot's TCP). */
  flashes: FlashMsg[];
  /** Auxiliary devices; re-sent in full by the server on every change. */
  devices: DeviceMsg[];
  /** Scenarios (named initial-state deltas); re-sent in full on change. */
  scenarios: ScenarioMsg[];
  /** The SFC chart overlay over the viewport (persisted). */
  sfcOpen: boolean;
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
    /** Which arm each branching step took (recordings have none). */
    branches: BranchTakenMsg[];
    /** Scenario the bake ran under; null = the unmodified scene. */
    scenario: string | null;
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
  selectGroup: (path: string) => void;
  selectSensor: (name: string) => void;
  selectDevice: (name: string) => void;
  /** Motion-tab list click; an existing motion also retargets the robot
   * to its owner, so added waypoints always fit the motion's DOF. */
  selectMotion: (name: string) => void;
  /** Viewport click: the machine first, a further click drills in. */
  selectFromViewport: (name: string) => void;
  /** Focus a robot's base for placement (also panel-selects it). */
  selectRobot: (robot: string) => void;
  /** Panel robot selector; retargets a robot-scoped gizmo selection. */
  setSelectedRobot: (robot: string) => void;
  setActiveTab: (tab: SidebarTab) => void;
  setSfcOpen: (open: boolean) => void;
  /**
   * Viewport pick: raise the tab holding the picked thing's tools
   * (robot → Motion, obstacle → Layout). The Sequence tab is sticky —
   * picking a part or an arm there is how grasp steps are authored, so
   * it must not lose its place.
   */
  focusTab: (target: "robot" | "obstacle") => void;
  beginSequenceSim: () => void;
  beginMotionPlanning: () => void;
  /** Scrub/advance playback; the sample becomes the display override. */
  setPlayback: (t: number, sample: PlaybackSample) => void;
  setPlaying: (playing: boolean) => void;
  /** Playhead only — the driver applies poses imperatively while playing
   * and syncs the full sample into state on pause/seek/end. */
  setPlaybackTime: (t: number) => void;
  setPlaybackSpeed: (speed: number) => void;
  setPlaybackLoop: (loop: boolean) => void;
  /** Ends playback and returns the display to the live state. */
  stopPlayback: () => void;
  setDroppedStage: (stage: { data: ArrayBuffer; name: string } | null) => void;
  toggleObstacleHidden: (name: string) => void;
}

export const useStudioStore = create<StudioState>((set, get) => ({
  robots: [],
  selectedRobot: null,
  connection: "connecting",
  gizmoMode: "translate",
  obstacles: [],
  frames: [],
  toolpaths: [],
  collisions: [],
  minDistance: null,
  selection: { type: "tcp", robot: "" },
  activeTab: initialTab(),
  sfcOpen: initialSfcOpen(),
  selectedMotion: null,
  playback: null,
  playbackTime: 0,
  playing: false,
  playbackSpeed: 1,
  playbackLoop: false,
  overridePoses: null,
  overrideJoints: null,
  overrideBases: null,
  overrideObstaclePoses: null,
  stowedObstacles: new Set<string>(),
  motions: [],
  sequences: [],
  signalDefs: [],
  sensors: [],
  flashes: [],
  devices: [],
  scenarios: [],
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
          toolpaths: [],
          collisions: [],
          minDistance: null,
          selection: { type: "tcp", robot: selected ?? "" },
          selectedMotion: null,
          playback: null,
          playbackTime: 0,
          playing: false,
          overridePoses: null,
          overrideJoints: null,
          overrideBases: null,
          overrideObstaclePoses: null,
          stowedObstacles: new Set<string>(),
          motions: [],
          sequences: [],
          signalDefs: [],
          sensors: [],
          devices: [],
          scenarios: [],
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
      // Python-side `plan_to_pose` against a live session broadcasts its
      // trajectory here; preview it. Failures surface on the caller's side
      // (an exception in the notebook), not in the studio.
      if (msg.ok && msg.trajectory) {
        set({
          ...startPlayback(tracksFromTrajectory(msg.robot, msg.trajectory)),
          segmentEnds: [],
          motionStats: null,
          motionError: null,
          timeline: null,
          recording: null,
        });
      }
    } else if (msg.type === "frames") {
      set({ frames: msg.frames });
    } else if (msg.type === "toolpaths") {
      set({ toolpaths: msg.toolpaths });
    } else if (msg.type === "motions") {
      set({ motions: msg.motions });
    } else if (msg.type === "sequences") {
      set({ sequences: msg.sequences, signalDefs: msg.signals });
    } else if (msg.type === "sensors") {
      set((s) => {
        const sel = s.selection;
        const gone =
          sel.type === "sensor" && !msg.sensors.some((x) => x.name === sel.name);
        return {
          sensors: msg.sensors,
          selection: gone
            ? { type: "tcp", robot: s.selectedRobot ?? "" }
            : sel,
        };
      });
    } else if (msg.type === "effects") {
      set({ flashes: msg.flashes });
    } else if (msg.type === "devices") {
      set((s) => {
        const sel = s.selection;
        const gone =
          sel.type === "device" && !msg.devices.some((x) => x.name === sel.name);
        return {
          devices: msg.devices,
          selection: gone
            ? { type: "tcp", robot: s.selectedRobot ?? "" }
            : sel,
        };
      });
    } else if (msg.type === "scenarios") {
      set({ scenarios: msg.scenarios });
    } else if (msg.type === "sequence_result") {
      if (msg.ok && msg.timeline) {
        // The SFC chart's flatten mirror must agree with the rollout's —
        // any drift shows up on the first bake, as warnings, not as a
        // silently wrong chart (see sfc.ts).
        for (const issue of attributionIssues(get().sequences, {
          stepSpans: msg.timeline.step_spans,
          branches: msg.timeline.branches,
        })) {
          console.warn(`sfc chart: ${issue}`);
        }
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
            branches: msg.timeline.branches,
            scenario: msg.scenario ?? null,
          },
          segmentEnds: msg.timeline.step_spans.map((s) => s.end),
          ...startPlayback(tracksFromTimeline(msg.timeline)),
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
            branches: msg.timeline.branches,
            scenario: null,
          },
          segmentEnds: [],
          ...startPlayback(tracksFromTimeline(msg.timeline)),
          motionStats: null,
          motionError: null,
        });
      } else {
        set({
          recording: null,
          recordingError: msg.error ?? "recording import failed",
        });
      }
    } else if (msg.type === "usd_document") {
      // The baked layer arrives as text; hand it straight to the browser
      // as a file download. Warnings and refusals surface on the
      // sequence-error line — that is where the export button lives.
      if (msg.ok && msg.text) {
        const blob = new Blob([msg.text], { type: "text/plain" });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = msg.name || "cell.usda";
        a.click();
        URL.revokeObjectURL(url);
        set({
          sequenceError:
            msg.warnings.length > 0 ? msg.warnings.join("\n") : null,
        });
      } else {
        set({ sequenceError: msg.error ?? "usd export failed" });
      }
    } else if (msg.type === "motion_result") {
      if (msg.ok && msg.trajectory) {
        // Auto-start the preview through the shared playback state.
        set({
          motionPlanning: false,
          motionError: null,
          motionStats: { planningTimeMs: msg.planning_time_ms ?? 0 },
          segmentEnds: msg.segment_ends,
          ...startPlayback(tracksFromTrajectory(msg.robot, msg.trajectory)),
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
  selectGroup: (path) => set({ selection: { type: "group", path } }),
  selectSensor: (name) => set({ selection: { type: "sensor", name } }),
  selectDevice: (name) => set({ selection: { type: "device", name } }),
  selectMotion: (name) =>
    set((s) => {
      const motion = s.motions.find((m) => m.name === name);
      const owner = motion
        ? (motion.robot ?? s.robots[0]?.desc.name ?? null)
        : null;
      if (owner === null || owner === s.selectedRobot) {
        return { selectedMotion: name };
      }
      return {
        selectedMotion: name,
        selectedRobot: owner,
        selection: retargetSelection(s.selection, owner),
      };
    }),
  selectFromViewport: (name) => {
    const { obstacles, selection } = get();
    const chain = drillChain(
      name,
      obstacles.map((o: ObstacleMsg) => o.name),
    );
    // Already somewhere in this obstacle's chain? Step one level in.
    // Anything else — empty space, another machine — starts over at the top.
    const current =
      selection.type === "group"
        ? selection.path
        : selection.type === "obstacle" && selection.name === name
          ? name
          : null;
    const at = current === null ? -1 : chain.indexOf(current);
    const next = chain[Math.min(at + 1, chain.length - 1)];
    set({
      selection:
        next === name
          ? { type: "obstacle", name }
          : { type: "group", path: next },
    });
    get().focusTab("obstacle");
  },
  selectRobot: (robot) =>
    set({ selection: { type: "robot", robot }, selectedRobot: robot }),
  setSelectedRobot: (robot) =>
    set((s) => ({
      selectedRobot: robot,
      selection: retargetSelection(s.selection, robot),
      // A motion follows its owner; anything else (another robot's motion,
      // a not-yet-created name) falls back to the new robot's own.
      selectedMotion:
        s.selectedMotion !== null &&
        s.motions.some(
          (m) =>
            m.name === s.selectedMotion &&
            (m.robot ?? s.robots[0]?.desc.name) === robot,
        )
          ? s.selectedMotion
          : null,
    })),
  setActiveTab: (tab) => set({ activeTab: persistTab(tab) }),
  setSfcOpen: (open) => set({ sfcOpen: persistSfcOpen(open) }),
  focusTab: (target) =>
    set((s) =>
      s.activeTab === "sequence"
        ? {}
        : { activeTab: persistTab(target === "robot" ? "motion" : "layout") },
    ),

  beginSequenceSim: () => set({ sequenceSimulating: true, sequenceError: null }),
  beginMotionPlanning: () => set({ motionPlanning: true, motionError: null }),

  setPlayback: (t, sample) =>
    set({
      playbackTime: t,
      overridePoses: sample.poses,
      overrideJoints: sample.joints,
      overrideBases: sample.bases,
      overrideObstaclePoses: sample.objects,
      stowedObstacles: sample.stowed,
    }),
  setPlaying: (playing) => set({ playing }),
  setPlaybackTime: (t) => set({ playbackTime: t }),
  setPlaybackSpeed: (speed) => set({ playbackSpeed: speed }),
  setPlaybackLoop: (loop) => set({ playbackLoop: loop }),
  stopPlayback: () =>
    set({
      playing: false,
      overridePoses: null,
      overrideJoints: null,
      overrideBases: null,
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
