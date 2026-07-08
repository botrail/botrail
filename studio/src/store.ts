import { create } from "zustand";
import type {
  CollisionPairMsg,
  IkStatusMsg,
  ObstacleMsg,
  PoseMsg,
  SceneDescriptionMsg,
  ServerMessage,
} from "./protocol";

export type ConnectionStatus = "connecting" | "connected" | "disconnected";
export type GizmoMode = "translate" | "rotate";

/** What the viewport gizmo is currently editing. */
export type Selection = { type: "tcp" } | { type: "obstacle"; name: string };

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

interface StudioState {
  sceneDesc: SceneDescriptionMsg | null;
  /** Joint position vector, indexed by q_index (length = DOF). */
  jointPositions: number[];
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
  /** Colliding pairs at the current configuration (empty when clear). */
  collisions: CollisionPairMsg[];
  /** Minimum robot-obstacle distance; null without obstacles. */
  minDistance: number | null;
  /** Which object the viewport gizmo edits; defaults to the TCP. */
  selection: Selection;

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
}

export const useStudioStore = create<StudioState>((set) => ({
  sceneDesc: null,
  jointPositions: [],
  linkPoses: [],
  connection: "connecting",
  ikStatus: null,
  tcpLink: null,
  gizmoMode: "translate",
  obstacles: [],
  collisions: [],
  minDistance: null,
  selection: { type: "tcp" },

  setConnection: (c) => set({ connection: c }),

  applyServerMessage: (msg) => {
    if (msg.type === "scene_init") {
      set({
        sceneDesc: msg.scene,
        jointPositions: new Array(actuatedDof(msg.scene)).fill(0),
        linkPoses: [],
        ikStatus: null,
        tcpLink: msg.scene.tcp_link,
        obstacles: [],
        collisions: [],
        minDistance: null,
        selection: { type: "tcp" },
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
    } else {
      set({
        jointPositions: msg.joint_positions,
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
}));
