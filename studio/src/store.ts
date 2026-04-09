import { create } from "zustand";
import type {
  IkStatusMsg,
  PoseMsg,
  SceneDescriptionMsg,
  ServerMessage,
} from "./protocol";

export type ConnectionStatus = "connecting" | "connected" | "disconnected";
export type GizmoMode = "translate" | "rotate";

/** Actuated joints (those with a q_index), in q_index order. */
function actuatedDof(scene: SceneDescriptionMsg): number {
  return scene.joints.filter((j) => j.q_index !== null).length;
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.min(Math.max(v, lo), hi);
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

  setConnection: (c: ConnectionStatus) => void;
  applyServerMessage: (msg: ServerMessage) => void;
  /** Optimistic local update of a single DOF. */
  setJointPosition: (qIndex: number, value: number) => void;
  /** Reset every DOF to 0 (clamped into limits when present). */
  resetJoints: () => void;
  setTcpLink: (link: string) => void;
  setGizmoMode: (mode: GizmoMode) => void;
}

export const useStudioStore = create<StudioState>((set) => ({
  sceneDesc: null,
  jointPositions: [],
  linkPoses: [],
  connection: "connecting",
  ikStatus: null,
  tcpLink: null,
  gizmoMode: "translate",

  setConnection: (c) => set({ connection: c }),

  applyServerMessage: (msg) => {
    if (msg.type === "scene_init") {
      set({
        sceneDesc: msg.scene,
        jointPositions: new Array(actuatedDof(msg.scene)).fill(0),
        linkPoses: [],
        ikStatus: null,
        tcpLink: msg.scene.tcp_link,
      });
    } else {
      set({
        jointPositions: msg.joint_positions,
        linkPoses: msg.link_poses,
        ikStatus: msg.ik_status,
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
}));
