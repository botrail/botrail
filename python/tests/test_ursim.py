"""End-to-end check of the URScript export against URSim (the Universal
Robots controller simulator).

Skipped unless ``BOTRAIL_URSIM_HOST`` is set. The easiest way to run:

    scripts/ursim_test.sh            # docker URSim + these tests + teardown

or against an already-running simulator:

    docker run --rm -d -e ROBOT_MODEL=UR5 \
        -p 29999:29999 -p 30001-30004:30001-30004 universalrobots/ursim_e-series
    BOTRAIL_URSIM_HOST=127.0.0.1 .venv/bin/pytest python/tests/test_ursim.py -v

The exported script is sent over the primary interface (port 30001) while
the joint stream is read from the real-time interface (port 30003). With
``blend_radius=0`` every sparse waypoint is an exact stop, so the stream
must pass through each of them and settle at the last one.

Note the test robot is ``simple_arm.urdf``, not a UR: joint-valued targets
transfer verbatim to any 6-axis controller, so the assertions are at the
joint level. Test postures keep wrist_2 far from 0 (a UR wrist singularity)
so the ``movel`` check cannot abort on the controller.
"""

import os
import socket
import struct
import time
from pathlib import Path

import pytest

import botrail as bt

URSIM_HOST = os.environ.get("BOTRAIL_URSIM_HOST")

pytestmark = pytest.mark.skipif(
    URSIM_HOST is None,
    reason="URSim integration: set BOTRAIL_URSIM_HOST (see scripts/ursim_test.sh)",
)

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"

DASHBOARD_PORT = 29999
PRIMARY_PORT = 30001
REALTIME_PORT = 30003

# Real-time (30003) packet layout: int32 size, time (1 double), q_target /
# qd_target / qdd_target / I_target / M_target (6 doubles each), then
# q_actual. Stable across CB3 and e-Series.
Q_ACTUAL_OFFSET = 4 + 8 + 5 * 48

# UR-safe postures (also collision-free for simple_arm): elbow bent and
# wrist_2 well away from the q5=0 wrist singularity.
START = [0.0, -1.2, 1.5, -1.8, -1.4, 0.3]


def linf(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


class UrSim:
    """Minimal test client: dashboard bring-up, script upload, joint stream."""

    def __init__(self, host: str):
        self.host = host

    def _dashboard(self, *commands: str) -> list[str]:
        with socket.create_connection((self.host, DASHBOARD_PORT), timeout=5.0) as s:
            f = s.makefile("rw", newline="\n")
            f.readline()  # welcome banner
            replies = []
            for command in commands:
                f.write(command + "\n")
                f.flush()
                replies.append(f.readline().strip())
            return replies

    def robotmode(self) -> str:
        return self._dashboard("robotmode")[0]

    def prepare(self, timeout: float = 120.0) -> None:
        """Brings the arm to RUNNING: close popups, power on, release brakes."""
        self._dashboard("close safety popup", "close popup", "stop")
        deadline = time.time() + timeout
        while time.time() < deadline:
            mode = self.robotmode()
            if mode.endswith("RUNNING"):
                return
            if mode.endswith("CONFIRM_SAFETY"):
                self._dashboard("close safety popup")
            elif mode.endswith("POWER_OFF"):
                self._dashboard("power on")
            elif mode.endswith("IDLE"):
                self._dashboard("brake release")
            time.sleep(1.0)
        pytest.fail(f"URSim not RUNNING within {timeout}s (last: {self.robotmode()!r})")

    def run_script(self, script: str) -> None:
        with socket.create_connection((self.host, PRIMARY_PORT), timeout=5.0) as s:
            s.sendall(script.encode())

    @staticmethod
    def _recv_exact(sock: socket.socket, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("realtime stream closed")
            buf += chunk
        return buf

    def wait_for_motion(self, targets, timeout=90.0, tol=5e-3):
        """Streams joints until the arm settles at the last target; returns
        the closest approach (L-inf, rad) to every target along the way."""
        final = targets[-1]
        best = [float("inf")] * len(targets)
        settled = 0
        prev = None
        with socket.create_connection((self.host, REALTIME_PORT), timeout=5.0) as s:
            s.settimeout(5.0)
            deadline = time.time() + timeout
            while time.time() < deadline:
                (size,) = struct.unpack(">i", self._recv_exact(s, 4))
                body = self._recv_exact(s, size - 4)
                if size < Q_ACTUAL_OFFSET + 48:
                    continue
                q = struct.unpack(">6d", body[Q_ACTUAL_OFFSET - 4 : Q_ACTUAL_OFFSET + 44])
                for i, target in enumerate(targets):
                    best[i] = min(best[i], linf(q, target))
                still = prev is not None and linf(q, prev) < 1e-5
                prev = q
                if still and linf(q, final) < tol:
                    settled += 1
                    # The stream runs at 125-500 Hz: ~0.2-1s of stillness.
                    if settled >= 100:
                        return best
                else:
                    settled = 0
        pytest.fail(
            f"arm did not settle at the final target within {timeout}s; "
            f"closest approaches: {[round(b, 4) for b in best]}, "
            f"robotmode: {self.robotmode()!r}"
        )


@pytest.fixture(scope="module")
def ursim() -> UrSim:
    sim = UrSim(URSIM_HOST)
    sim.prepare()
    return sim


@pytest.fixture()
def scene() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))
    scene.set_joint_positions(START)
    return scene


def test_joint_motion_visits_every_waypoint(ursim: UrSim, scene: bt.Scene) -> None:
    g1 = [0.8, -1.0, 1.2, -1.6, -1.2, 0.1]
    g2 = [-0.5, -1.3, 1.6, -1.7, -1.6, 0.5]
    scene.add_segment("m", goal=g1)
    scene.add_segment("m", goal=g2)
    traj = scene.plan_motion("m", broadcast=False)

    ursim.run_script(traj.to_script(name="botrail_e2e"))

    # With r=0 the controller stops exactly at every sparse waypoint, so
    # the joint stream must pass through all of them (start, any RRT
    # detour points, g1, g2) and settle at g2.
    waypoints = [q for _, wps in traj.segments for q in wps]
    best = ursim.wait_for_motion(waypoints)
    for i, closest in enumerate(best):
        assert closest < 5e-3, f"waypoint {i} missed by {closest:.4f} rad"


def test_cartesian_segment_lands_on_planned_branch(ursim: UrSim, scene: bt.Scene) -> None:
    (x, y, z), quat = scene.link_pose("tool0")
    ik = scene.robot.ik((x, y, z + 0.06), quat, seed=START)
    assert ik.converged

    scene.add_segment("ascend", goal=ik.q, kind="cartesian_line")
    traj = scene.plan_motion("ascend", broadcast=False)
    script = traj.to_script(name="botrail_e2e_lin", tcp_speed=0.1)
    assert script.count("movel(") == 1

    ursim.run_script(script)
    # movel toward a joint-valued target: the controller FKs the target
    # and tracks the line, so it must land on the planned IK branch.
    best = ursim.wait_for_motion([START, ik.q], tol=2e-2)
    assert best[0] < 5e-3  # move-to-start reached exactly
    assert best[1] < 2e-2  # movel landed on the planned configuration
