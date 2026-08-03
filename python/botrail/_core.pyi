from pathlib import Path
from typing import Optional, Union

__version__: str

class IkResult:
    @property
    def q(self) -> list[float]: ...
    @property
    def converged(self) -> bool: ...
    @property
    def pos_error(self) -> float: ...
    @property
    def rot_error(self) -> float: ...
    @property
    def iters(self) -> int: ...

class Robot:
    @staticmethod
    def from_urdf(path: Union[str, Path]) -> "Robot": ...
    @staticmethod
    def from_urdf_string(xml: str) -> "Robot": ...
    @staticmethod
    def from_xacro(path: Union[str, Path]) -> "Robot": ...
    @staticmethod
    def from_usd(
        path: Union[str, Path],
        articulation_root: Optional[str] = None,
        search_paths: Optional[list[Union[str, Path]]] = None,
    ) -> "Robot": ...
    @property
    def name(self) -> str: ...
    @property
    def dof(self) -> int: ...
    @property
    def joint_names(self) -> list[str]: ...
    @property
    def joint_limits(self) -> list[Optional[tuple[float, float]]]: ...
    @property
    def link_names(self) -> list[str]: ...
    @property
    def tcp_link(self) -> str: ...
    def ik(
        self,
        position: tuple[float, float, float],
        quaternion: Optional[tuple[float, float, float, float]] = None,
        link: Optional[str] = None,
        seed: Optional[list[float]] = None,
        max_iters: int = 100,
    ) -> IkResult: ...

class Scene:
    def __init__(
        self,
        robot: Robot,
        base_position: Optional[tuple[float, float, float]] = None,
        base_quaternion: Optional[tuple[float, float, float, float]] = None,
        name: Optional[str] = None,
    ) -> None: ...
    def add_robot(
        self,
        robot: Robot,
        name: Optional[str] = None,
        base_position: Optional[tuple[float, float, float]] = None,
        base_quaternion: Optional[tuple[float, float, float, float]] = None,
    ) -> str: ...
    @property
    def robot(self) -> Robot: ...
    @property
    def robots(self) -> list[str]: ...
    def robot_of(self, name: str) -> Robot: ...
    @property
    def robot_base_pose(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]: ...
    def robot_base_pose_of(
        self, name: str
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]: ...
    def set_robot_base_pose(
        self,
        position: tuple[float, float, float],
        quaternion: Optional[tuple[float, float, float, float]] = None,
        robot: Optional[str] = None,
    ) -> None: ...
    @property
    def joint_positions(self) -> list[float]: ...
    def joint_positions_of(self, name: str) -> list[float]: ...
    def set_joint_positions(
        self, positions: list[float], robot: Optional[str] = None
    ) -> None: ...
    def link_pose(
        self, link_name: str, robot: Optional[str] = None
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]: ...
    def set_tcp_target(
        self,
        position: tuple[float, float, float],
        quaternion: Optional[tuple[float, float, float, float]] = None,
        link: Optional[str] = None,
        max_iters: int = 100,
        robot: Optional[str] = None,
    ) -> IkResult: ...
    def add_box(
        self,
        name: str,
        size: tuple[float, float, float],
        position: tuple[float, float, float],
        quaternion: Optional[tuple[float, float, float, float]] = None,
        color: Optional[tuple[float, float, float]] = None,
    ) -> str: ...
    def add_sphere(
        self,
        name: str,
        radius: float,
        position: tuple[float, float, float],
        quaternion: Optional[tuple[float, float, float, float]] = None,
        color: Optional[tuple[float, float, float]] = None,
    ) -> str: ...
    def add_cylinder(
        self,
        name: str,
        radius: float,
        length: float,
        position: tuple[float, float, float],
        quaternion: Optional[tuple[float, float, float, float]] = None,
        color: Optional[tuple[float, float, float]] = None,
    ) -> str: ...
    def add_mesh(
        self,
        name: str,
        path: Union[str, Path],
        position: tuple[float, float, float],
        scale: Optional[tuple[float, float, float]] = None,
        quaternion: Optional[tuple[float, float, float, float]] = None,
        color: Optional[tuple[float, float, float]] = None,
    ) -> str: ...
    def load_usd(
        self,
        path: Union[str, Path],
        prefix: Optional[str] = None,
        search_paths: Optional[list[Union[str, Path]]] = None,
    ) -> list[str]: ...
    def add_frame(
        self,
        name: str,
        position: tuple[float, float, float],
        quaternion: Optional[tuple[float, float, float, float]] = None,
    ) -> None: ...
    @property
    def frames(
        self,
    ) -> dict[
        str, tuple[tuple[float, float, float], tuple[float, float, float, float]]
    ]: ...
    def frame(
        self, name: str
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]: ...
    def remove_obstacle(self, name: str) -> None: ...
    def set_obstacle_enabled(self, name: str, enabled: bool) -> None: ...
    def set_obstacle_color(
        self, name: str, color: Optional[tuple[float, float, float]]
    ) -> None: ...
    def obstacle_color(self, name: str) -> Optional[tuple[float, float, float]]: ...
    def rename_robot(self, robot: str, name: str) -> str: ...
    def allow_inter_robot_collision(
        self, robot_a: str, link_a: str, robot_b: str, link_b: str
    ) -> None: ...
    def set_obstacle_pose(
        self,
        name: str,
        position: tuple[float, float, float],
        quaternion: Optional[tuple[float, float, float, float]] = None,
    ) -> None: ...
    @property
    def obstacle_names(self) -> list[str]: ...
    def obstacle_pose(
        self, name: str
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]: ...
    def attach(
        self,
        name: str,
        link: Optional[str] = None,
        touch_links: Optional[list[str]] = None,
        robot: Optional[str] = None,
    ) -> None: ...
    def detach(self, name: str) -> None: ...
    @property
    def attachments(self) -> list[tuple[str, str]]: ...
    def export_usd(
        self,
        trajectory: Trajectory,
        path: Union[str, Path],
        fps: float = 60.0,
        robot: Optional[str] = None,
    ) -> list[str]: ...
    def play_usd_animation(
        self,
        path: Union[str, Path],
        force_transforms: bool = False,
        robot_roots: Optional[dict[str, str]] = None,
    ) -> dict: ...
    def define_signal(self, name: str, initial: bool = False) -> None: ...
    def remove_signal(self, name: str) -> None: ...
    @property
    def signals(self) -> list[tuple[str, bool]]: ...
    def sequence(self, name: str): ...  # -> botrail.seq.SequenceBuilder
    def _upsert_sequence_json(self, json: str) -> None: ...
    def remove_sequence(self, name: str) -> None: ...
    @property
    def sequence_names(self) -> list[str]: ...
    def simulate_sequence(
        self,
        name: str,
        dt: float = 0.01,
        max_duration: float = 120.0,
    ) -> "SequenceTimeline": ...
    def add_zone_sensor(
        self,
        name: str,
        position: tuple[float, float, float],
        size: tuple[float, float, float],
        quaternion: Optional[tuple[float, float, float, float]] = None,
        watch: Optional[list[str]] = None,
        watch_robot: bool = False,
        watch_robots: Optional[list[str]] = None,
    ) -> None: ...
    def add_beam_sensor(
        self,
        name: str,
        frm: tuple[float, float, float],
        to: tuple[float, float, float],
        radius: float = 0.005,
        watch: Optional[list[str]] = None,
        watch_robot: bool = False,
        watch_robots: Optional[list[str]] = None,
    ) -> None: ...
    def remove_sensor(self, name: str) -> None: ...
    @property
    def sensor_names(self) -> list[str]: ...
    def add_conveyor(
        self,
        name: str,
        zone_position: tuple[float, float, float],
        zone_size: tuple[float, float, float],
        velocity: tuple[float, float, float],
        zone_quaternion: Optional[tuple[float, float, float, float]] = None,
        running: bool = True,
    ) -> None: ...
    def add_source(
        self,
        name: str,
        pool: list[str],
        park: tuple[float, float, float],
        position: tuple[float, float, float],
        pitch: Optional[tuple[float, float, float]] = None,
        interval: float = 0.0,
        running: bool = False,
    ) -> None: ...
    def add_sink(
        self,
        name: str,
        zone_position: tuple[float, float, float],
        zone_size: tuple[float, float, float],
        source: str,
        zone_quaternion: Optional[tuple[float, float, float, float]] = None,
    ) -> None: ...
    def add_linear_axis(
        self,
        name: str,
        objects: list[str],
        axis: tuple[float, float, float],
        speed: float,
        range: tuple[float, float],
        position: float = 0.0,
    ) -> None: ...
    def remove_device(self, name: str) -> None: ...
    @property
    def device_names(self) -> list[str]: ...
    def check_collisions(
        self,
    ) -> list[tuple[tuple[str, str], tuple[str, str]]]: ...
    def in_collision(self) -> bool: ...
    def min_obstacle_distance(self) -> Optional[float]: ...
    @property
    def collision_warnings(self) -> list[str]: ...
    def plan(
        self,
        goal: list[float],
        max_iters: int = 10000,
        seed: Optional[int] = None,
        broadcast: bool = True,
        robot: Optional[str] = None,
    ) -> Trajectory: ...
    def plan_to_pose(
        self,
        position: tuple[float, float, float],
        quaternion: Optional[tuple[float, float, float, float]] = None,
        link: Optional[str] = None,
        max_iters: int = 10000,
        seed: Optional[int] = None,
        broadcast: bool = True,
        robot: Optional[str] = None,
    ) -> Trajectory: ...
    def add_segment(
        self,
        motion: str,
        goal: Optional[list[float]] = None,
        kind: str = "joint",
        orientation_cone: Optional[
            tuple[tuple[float, float, float], tuple[float, float, float], float]
        ] = None,
        position_box: Optional[
            tuple[tuple[float, float, float], tuple[float, float, float]]
        ] = None,
        robot: Optional[str] = None,
    ) -> None: ...
    def remove_segment(self, motion: str, index: int) -> None: ...
    def clear_motion(self, motion: str) -> None: ...
    @property
    def motion_names(self) -> list[str]: ...
    def motion_segments(self, name: str) -> list[tuple[str, list[float]]]: ...
    def plan_motion(
        self, motion: str, seed: Optional[int] = None, broadcast: bool = True
    ) -> Trajectory: ...
    def save_project(self, path: Union[str, Path]) -> None: ...
    @staticmethod
    def load_project(path: Union[str, Path]) -> "Scene": ...
    def generate_python(self) -> str: ...

class Trajectory:
    @property
    def joint_names(self) -> list[str]: ...
    @property
    def times(self) -> list[float]: ...
    @property
    def positions(self) -> list[list[float]]: ...
    @property
    def velocities(self) -> list[list[float]]: ...
    @property
    def duration(self) -> float: ...
    @property
    def segment_ends(self) -> list[float]: ...
    @property
    def segments(self) -> list[tuple[str, list[list[float]]]]: ...
    def sample(self, t: float) -> list[float]: ...
    def export_json(self, path: Union[str, Path]) -> None: ...
    def export_csv(self, path: Union[str, Path], dt: Optional[float] = None) -> None: ...
    def to_script(
        self,
        dialect: str = "urscript",
        name: str = "botrail_program",
        speed_scale: float = 1.0,
        blend_radius: float = 0.0,
        tcp_speed: float = 0.25,
        tcp_accel: float = 1.2,
        move_to_start: bool = True,
    ) -> str: ...
    def export_script(
        self,
        path: Union[str, Path],
        dialect: str = "urscript",
        name: Optional[str] = None,
        speed_scale: float = 1.0,
        blend_radius: float = 0.0,
        tcp_speed: float = 0.25,
        tcp_accel: float = 1.2,
        move_to_start: bool = True,
    ) -> None: ...

class SequenceTimeline:
    @property
    def duration(self) -> float: ...
    @property
    def robots(self) -> list[str]: ...
    @property
    def step_spans(self) -> list[tuple[str, float, float]]: ...
    @property
    def signals(self) -> list[tuple[str, list[tuple[float, bool]]]]: ...
    def sample(self, t: float, robot: Optional[str] = None) -> list[float]: ...
    def moves(
        self, robot: Optional[str] = None
    ) -> list[tuple[str, float, float]]: ...
    def object_pose(
        self, name: str, t: float
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]: ...
    def object_visible(self, name: str, t: float) -> bool: ...
    def robot_trajectory(self, robot: Optional[str] = None) -> Trajectory: ...
    @property
    def trajectory(self) -> Trajectory: ...
    def export_usd(
        self, path: Union[str, Path], fps: float = 60.0
    ) -> list[str]: ...
    def step_span(self, name: str) -> Span: ...
    def signal(self, name: str) -> SignalTrack: ...
    def min_clearance(self, dt: float = 0.01) -> Clearance: ...

class Span:
    @property
    def name(self) -> str: ...
    @property
    def start(self) -> float: ...
    @property
    def end(self) -> float: ...
    @property
    def duration(self) -> float: ...

class SignalTrack:
    @property
    def name(self) -> str: ...
    @property
    def edges(self) -> list[tuple[float, bool]]: ...
    def value_at(self, t: float) -> bool: ...
    def rising_edges(self) -> list[float]: ...
    def falling_edges(self) -> list[float]: ...
    def high_spans(self) -> list[tuple[float, float]]: ...
    def high_total(self) -> float: ...

class Clearance:
    @property
    def distance(self) -> float: ...
    @property
    def t(self) -> float: ...
    @property
    def pair(self) -> Optional[tuple[str, str]]: ...
    def __float__(self) -> float: ...
    def __lt__(self, value: Union[Clearance, float]) -> bool: ...
    def __le__(self, value: Union[Clearance, float]) -> bool: ...
    def __gt__(self, value: Union[Clearance, float]) -> bool: ...
    def __ge__(self, value: Union[Clearance, float]) -> bool: ...

class StudioServer:
    @property
    def url(self) -> str: ...
    def stop(self) -> None: ...

def serve_studio(
    scene: Scene, studio_dir: str, host: str = "127.0.0.1", port: int = 0
) -> StudioServer: ...
