from pathlib import Path
from typing import Any, Optional, Union

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
    @staticmethod
    def from_catalog(
        id: str,
        revision: Optional[str] = None,
        format: Optional[str] = None,
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
    def mimic_joints(self) -> dict[str, tuple[str, float, float]]: ...
    def joint_values(self, positions: list[float]) -> dict[str, float]: ...
    @property
    def link_names(self) -> list[str]: ...
    @property
    def tcp_link(self) -> str: ...
    @property
    def flange_link(self) -> Optional[str]: ...
    @property
    def mount_link(self) -> Optional[str]: ...
    def ik(
        self,
        position: tuple[float, float, float],
        quaternion: Optional[tuple[float, float, float, float]] = None,
        link: Optional[str] = None,
        seed: Optional[list[float]] = None,
        max_iters: int = 100,
        restarts: Optional[int] = None,
    ) -> IkResult: ...
    def attach_tool(
        self,
        tool: "Robot",
        flange: Optional[str] = None,
        mount: Optional[str] = None,
        offset_position: Optional[tuple[float, float, float]] = None,
        offset_quaternion: Optional[tuple[float, float, float, float]] = None,
        tcp: Optional[str] = None,
        prefix: Optional[str] = None,
    ) -> "Robot": ...

class Scene:
    def __init__(
        self,
        robot: Optional[Robot] = None,
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
    def mount_robot(
        self,
        device: str,
        offset_position: Optional[tuple[float, float, float]] = None,
        offset_quaternion: Optional[tuple[float, float, float, float]] = None,
        robot: Optional[str] = None,
        gait: Optional[Any] = None,
        spin: Optional[dict[str, float]] = None,
    ) -> None: ...
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
    def load_urdf(
        self,
        path: Union[str, Path],
        prefix: Optional[str] = None,
        position: Optional[tuple[float, float, float]] = None,
        quaternion: Optional[tuple[float, float, float, float]] = None,
        args: Optional[dict[str, str]] = None,
        geometry: str = "visual",
        frames: bool = True,
        package_paths: Optional[dict[str, Union[str, Path]]] = None,
    ) -> list[str]: ...
    def add_frame(
        self,
        name: str,
        position: tuple[float, float, float],
        quaternion: Optional[tuple[float, float, float, float]] = None,
    ) -> None: ...
    def remove_frame(self, name: str) -> None: ...
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
    def set_obstacle_visible(self, name: str, visible: bool) -> None: ...
    def set_obstacle_walkable(self, name: str, walkable: bool = True) -> None: ...
    def set_obstacle_color(
        self, name: str, color: Optional[tuple[float, float, float]]
    ) -> None: ...
    def obstacle_color(self, name: str) -> Optional[tuple[float, float, float]]: ...
    def set_obstacle_material(
        self,
        name: str,
        metalness: Optional[float] = None,
        roughness: Optional[float] = None,
    ) -> None: ...
    def set_obstacle_legend(
        self,
        name: str,
        title: str = "",
        stops: Optional[list[tuple[tuple[float, float, float], str]]] = None,
    ) -> None: ...
    def obstacle_material(self, name: str) -> Optional[tuple[float, float]]: ...
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
    def obstacle_bounds(
        self, name: str
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]: ...
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
        path: Union[str, Path],
        trajectory: Optional[Trajectory] = None,
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
    def add_weld_flash(self, name: str, signal: str, robot: str) -> None: ...
    def add_cut_trace(
        self,
        name: str,
        signal: str,
        robot: str,
        spin_link: Optional[str] = None,
    ) -> None: ...
    def add_spray_cone(
        self,
        name: str,
        signal: str,
        robot: str,
        length: float = 0.25,
        radius: float = 0.08,
    ) -> None: ...
    @property
    def signals(self) -> list[tuple[str, bool]]: ...
    def sequence(self, name: str): ...  # -> botrail.seq.SequenceBuilder
    def link_pose_at(
        self, link_name: str, joints: list[float], robot: Optional[str] = None
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]: ...
    def _project_json(self) -> str: ...
    def requirements(
        self, *, sequences: Optional[list[str]] = None, margin: float = 0.1, timeline: Optional[Any] = None
    ): ...  # -> botrail.select.Requirements
    def check(
        self, *, sequences: Optional[list[str]] = None, timeline: Optional[Any] = None
    ): ...  # -> botrail.select.CheckReport
    def _upsert_sequence_json(self, json: str) -> None: ...
    def remove_sequence(self, name: str) -> None: ...
    @property
    def sequence_names(self) -> list[str]: ...
    def allow_link_obstacle_contact(
        self, link: str, obstacle: str, robot: Optional[str] = None
    ) -> None: ...
    def disallow_link_obstacle_contact(
        self, link: str, obstacle: str, robot: Optional[str] = None
    ) -> None: ...
    def add_toolpath(self, name: str, toolpath: Union[dict, str]) -> None: ...
    def remove_toolpath(self, name: str) -> None: ...
    @property
    def toolpath_names(self) -> list[str]: ...
    def plan_toolpath(
        self,
        name: str,
        robot: Optional[str] = None,
        tcp_link: Optional[str] = None,
        step_pos: float = 0.005,
        step_rot: float = 0.05,
        jump_threshold: float = 0.5,
        rapid_speed: Optional[float] = None,
        axis_tolerance: float = 0.0,
        spin: str = "greedy",
    ) -> "Trajectory": ...
    def check_toolpath(
        self,
        name: str,
        robot: Optional[str] = None,
        tcp_link: Optional[str] = None,
        step_pos: float = 0.005,
        step_rot: float = 0.05,
        jump_threshold: float = 0.5,
        axis_tolerance: float = 0.0,
        spin: str = "greedy",
    ) -> "ToolpathReport": ...
    def define_applicator(self, name: str, applicator: Union[dict, str]) -> None: ...
    def define_brush(
        self,
        name: str,
        applicator: str,
        flow: float = 1.0,
        lead: float = 0.0,
        lag: float = 0.0,
    ) -> None: ...
    def remove_applicator(self, name: str) -> None: ...
    def remove_brush(self, name: str) -> None: ...
    @property
    def applicator_names(self) -> list[str]: ...
    @property
    def brush_names(self) -> list[str]: ...
    def brush(self, name: str) -> dict: ...
    def check_paint(
        self,
        name: str,
        target: str,
        standoff: Optional[tuple[float, float]] = None,
        max_incidence: float = ...,
        max_range: Optional[float] = None,
        step_pos: float = 0.005,
        step_rot: float = 0.05,
    ) -> "PaintReport": ...
    def clear_toolpath_marks(self, name: str) -> None: ...
    def show_film(self, film: "FilmCoat", name: Optional[str] = None) -> str: ...
    def animate_paint(
        self,
        timeline: "SequenceTimeline",
        target: str,
        applicator: Optional[Union[dict, str]] = None,
        stages: Optional[int] = None,
        patch_size: float = 0.01,
        dt: float = 0.01,
        gate: Optional[str] = None,
        spec: Optional[tuple[float, float]] = None,
        facing: Optional[tuple[float, float, float]] = None,
        facing_tolerance: float = ...,
        occlusion: bool = True,
        robot: Optional[str] = None,
        tcp_link: Optional[str] = None,
        trigger_signal: Optional[str] = None,
        style: str = "amount",
        paint_color: Optional[tuple[float, float, float]] = None,
        substrate: Optional[tuple[float, float, float]] = None,
    ) -> "SequenceTimeline": ...
    def animate_carve(
        self,
        timeline: "SequenceTimeline",
        stock: str,
        stages: Optional[int] = None,
        voxel_size: float = 0.001,
        cutter_radius: float = 0.004,
        cutter_length: float = 0.03,
        dt: float = 0.01,
        robot: Optional[str] = None,
        tcp_link: Optional[str] = None,
    ) -> "SequenceTimeline": ...
    def timeline_from_trajectory(
        self,
        trajectory: "Trajectory",
        robot: Optional[str] = None,
        label: str = "trajectory",
    ) -> "SequenceTimeline": ...
    def simulate_sequence(
        self,
        name: str,
        dt: float = 0.01,
        max_duration: float = 120.0,
        plan_resolution: Optional[float] = None,
        scenario: Optional[str] = None,
        toolpath_spin: Optional[str] = None,
    ) -> "SequenceTimeline": ...
    def simulate_sequences(
        self,
        names: list[str],
        dt: float = 0.01,
        max_duration: float = 120.0,
        plan_resolution: Optional[float] = None,
        scenario: Optional[str] = None,
        toolpath_spin: Optional[str] = None,
    ) -> "SequenceTimeline": ...
    def add_scenario(
        self,
        name: str,
        signals: Optional[dict[str, bool]] = None,
        obstacles: Optional[
            dict[
                str,
                Union[
                    tuple[float, float, float],
                    tuple[
                        tuple[float, float, float],
                        tuple[float, float, float, float],
                    ],
                ],
            ]
        ] = None,
        joints: Optional[dict[str, list[float]]] = None,
        faults: Optional[list[dict]] = None,
    ) -> None: ...
    def remove_scenario(self, name: str) -> None: ...
    @property
    def scenario_names(self) -> list[str]: ...
    def simulate_scenarios(
        self,
        names: list[str],
        scenarios: Optional[list[str]] = None,
        dt: float = 0.01,
        max_duration: float = 120.0,
        plan_resolution: Optional[float] = None,
    ) -> "ScenarioRuns": ...
    def add_zone_sensor(
        self,
        name: str,
        position: tuple[float, float, float],
        size: tuple[float, float, float],
        quaternion: Optional[tuple[float, float, float, float]] = None,
        watch: Optional[list[str]] = None,
        watch_robot: bool = False,
        watch_robots: Optional[list[str]] = None,
        mount: Optional[str] = None,
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
        mount: Optional[str] = None,
    ) -> None: ...
    def add_vision_sensor(
        self,
        name: str,
        camera: str,
        watch: Optional[list[str]] = None,
        watch_robot: bool = False,
        watch_robots: Optional[list[str]] = None,
        detect_range: Optional[tuple[float, float]] = None,
        occlusion: bool = True,
    ) -> None: ...
    def add_field_sensor(
        self,
        name: str,
        lidar: str,
        watch: Optional[list[str]] = None,
        watch_robot: bool = False,
        watch_robots: Optional[list[str]] = None,
        range: Optional[float] = None,
        sector: Optional[tuple[float, float]] = None,
        shadowing: bool = True,
    ) -> None: ...
    def remove_sensor(self, name: str) -> None: ...
    @property
    def sensor_names(self) -> list[str]: ...
    def add_camera(
        self,
        name: str,
        position: tuple[float, float, float] = (0.0, 0.0, 0.0),
        quaternion: Optional[tuple[float, float, float, float]] = None,
        look_at: Optional[tuple[float, float, float]] = None,
        fov: Optional[float] = None,
        resolution: Optional[tuple[int, int]] = None,
        near: Optional[float] = None,
        far: Optional[float] = None,
        mount: Optional[str] = None,
        robot: Optional[str] = None,
        link: Optional[str] = None,
        from_catalog: Optional[str] = None,
        revision: Optional[str] = None,
    ) -> None: ...
    def remove_camera(self, name: str) -> None: ...
    @property
    def camera_names(self) -> list[str]: ...
    def add_lidar(
        self,
        name: str,
        position: tuple[float, float, float] = (0.0, 0.0, 0.0),
        quaternion: Optional[tuple[float, float, float, float]] = None,
        yaw: Optional[float] = None,
        fov: Optional[float] = None,
        range: Optional[tuple[float, float]] = None,
        resolution: Optional[float] = None,
        mount: Optional[str] = None,
        robot: Optional[str] = None,
        link: Optional[str] = None,
        from_catalog: Optional[str] = None,
        revision: Optional[str] = None,
    ) -> None: ...
    def remove_lidar(self, name: str) -> None: ...
    @property
    def lidar_names(self) -> list[str]: ...
    def lidar_scan(self, name: str, t: Optional[float] = None) -> "ScanFrame": ...
    def scan_sweep(self, name: str, fps: float = 10.0) -> list["ScanFrame"]: ...
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
    def add_vehicle(
        self,
        name: str,
        body: list[str],
        path: list[tuple[float, float]] | list[tuple[float, float, float]],
        stations: dict[str, int],
        speed: float = 0.5,
        turn_speed: float = 1.5707963267948966,
        start: Optional[str] = None,
        ring: bool = False,
        allow_reverse: bool = False,
        max_grade: Optional[float] = None,
        drive: str = "differential",
        climb_speed: Optional[float] = None,
        descent_speed: Optional[float] = None,
        fixed_yaw: Optional[float] = None,
        tray_position: Optional[tuple[float, float, float]] = None,
        tray_size: Optional[tuple[float, float, float]] = None,
        tray_quaternion: Optional[tuple[float, float, float, float]] = None,
    ) -> None: ...
    def add_lift(
        self,
        name: str,
        car: list[str],
        zone_position: tuple[float, float, float],
        zone_size: tuple[float, float, float],
        stops: dict[str, float],
        speed: float = 0.5,
        axis: tuple[float, float, float] = (0.0, 0.0, 1.0),
        zone_quaternion: Optional[tuple[float, float, float, float]] = None,
        start: Optional[str] = None,
    ) -> None: ...
    def remove_device(self, name: str) -> None: ...
    @property
    def device_names(self) -> list[str]: ...
    def io_points(self, sequences: Optional[list[str]] = None) -> list[IoPoint]: ...
    def io_report(self, sequences: Optional[list[str]] = None) -> IoReport: ...
    def io_list(self, format: str = "csv", sequences: Optional[list[str]] = None) -> str: ...
    def export_io_list(
        self, path: Union[str, Path], sequences: Optional[list[str]] = None
    ) -> None: ...
    def add_io_node(
        self,
        name: str,
        kind: str = "plc",
        robots: Optional[list[str]] = None,
        programs: Optional[list[str]] = None,
        uplink: Union[str, tuple[str, str], None] = None,
        channels: Optional[list[dict]] = None,
        place: Optional[str] = None,
        model: Optional[str] = None,
        label: Optional[str] = None,
    ) -> None: ...
    def remove_io_node(self, name: str) -> None: ...
    def set_part(
        self,
        name: str,
        *,
        kind: Optional[str] = None,
        catalog: Union[str, tuple[str, Optional[str]], None] = None,
        manufacturer: Optional[str] = None,
        model: Optional[str] = None,
        category: Optional[str] = None,
        description: Optional[str] = None,
        qty: int = 1,
        attributes: Optional[dict[str, Union[float, str]]] = None,
        **extra: Union[float, str],
    ) -> str: ...
    def remove_part(self, name: str) -> None: ...
    def part(self, name: str) -> Optional[dict[str, Any]]: ...
    def parts(self) -> list[dict[str, Any]]: ...
    def bom(self) -> "Bom": ...
    def export_bom(self, path: Union[str, Path], format: Optional[str] = None) -> None: ...
    def plcopen(
        self,
        sequences: Optional[list[str]] = None,
        *,
        name: str = "cell",
        cycle: bool = True,
        task_interval_ms: int = 10,
    ) -> str: ...
    def export_plcopen(
        self,
        path: Union[str, Path],
        sequences: Optional[list[str]] = None,
        *,
        name: str = "cell",
        cycle: bool = True,
        task_interval_ms: int = 10,
    ) -> None: ...
    def layout(
        self,
        format: str = "svg",
        *,
        scale: float = 100.0,
        units: str = "mm",
        ground_z: float = 0.02,
        frames: bool = True,
        labels: bool = True,
        reach: bool = True,
        grid: Optional[float] = 1.0,
        title: Optional[str] = None,
    ) -> str: ...
    def export_layout(
        self,
        path: Union[str, Path],
        format: Optional[str] = None,
        *,
        scale: float = 100.0,
        units: str = "mm",
        ground_z: float = 0.02,
        frames: bool = True,
        labels: bool = True,
        reach: bool = True,
        grid: Optional[float] = 1.0,
        title: Optional[str] = None,
    ) -> None: ...
    def footprint(self, ground_z: float = 0.02) -> dict[str, Any]: ...
    def cell_report(
        self,
        timelines: Union[
            "SequenceTimeline",
            list["SequenceTimeline"],
            dict[str, "SequenceTimeline"],
            None,
        ] = None,
        *,
        scenarios: Optional["ScenarioRuns"] = None,
        deliverables: Optional[list[Union[str, Path]]] = None,
        clearance_dt: Optional[float] = 0.01,
        title: Optional[str] = None,
        ground_z: float = 0.02,
    ) -> "CellReport": ...
    def bind_input(
        self,
        name: str,
        node: str,
        channel: str,
        tag: Optional[str] = None,
        field: Optional[str] = None,
        invert: bool = False,
        contact: Optional[str] = None,
        safety: bool = False,
        voltage: Optional[float] = None,
        logic: Optional[str] = None,
        note: Optional[str] = None,
    ) -> None: ...
    def bind_output(
        self,
        name: str,
        node: str,
        channel: str,
        tag: Optional[str] = None,
        field: Optional[str] = None,
        invert: bool = False,
        contact: Optional[str] = None,
        safety: bool = False,
        voltage: Optional[float] = None,
        logic: Optional[str] = None,
        note: Optional[str] = None,
    ) -> None: ...
    def unbind_input(self, name: str, node: Optional[str] = None) -> int: ...
    def unbind_output(self, name: str, node: Optional[str] = None) -> int: ...
    def declare_io(
        self,
        name: str,
        role: Optional[str] = None,
        kind: Optional[str] = None,
        safety: bool = False,
        pair: Optional[str] = None,
        note: Optional[str] = None,
    ) -> None: ...
    def undeclare_io(self, name: str) -> None: ...
    def io_map(self) -> IoMap: ...
    def auto_assign_io(
        self, sequences: Optional[list[str]] = None, reassign: bool = False
    ) -> IoReport: ...
    def io_topology(
        self,
        format: str = "mermaid",
        sequences: Optional[list[str]] = None,
        layers: Optional[list[str]] = None,
        include_cosmetic: bool = False,
    ) -> str: ...
    def export_topology(
        self,
        path: Union[str, Path],
        sequences: Optional[list[str]] = None,
        layers: Optional[list[str]] = None,
        include_cosmetic: bool = False,
    ) -> None: ...
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
    def feed_report(self) -> Optional["FeedReport"]: ...
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
    def footfalls(
        self, robot: Optional[str] = None
    ) -> list[tuple[str, float, float, tuple[float, float, float]]]: ...
    def moves(
        self, robot: Optional[str] = None
    ) -> list[tuple[str, float, float]]: ...
    def busy_seconds(self, robot: Optional[str] = None) -> float: ...
    def utilization(self, robot: Optional[str] = None) -> float: ...
    def utilizations(self) -> dict[str, float]: ...
    def vehicle_airborne(self, name: str) -> float: ...
    def robot_busy(self, robot: Optional[str] = None) -> list[tuple[float, float]]: ...
    def handshake_spec(self, io: Optional[IoMap] = None) -> str: ...
    def export_handshake_spec(
        self, path: Union[str, Path], io: Optional[IoMap] = None
    ) -> None: ...
    def base_pose(
        self, t: float, robot: Optional[str] = None
    ) -> Optional[
        tuple[tuple[float, float, float], tuple[float, float, float, float]]
    ]: ...
    def object_pose(
        self, name: str, t: float
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]: ...
    def object_visible(self, name: str, t: float) -> bool: ...
    def carve_stock(
        self,
        stock: str,
        robot: Optional[str] = None,
        tcp_link: Optional[str] = None,
        voxel_size: float = 0.001,
        cutter_radius: float = 0.004,
        cutter_length: float = 0.03,
        dt: float = 0.01,
    ) -> "StockCarve": ...
    def spray_coat(
        self,
        target: str,
        applicator: Optional[Union[dict, str]] = None,
        robot: Optional[str] = None,
        tcp_link: Optional[str] = None,
        patch_size: float = 0.005,
        dt: float = 0.01,
        gate: Optional[str] = None,
        spec: Optional[tuple[float, float]] = None,
        max_incidence: float = ...,
        facing: Optional[tuple[float, float, float]] = None,
        facing_tolerance: float = ...,
        occlusion: bool = True,
        style: str = "auto",
        paint_color: Optional[tuple[float, float, float]] = None,
        substrate: Optional[tuple[float, float, float]] = None,
    ) -> "FilmCoat": ...
    def paint_report(
        self,
        target: str,
        robot: Optional[str] = None,
        tcp_link: Optional[str] = None,
        gate: Optional[str] = None,
        standoff: Optional[tuple[float, float]] = None,
        max_incidence: float = ...,
        max_range: Optional[float] = None,
        dt: float = 0.01,
    ) -> "PaintReport": ...
    def with_trigger_signal(
        self,
        name: str = "spraying",
        gate: Optional[str] = None,
        robot: Optional[str] = None,
        dt: float = 0.01,
    ) -> "SequenceTimeline": ...
    def process_spans(
        self, robot: Optional[str] = None
    ) -> list[tuple[float, float, Optional[str]]]: ...
    def feed_report(self, toolpath: Optional[str] = None) -> "FeedReport": ...
    def robot_trajectory(self, robot: Optional[str] = None) -> Trajectory: ...
    @property
    def trajectory(self) -> Trajectory: ...
    def export_usd(
        self,
        path: Union[str, Path],
        fps: float = 60.0,
        start: Optional[float] = None,
        end: Optional[float] = None,
    ) -> list[str]: ...
    def step_span(self, name: str) -> Span: ...
    def signal(self, name: str) -> SignalTrack: ...
    def min_clearance(self, dt: float = 0.01) -> Clearance: ...
    def diff(
        self,
        trace: Any,
        *,
        tolerance: float = 0.05,
        signals: Optional[list[str]] = None,
        align_on: Optional[str] = None,
        io: Optional["IoMap"] = None,
    ) -> Any: ...  # -> botrail.trace.TraceDiff
    @property
    def sequences(self) -> list[str]: ...
    @property
    def scenario(self) -> Optional[str]: ...
    @property
    def branches(self) -> list[tuple[str, str, int]]: ...
    def to_script(
        self,
        sequence: Optional[str] = None,
        dialect: str = "urscript",
        name: Optional[str] = None,
        inputs: Optional[dict[str, int]] = None,
        outputs: Optional[dict[str, int]] = None,
        speed_scale: float = 1.0,
        blend_radius: float = 0.0,
        tcp_speed: float = 0.25,
        tcp_accel: float = 1.2,
        move_to_start: bool = True,
        node: Optional[str] = None,
        io: Optional[IoMap] = None,
    ) -> str: ...
    def export_script(
        self,
        path: Union[str, Path],
        sequence: Optional[str] = None,
        dialect: str = "urscript",
        name: Optional[str] = None,
        inputs: Optional[dict[str, int]] = None,
        outputs: Optional[dict[str, int]] = None,
        speed_scale: float = 1.0,
        blend_radius: float = 0.0,
        tcp_speed: float = 0.25,
        tcp_accel: float = 1.2,
        move_to_start: bool = True,
        node: Optional[str] = None,
        io: Optional[IoMap] = None,
    ) -> None: ...

class ScenarioRuns:
    @property
    def names(self) -> list[str]: ...
    @property
    def errors(self) -> dict[str, str]: ...
    @property
    def durations(self) -> dict[str, float]: ...
    def __len__(self) -> int: ...
    def __contains__(self, key: str) -> bool: ...
    def __getitem__(self, key: str) -> SequenceTimeline: ...
    def items(self) -> list[tuple[str, SequenceTimeline]]: ...
    def uncovered_arms(self) -> list[tuple[str, str, int, str]]: ...
    def min_clearances(self, dt: float = 0.01) -> dict[str, Clearance]: ...
    def to_script(
        self,
        sequence: Optional[str] = None,
        dialect: str = "urscript",
        name: Optional[str] = None,
        primary: Optional[str] = None,
        inputs: Optional[dict[str, int]] = None,
        outputs: Optional[dict[str, int]] = None,
        speed_scale: float = 1.0,
        blend_radius: float = 0.0,
        tcp_speed: float = 0.25,
        tcp_accel: float = 1.2,
        move_to_start: bool = True,
        node: Optional[str] = None,
        io: Optional[IoMap] = None,
    ) -> str: ...
    def export_script(
        self,
        path: Union[str, Path],
        sequence: Optional[str] = None,
        dialect: str = "urscript",
        name: Optional[str] = None,
        primary: Optional[str] = None,
        inputs: Optional[dict[str, int]] = None,
        outputs: Optional[dict[str, int]] = None,
        speed_scale: float = 1.0,
        blend_radius: float = 0.0,
        tcp_speed: float = 0.25,
        tcp_accel: float = 1.2,
        move_to_start: bool = True,
        node: Optional[str] = None,
        io: Optional[IoMap] = None,
    ) -> None: ...

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
    def kind(self) -> str: ...
    @property
    def edges(self) -> list[tuple[float, bool]]: ...
    def value_at(self, t: float) -> bool: ...
    def rising_edges(self) -> list[float]: ...
    def falling_edges(self) -> list[float]: ...
    def high_spans(self) -> list[tuple[float, float]]: ...
    def high_total(self) -> float: ...

class ScanFrame:
    @property
    def lidar(self) -> str: ...
    @property
    def angles(self) -> list[float]: ...
    @property
    def ranges(self) -> list[float]: ...
    @property
    def hits(self) -> list[Optional[str]]: ...
    @property
    def position(self) -> tuple[float, float, float]: ...
    @property
    def quaternion(self) -> tuple[float, float, float, float]: ...
    @property
    def range(self) -> tuple[float, float]: ...
    @property
    def t(self) -> Optional[float]: ...
    def points(
        self, world: bool = True, stride: int = 1
    ) -> list[tuple[float, float, float]]: ...
    def save_ply(self, path: Union[str, Path], stride: int = 1) -> None: ...

class IoPoint:
    @property
    def name(self) -> str: ...
    @property
    def aspect(self) -> Optional[str]: ...
    @property
    def label(self) -> str: ...
    @property
    def direction(self) -> str: ...
    @property
    def kind(self) -> str: ...
    @property
    def source(self) -> str: ...
    @property
    def host(self) -> Optional[str]: ...
    @property
    def safety(self) -> bool: ...
    @property
    def writers(self) -> list[tuple[str, int, str]]: ...
    @property
    def readers(self) -> list[tuple[str, int, str]]: ...
    @property
    def status(self) -> str: ...

class IoFinding:
    @property
    def severity(self) -> str: ...
    @property
    def code(self) -> str: ...
    @property
    def message(self) -> str: ...
    @property
    def at(self) -> list[tuple[str, int, str]]: ...

class IoReport:
    @property
    def findings(self) -> list[IoFinding]: ...
    def errors(self) -> list[IoFinding]: ...
    def warnings(self) -> list[IoFinding]: ...
    def infos(self) -> list[IoFinding]: ...
    def to_json(self) -> str: ...
    @property
    def ok(self) -> bool: ...
    def __len__(self) -> int: ...

class IoMap:
    @property
    def nodes(self) -> list[str]: ...
    @property
    def bindings(self) -> list[tuple[str, str, str, str]]: ...
    @property
    def decls(self) -> list[str]: ...
    def to_json(self) -> str: ...

class CellReport:
    @property
    def title(self) -> str: ...
    @property
    def robots(self) -> list[dict[str, Any]]: ...
    @property
    def cycles(self) -> list[dict[str, Any]]: ...
    @property
    def io(self) -> Optional[dict[str, Any]]: ...
    @property
    def io_error(self) -> Optional[str]: ...
    @property
    def scenarios(self) -> list[dict[str, Any]]: ...
    @property
    def bom(self) -> dict[str, Any]: ...
    @property
    def footprint(self) -> dict[str, Any]: ...
    @property
    def deliverables(self) -> list[dict[str, Any]]: ...
    def cycle_time(self, name: Optional[str] = None) -> Optional[float]: ...
    def min_clearance(self) -> Optional[float]: ...
    def to_markdown(self) -> str: ...
    def to_json(self) -> str: ...
    def save(self, path: Union[str, Path], format: Optional[str] = None) -> None: ...

class Bom:
    @property
    def rows(self) -> list[dict[str, Any]]: ...
    def unidentified(self) -> list[dict[str, Any]]: ...
    def total(self, key: str) -> Optional[float]: ...
    def attribute_keys(self) -> list[str]: ...
    def to_csv(self) -> str: ...
    def to_markdown(self) -> str: ...
    def to_json(self) -> str: ...
    def save(self, path: Union[str, Path], format: Optional[str] = None) -> None: ...
    def __len__(self) -> int: ...

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

class StockCarve:
    @property
    def removed_volume(self) -> float: ...
    @property
    def remaining_volume(self) -> float: ...
    @property
    def initial_volume(self) -> float: ...
    @property
    def voxel_size(self) -> float: ...
    @property
    def triangle_count(self) -> int: ...
    @property
    def pose(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]: ...
    def save_stl(self, path: Union[str, Path]) -> None: ...
    def save_obj(self, path: Union[str, Path]) -> None: ...

class FilmCoat:
    @property
    def mean(self) -> float: ...
    @property
    def min(self) -> float: ...
    @property
    def max(self) -> float: ...
    @property
    def sigma(self) -> float: ...
    @property
    def in_spec_ratio(self) -> Optional[float]: ...
    @property
    def uncoated_area(self) -> float: ...
    @property
    def thin_area(self) -> float: ...
    @property
    def thick_area(self) -> float: ...
    @property
    def total_area(self) -> float: ...
    @property
    def surface_area(self) -> float: ...
    @property
    def sprayed_volume(self) -> float: ...
    @property
    def deposited_volume(self) -> float: ...
    @property
    def effective_transfer_efficiency(self) -> float: ...
    @property
    def gun_on_time(self) -> float: ...
    @property
    def too_close_time(self) -> float: ...
    def overspray(self) -> dict[str, float]: ...
    @property
    def lost_volume(self) -> float: ...
    def sprayed_by_brush(self) -> dict[str, float]: ...
    def deposited_by_brush(self) -> dict[str, float]: ...
    @property
    def patch_count(self) -> int: ...
    @property
    def patch_size(self) -> float: ...
    @property
    def thickness(self) -> list[float]: ...
    @property
    def pose(
        self,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]: ...
    def save_obj(self, path: Union[str, Path]) -> None: ...

class PaintReport:
    @property
    def ok(self) -> bool: ...
    @property
    def total_samples(self) -> int: ...
    @property
    def hits(self) -> int: ...
    @property
    def in_band_ratio(self) -> float: ...
    @property
    def on_target_ratio(self) -> float: ...
    @property
    def standoff_min(self) -> float: ...
    @property
    def standoff_max(self) -> float: ...
    @property
    def standoff_mean(self) -> float: ...
    @property
    def incidence_max(self) -> float: ...
    @property
    def issues(self) -> list[dict]: ...
    @property
    def probes(self) -> list[dict]: ...
    def spans(self, kind: str) -> list[tuple[float, float]]: ...
    def __bool__(self) -> bool: ...

class FeedReport:
    @property
    def hold_ratio(self) -> float: ...
    @property
    def commanded_cut_seconds(self) -> float: ...
    @property
    def achieved_cut_seconds(self) -> float: ...
    @property
    def slow_spans(self) -> list[dict]: ...

class ToolpathReport:
    @property
    def total_samples(self) -> int: ...
    @property
    def ok(self) -> bool: ...
    @property
    def issues(self) -> list[dict]: ...
    def __bool__(self) -> bool: ...

class StudioServer:
    @property
    def url(self) -> str: ...
    def stop(self) -> None: ...

def serve_studio(
    scene: Scene, studio_dir: str, host: str = "127.0.0.1", port: int = 0
) -> StudioServer: ...
def catalog_package(id: str, *, revision: str | None = None) -> str: ...
def project_schema() -> str: ...
def _parse_gcode_json(text: str, chord_tol: float = 1e-4) -> str: ...
def _parse_apt_json(text: str) -> str: ...
