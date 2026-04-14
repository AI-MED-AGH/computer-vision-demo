import time

import numpy as np

from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.solvers import Solve
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.multibody.parsing import Parser
from pydrake.geometry import MeshcatVisualizer, StartMeshcat, Box, Mesh, Rgba


class DrakeArmController:
    def __init__(self, urdf_path: str, end_effector_frame_name: str, use_meshcat: bool = True):
        self.end_effector_frame_name = end_effector_frame_name

        self.builder = DiagramBuilder()

        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            self.builder,
            time_step=0.001
        )

        parser = Parser(self.plant)
        model_instances = parser.AddModels(urdf_path)

        if len(model_instances) != 1:
            raise RuntimeError(f"Expected exactly 1 model from URDF, got {len(model_instances)}")

        self.robot_model_instance = model_instances[0]

        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName("base_link", self.robot_model_instance),
            RigidTransform(RotationMatrix.MakeZRotation(np.pi / 9), [0, 0, 0])
        )

        self.box_size = np.array([0.08, 0.08, 0.16], dtype=float)
        self.box_floor_z = self.box_size[2] / 2.0
        self.grasp_distance = 0.16
        self.box_grasp_offset = np.array([0.0, 0.0, -self.box_size[2] * 0.35])
        self.box_gravity = np.array([0.0, 0.0, -9.81])
        self.box_restitution = 0.45
        self.box_floor_friction = 0.82
        self.box_sleep_speed = 0.025
        self.box_release_limits = np.array([
            [0.12, 0.70],
            [-0.35, 0.35],
        ], dtype=float)
        self.box_initial_positions = [
            np.array([0.48, 0.04, self.box_floor_z], dtype=float),
            np.array([0.48, 0.16, self.box_floor_z], dtype=float),
        ]
        self.box_paths = [
            '/pickable_boxes/box_1',
            '/pickable_boxes/box_2',
        ]
        self.box_colors = [
            Rgba(0.90, 0.15, 0.12, 1.0),
            Rgba(0.10, 0.35, 0.95, 1.0),
        ]
        self.box_poses = [RigidTransform(position) for position in self.box_initial_positions]
        self.box_velocities = [np.zeros(3) for _ in self.box_initial_positions]
        self.box_last_positions = [position.copy() for position in self.box_initial_positions]
        self.carried_box_index = None
        self.last_box_update_time = None

        self._add_environment()
        self.plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, 0.0])
        self.plant.Finalize()

        self.meshcat = None
        if use_meshcat:
            self.meshcat = StartMeshcat()
            MeshcatVisualizer.AddToBuilder(self.builder, self.scene_graph, self.meshcat)
            self._add_pickable_boxes()

        self.diagram = self.builder.Build()
        self.context = self.diagram.CreateDefaultContext()
        self.plant_context = self.diagram.GetMutableSubsystemContext(self.plant, self.context)

        self.ee_frame = self.plant.GetFrameByName(end_effector_frame_name, self.robot_model_instance)
        self.num_positions = self.plant.num_positions(self.robot_model_instance)
        self.gripper_joint_name = "Gripper_Servo_Gear_Joint"
        self.gripper_joint = self.plant.GetJointByName(
            self.gripper_joint_name,
            self.robot_model_instance
        )
        self.gripper_index = self.gripper_joint.position_start()
        self.gripper_mimic_joint_indices = []
        for joint_name, multiplier in [
            ("Gripper_Idol_Gear_Joint", -1.0),
            ("Pivot_Arm_Gripper_Servo_Joint", 1.0),
            ("Tip_Gripper_Servo_Joint", 1.0),
            ("Pivot_Arm_Gripper_Idol_Joint", -1.0),
            ("Tip_Gripper_Idol_Joint", 1.0),
        ]:
            joint = self.plant.GetJointByName(joint_name, self.robot_model_instance)
            self.gripper_mimic_joint_indices.append((joint.position_start(), multiplier))

        self.gripper_open_angle = 0.05
        self.gripper_closed_angle = 0.85
        self.gripper_current_angle = self.gripper_open_angle
        self.gripper_max_step = 0.035
        self.q_nominal = np.zeros(self.num_positions)

        # Delikatnie lepsza pozycja startowa dla ramienia.
        if self.num_positions > 1:
            self.q_nominal[1] = -0.35
        if self.num_positions > 2:
            self.q_nominal[2] = 0.75
        if self.num_positions > 3:
            self.q_nominal[3] = -0.55
        if self.num_positions > 4:
            self.q_nominal[4] = 0.10
        self.q_home = self.q_nominal.copy()

        self.simulator = Simulator(self.diagram, self.context)
        self.simulator.set_target_realtime_rate(1.0)
        self.simulator.Initialize()

        self.reset_boxes()
        self.reset_robot_home()
        self.diagram.ForcedPublish(self.context)

        try:
            X_WE = self.get_end_effector_pose()
            print("EE home position:", np.round(X_WE.translation(), 3))
        except Exception as e:
            print("Could not print EE home pose:", e)

    def set_gripper_closed(self, q, closed: bool):
        q = np.asarray(q, dtype=float).copy().reshape(-1)

        if q.shape[0] != self.num_positions:
            raise ValueError(f"Expected q of length {self.num_positions}, got {q.shape[0]}")

        target_angle = self.gripper_closed_angle if closed else self.gripper_open_angle
        delta = np.clip(
            target_angle - self.gripper_current_angle,
            -self.gripper_max_step,
            self.gripper_max_step
        )
        self.gripper_current_angle += delta

        q[self.gripper_index] = self.gripper_current_angle
        for joint_index, multiplier in self.gripper_mimic_joint_indices:
            q[joint_index] = multiplier * self.gripper_current_angle
        return q

    def hand_state_to_gripper_closed(self, hand_state: int, hand_confidence: int = None) -> bool:
        # 2=open, 3=closed
        # confidence: 0=low, 1=high; low-confidence states are still useful
        # for the gripper, so do not reject them outright.
        if hand_state == 3:
            return True
        if hand_state == 2:
            return False

        return False

    def _add_pickable_boxes(self):
        shape = Box(*self.box_size)

        for i, path in enumerate(self.box_paths):
            self.meshcat.SetObject(path, shape, self.box_colors[i])
            self.meshcat.SetTransform(path, self.box_poses[i])

    def _set_box_pose(self, index, position):
        position = np.asarray(position, dtype=float).reshape(3)
        pose = RigidTransform(position)
        self.box_poses[index] = pose
        self.box_last_positions[index] = position.copy()

        if self.meshcat is not None:
            self.meshcat.SetTransform(self.box_paths[index], pose)

    def _update_box_velocity_from_pose(self, index, position, dt):
        if dt <= 1e-6:
            return

        previous_position = self.box_last_positions[index]
        self.box_velocities[index] = (position - previous_position) / dt
        self.box_last_positions[index] = position.copy()

    def _advance_box_physics(self, dt):
        if dt <= 1e-6:
            return

        dt = min(dt, 0.05)
        for i, pose in enumerate(self.box_poses):
            if i == self.carried_box_index:
                continue

            position = pose.translation().copy()
            velocity = self.box_velocities[i] + self.box_gravity * dt
            position = position + velocity * dt

            if position[2] <= self.box_floor_z:
                position[2] = self.box_floor_z
                if velocity[2] < 0.0:
                    velocity[2] = -velocity[2] * self.box_restitution
                    velocity[0] *= self.box_floor_friction
                    velocity[1] *= self.box_floor_friction

                if np.linalg.norm(velocity) < self.box_sleep_speed:
                    velocity[:] = 0.0

            self.box_velocities[i] = velocity
            self._set_box_pose(i, position)

    def _update_pickable_boxes(self, gripper_closed):
        now = time.monotonic()
        if self.last_box_update_time is None:
            dt = 0.0
        else:
            dt = now - self.last_box_update_time
        self.last_box_update_time = now

        X_WE = self.get_end_effector_pose()
        ee_position = X_WE.translation()

        if self.carried_box_index is None and gripper_closed:
            distances = [
                np.linalg.norm(ee_position - (pose.translation() - self.box_grasp_offset))
                for pose in self.box_poses
            ]
            nearest_index = int(np.argmin(distances))
            if distances[nearest_index] <= self.grasp_distance:
                self.carried_box_index = nearest_index
                self.box_velocities[nearest_index] = np.zeros(3)
                self.box_last_positions[nearest_index] = self.box_poses[nearest_index].translation().copy()

        if self.carried_box_index is not None:
            box_index = self.carried_box_index
            if gripper_closed:
                carried_position = ee_position + self.box_grasp_offset
                self._update_box_velocity_from_pose(box_index, carried_position, dt)
                self._set_box_pose(box_index, carried_position)
            else:
                self.carried_box_index = None

        self._advance_box_physics(dt)

    def solve_ik(self, target_position, target_rotation=None):
        target_position = np.asarray(target_position, dtype=float).reshape(3)

        ik = InverseKinematics(self.plant, self.plant_context)
        q = ik.q()

        world_frame = self.plant.world_frame()
        ee_frame = self.ee_frame

        eps = 0.03
        ik.AddPositionConstraint(
            frameB=ee_frame,
            p_BQ=[0, 0, 0],
            frameA=world_frame,
            p_AQ_lower=target_position - eps,
            p_AQ_upper=target_position + eps
        )

        if target_rotation is not None:
            if isinstance(target_rotation, RotationMatrix):
                R_target = target_rotation
            else:
                R_target = RotationMatrix(target_rotation)

            ik.AddOrientationConstraint(
                frameAbar=world_frame,
                R_AbarA=R_target,
                frameBbar=ee_frame,
                R_BbarB=RotationMatrix(),
                theta_bound=0.15
            )

        current_q = self.plant.GetPositions(self.plant_context, self.robot_model_instance)

        # Koszt: nie odchodź za daleko od sensownej pozycji nominalnej.
        ik.prog().AddQuadraticErrorCost(
            np.eye(self.num_positions) * 1.0,
            self.q_nominal,
            q
        )

        # Koszt: nie skacz zbyt mocno względem aktualnej pozycji.
        ik.prog().AddQuadraticErrorCost(
            np.eye(self.num_positions) * 0.25,
            current_q,
            q
        )

        guesses = [
            current_q,
            self.q_nominal,
            0.5 * current_q + 0.5 * self.q_nominal,
            self.q_home,
        ]

        for guess in guesses:
            ik.prog().SetInitialGuess(q, guess)
            result = Solve(ik.prog())
            if result.is_success():
                return result.GetSolution(q)

        return None

    def _add_environment(self):
        room_x = 2.5
        room_y = 2.5
        room_h = 2.0
        wall_t = 0.04

        floor_z = -wall_t / 2.0
        ceiling_z = room_h + wall_t / 2.0
        wall_z = room_h / 2.0

        tile_h = 0.02
        tile_size = 0.25
        nx = int(room_x / tile_size)
        ny = int(room_y / tile_size)

        color_a = np.array([1.0, 1.0, 1.0, 1.0], dtype=float)
        color_b = np.array([0.90, 0.84, 0.25, 1.0], dtype=float)

        start_x = -room_x / 2.0 + tile_size / 2.0
        start_y = -room_y / 2.0 + tile_size / 2.0

        for ix in range(nx):
            for iy in range(ny):
                x = start_x + ix * tile_size
                y = start_y + iy * tile_size
                color = color_a if (ix + iy) % 2 == 0 else color_b

                self.plant.RegisterVisualGeometry(
                    self.plant.world_body(),
                    RigidTransform([x, y, floor_z + tile_h / 2.0]),
                    Box(tile_size, tile_size, tile_h),
                    f"floor_tile_{ix}_{iy}",
                    color
                )

        floor_collision_shape = Box(room_x, room_y, wall_t)
        self.plant.RegisterCollisionGeometry(
            self.plant.world_body(),
            RigidTransform([0.0, 0.0, floor_z]),
            floor_collision_shape,
            "floor_collision",
            CoulombFriction(0.9, 0.8)
        )

        ceiling_color = np.array([0.92, 0.92, 0.9, 1.0], dtype=float)
        ceiling_shape = Box(room_x, room_y, wall_t)
        self.plant.RegisterVisualGeometry(
            self.plant.world_body(),
            RigidTransform([0.0, 0.0, ceiling_z]),
            ceiling_shape,
            "ceiling_visual",
            ceiling_color
        )

        wood_colors = [
            np.array([1.0, 1.0, 1.0, 1.0], dtype=float),
            np.array([0.98, 0.98, 0.98, 1.0], dtype=float),
            np.array([0.96, 0.96, 0.96, 1.0], dtype=float),
            np.array([0.99, 0.99, 0.99, 1.0], dtype=float),
        ]

        panel_w_y = 0.22
        n_left = int(room_y / panel_w_y) + 1
        start_y_left = -room_y / 2.0 + panel_w_y / 2.0

        for i in range(n_left):
            y = start_y_left + i * panel_w_y
            self.plant.RegisterVisualGeometry(
                self.plant.world_body(),
                RigidTransform([-(room_x / 2.0) - wall_t / 2.0, y, wall_z]),
                Box(wall_t, panel_w_y, room_h),
                f"wall_left_panel_{i}",
                wood_colors[i % len(wood_colors)]
            )

        n_right = int(room_y / panel_w_y) + 1
        start_y_right = -room_y / 2.0 + panel_w_y / 2.0

        for i in range(n_right):
            y = start_y_right + i * panel_w_y
            self.plant.RegisterVisualGeometry(
                self.plant.world_body(),
                RigidTransform([(room_x / 2.0) + wall_t / 2.0, y, wall_z]),
                Box(wall_t, panel_w_y, room_h),
                f"wall_right_panel_{i}",
                wood_colors[i % len(wood_colors)]
            )

        panel_w_x = 0.22
        n_back = int(room_x / panel_w_x) + 1
        start_x_back = -room_x / 2.0 + panel_w_x / 2.0

        for i in range(n_back):
            x = start_x_back + i * panel_w_x
            self.plant.RegisterVisualGeometry(
                self.plant.world_body(),
                RigidTransform([x, -(room_y / 2.0) - wall_t / 2.0, wall_z]),
                Box(panel_w_x, wall_t, room_h),
                f"wall_back_panel_{i}",
                wood_colors[i % len(wood_colors)]
            )

        n_front = int(room_x / panel_w_x) + 1
        start_x_front = -room_x / 2.0 + panel_w_x / 2.0

        for i in range(n_front):
            x = start_x_front + i * panel_w_x
            self.plant.RegisterVisualGeometry(
                self.plant.world_body(),
                RigidTransform([x, (room_y / 2.0) + wall_t / 2.0, wall_z]),
                Box(panel_w_x, wall_t, room_h),
                f"wall_front_panel_{i}",
                wood_colors[i % len(wood_colors)]
            )
        # ===== OKNA WEWNĘTRZNE =====

        window_w = 0.75
        window_h = 0.55
        frame_t = 0.04
        glass_t = 0.01

        z_center = 0.95  # niżej niż wcześniej
        y_wall_inner = (room_y / 2.0) - wall_t / 2.0 - 0.01  # od wewnątrz ściany

        frame_color = np.array([1.0, 1.0, 1.0, 1.0], dtype=float)
        glass_color = np.array([0.35, 0.65, 1.0, 1.0], dtype=float)

        # dwa okna na jednej ścianie
        for i, x in enumerate([-0.45, 0.45]):
            # ===== ZEWNĘTRZNA RAMA =====

            # góra
            self.plant.RegisterVisualGeometry(
                self.plant.world_body(),
                RigidTransform([x, y_wall_inner, z_center + window_h / 2]),
                Box(window_w, frame_t, frame_t),
                f"window_top_{i}",
                frame_color
            )

            # dół
            self.plant.RegisterVisualGeometry(
                self.plant.world_body(),
                RigidTransform([x, y_wall_inner, z_center - window_h / 2]),
                Box(window_w, frame_t, frame_t),
                f"window_bottom_{i}",
                frame_color
            )

            # lewa
            self.plant.RegisterVisualGeometry(
                self.plant.world_body(),
                RigidTransform([x - window_w / 2, y_wall_inner, z_center]),
                Box(frame_t, frame_t, window_h),
                f"window_left_{i}",
                frame_color
            )

            # prawa
            self.plant.RegisterVisualGeometry(
                self.plant.world_body(),
                RigidTransform([x + window_w / 2, y_wall_inner, z_center]),
                Box(frame_t, frame_t, window_h),
                f"window_right_{i}",
                frame_color
            )

            # ===== SZYBA / TŁO =====
            self.plant.RegisterVisualGeometry(
                self.plant.world_body(),
                RigidTransform([x, y_wall_inner + 0.002, z_center]),
                Box(window_w - frame_t, glass_t, window_h - frame_t),
                f"window_glass_{i}",
                glass_color
            )

            # ===== KRZYŻ NA ŚRODKU (PLUS) =====

            # pionowy podział
            self.plant.RegisterVisualGeometry(
                self.plant.world_body(),
                RigidTransform([x, y_wall_inner + 0.004, z_center]),
                Box(frame_t, glass_t, window_h - frame_t),
                f"window_cross_vertical_{i}",
                frame_color
            )

            # poziomy podział
            self.plant.RegisterVisualGeometry(
                self.plant.world_body(),
                RigidTransform([x, y_wall_inner + 0.004, z_center]),
                Box(window_w - frame_t, glass_t, frame_t),
                f"window_cross_horizontal_{i}",
                frame_color
            )
        decor_color = np.array([1.0, 1.0, 1.0, 1.0], dtype=float)

        def add_box(name, xyz, size, color):
            self.plant.RegisterVisualGeometry(
                self.plant.world_body(),
                RigidTransform(xyz),
                Box(*size),
                name,
                np.array(color, dtype=float)
            )

        def add_cylinder(name, xyz, radius, length, color):
            add_box(name, xyz, [radius * 2.0, radius * 2.0, length], color)

        def add_sphere(name, xyz, radius, color):
            add_box(name, xyz, [radius * 1.6, radius * 1.6, radius * 1.6], color)

        rug_color = [0.18, 0.48, 0.62, 1.0]
        rug_edge_color = [0.08, 0.18, 0.22, 1.0]
        wood_color = [0.50, 0.30, 0.16, 1.0]
        dark_wood_color = [0.24, 0.15, 0.09, 1.0]
        plant_green = [0.12, 0.55, 0.22, 1.0]
        plant_light_green = [0.32, 0.72, 0.28, 1.0]
        clay_color = [0.74, 0.22, 0.14, 1.0]
        metal_color = [0.70, 0.72, 0.74, 1.0]
        warm_light = [1.0, 0.88, 0.42, 1.0]

        add_box("decor_rug", [0.38, -0.20, 0.006], [1.15, 0.82, 0.012], rug_color)
        add_box("decor_rug_front_edge", [0.38, 0.215, 0.018], [1.15, 0.025, 0.014], rug_edge_color)
        add_box("decor_rug_back_edge", [0.38, -0.615, 0.018], [1.15, 0.025, 0.014], rug_edge_color)

        table_z = 0.43
        add_box("decor_workbench_top", [0.70, -0.95, table_z], [0.78, 0.28, 0.045], wood_color)
        for i, (x, y) in enumerate([(0.36, -1.06), (1.04, -1.06), (0.36, -0.84), (1.04, -0.84)]):
            add_box(f"decor_workbench_leg_{i}", [x, y, table_z / 2.0], [0.045, 0.045, table_z], dark_wood_color)
        add_box("decor_workbench_drawer", [0.70, -0.805, 0.34], [0.48, 0.035, 0.12], [0.38, 0.22, 0.12, 1.0])
        add_box("decor_laptop_base", [0.61, -0.95, 0.475], [0.24, 0.16, 0.018], [0.08, 0.09, 0.10, 1.0])
        add_box("decor_laptop_screen", [0.61, -1.03, 0.56], [0.24, 0.018, 0.16], [0.02, 0.04, 0.05, 1.0])

        shelf_y = -1.245
        for i, z in enumerate([0.72, 1.02, 1.32]):
            add_box(f"decor_back_shelf_{i}", [-0.72, shelf_y, z], [0.76, 0.045, 0.045], wood_color)
            add_box(f"decor_back_shelf_left_bracket_{i}", [-1.08, shelf_y + 0.02, z - 0.09], [0.04, 0.035, 0.18], dark_wood_color)
            add_box(f"decor_back_shelf_right_bracket_{i}", [-0.36, shelf_y + 0.02, z - 0.09], [0.04, 0.035, 0.18], dark_wood_color)
        book_colors = [
            [0.90, 0.12, 0.10, 1.0],
            [0.10, 0.35, 0.85, 1.0],
            [0.95, 0.72, 0.10, 1.0],
            [0.18, 0.62, 0.35, 1.0],
        ]
        for i, color in enumerate(book_colors):
            add_box(f"decor_book_{i}", [-0.98 + i * 0.065, shelf_y + 0.03, 0.81], [0.045, 0.055, 0.16], color)
        add_box("decor_storage_box", [-0.56, shelf_y + 0.03, 1.105], [0.22, 0.07, 0.12], [0.82, 0.82, 0.78, 1.0])

        add_cylinder("decor_plant_pot", [-0.95, 0.82, 0.105], 0.105, 0.21, clay_color)
        add_cylinder("decor_plant_stem", [-0.95, 0.82, 0.31], 0.018, 0.28, [0.18, 0.34, 0.10, 1.0])
        for i, (dx, dy, dz, radius, color) in enumerate([
            (0.00, 0.00, 0.49, 0.13, plant_green),
            (0.09, 0.02, 0.43, 0.09, plant_light_green),
            (-0.08, -0.03, 0.42, 0.085, plant_green),
            (0.03, -0.08, 0.38, 0.075, plant_light_green),
        ]):
            add_sphere(f"decor_plant_leaf_{i}", [-0.95 + dx, 0.82 + dy, dz], radius, color)

        add_cylinder("decor_floor_lamp_stand", [1.05, 0.92, 0.54], 0.018, 1.08, metal_color)
        add_cylinder("decor_floor_lamp_base", [1.05, 0.92, 0.035], 0.13, 0.035, metal_color)
        add_cylinder("decor_floor_lamp_shade", [1.05, 0.92, 1.12], 0.14, 0.16, warm_light)
        add_sphere("decor_lamp_glow", [1.05, 0.92, 1.08], 0.08, [1.0, 0.94, 0.55, 0.55])

        for i, (x, z, color) in enumerate([
            (-0.78, 1.50, [0.86, 0.18, 0.20, 1.0]),
            (-0.35, 1.46, [0.12, 0.46, 0.74, 1.0]),
            (0.08, 1.52, [0.24, 0.62, 0.35, 1.0]),
        ]):
            add_box(f"decor_wall_art_frame_{i}", [x, shelf_y + 0.002, z], [0.28, 0.018, 0.22], dark_wood_color)
            add_box(f"decor_wall_art_canvas_{i}", [x, shelf_y + 0.014, z], [0.22, 0.012, 0.16], color)

        add_box("decor_side_cabinet", [-1.08, -0.52, 0.24], [0.26, 0.42, 0.48], [0.40, 0.25, 0.14, 1.0])
        add_box("decor_side_cabinet_top", [-1.08, -0.52, 0.50], [0.30, 0.46, 0.04], wood_color)
        add_box("decor_side_cabinet_handle", [-0.945, -0.52, 0.30], [0.018, 0.16, 0.026], metal_color)


    def update_visualization(self, q):
        q = np.asarray(q, dtype=float).reshape(-1)

        if q.shape[0] != self.num_positions:
            raise ValueError(f"Expected q of length {self.num_positions}, got {q.shape[0]}")

        self.plant.SetPositions(self.plant_context, self.robot_model_instance, q)
        self.plant.SetVelocities(
            self.plant_context,
            self.robot_model_instance,
            np.zeros(self.plant.num_velocities(self.robot_model_instance))
        )
        gripper_closed = q[self.gripper_index] >= (
            self.gripper_open_angle + self.gripper_closed_angle
        ) / 2.0
        self._update_pickable_boxes(gripper_closed)
        self.diagram.ForcedPublish(self.context)

    def get_joint_angles_deg(self, q):
        q = np.asarray(q, dtype=float).reshape(-1)

        if q.shape[0] != self.num_positions:
            raise ValueError(f"Expected q of length {self.num_positions}, got {q.shape[0]}")

        return np.degrees(q)

    def step_simulation(self, dt=0.01):
        current_time = self.context.get_time()
        self.simulator.AdvanceTo(current_time + dt)

    def reset_robot_home(self):
        self.plant.SetPositions(self.plant_context, self.robot_model_instance, self.q_nominal)
        self.plant.SetVelocities(
            self.plant_context,
            self.robot_model_instance,
            np.zeros(self.plant.num_velocities(self.robot_model_instance))
        )

    def reset_boxes(self):
        self.carried_box_index = None
        self.last_box_update_time = None
        for i, position in enumerate(self.box_initial_positions):
            self.box_velocities[i] = np.zeros(3)
            self.box_last_positions[i] = position.copy()
            self._set_box_pose(i, position)

    def get_end_effector_pose(self):
        return self.ee_frame.CalcPoseInWorld(self.plant_context)

    def print_end_effector_position(self):
        X_WE = self.get_end_effector_pose()
        print("EE xyz:", X_WE.translation())