import numpy as np

from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.solvers import Solve
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.multibody.parsing import Parser
from pydrake.geometry import MeshcatVisualizer, StartMeshcat, Box, Mesh


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

        self._add_environment()
        self.plant.mutable_gravity_field().set_gravity_vector([0.0, 0.0, 0.0])
        self.plant.Finalize()

        self.meshcat = None
        if use_meshcat:
            self.meshcat = StartMeshcat()
            MeshcatVisualizer.AddToBuilder(self.builder, self.scene_graph, self.meshcat)

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

        self.gripper_open_angle = 0.05
        self.gripper_closed_angle = 0.65
        self.q_home = np.zeros(self.num_positions)
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

        self.simulator = Simulator(self.diagram, self.context)
        self.simulator.set_target_realtime_rate(1.0)
        self.simulator.Initialize()

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

        q[self.gripper_index] = self.gripper_closed_angle if closed else self.gripper_open_angle
        return q

    def hand_state_to_gripper_closed(self, hand_state: int, hand_confidence: int = None) -> bool:
        # 2=open, 3=closed
        # confidence: 0=low, 1=high
        if hand_confidence is not None and hand_confidence == 0:
            return False

        if hand_state == 3:
            return True
        if hand_state == 2:
            return False

        return False

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

    def get_end_effector_pose(self):
        return self.ee_frame.CalcPoseInWorld(self.plant_context)

    def print_end_effector_position(self):
        X_WE = self.get_end_effector_pose()
        print("EE xyz:", X_WE.translation())