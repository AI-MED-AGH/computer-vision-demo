import numpy as np

from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.multibody.parsing import Parser
from pydrake.geometry import MeshcatVisualizer, StartMeshcat, Box
from pydrake.math import RigidTransform


class DrakeArmController:
    def __init__(self, urdf_path: str, end_effector_frame_name: str, use_meshcat: bool = True):
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
            RigidTransform()
        )

        self._add_environment()

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
        self.q_home = np.zeros(self.num_positions)

        self.simulator = Simulator(self.diagram, self.context)
        self.simulator.set_target_realtime_rate(1.0)
        self.simulator.Initialize()

        self.reset_robot_home()
        self.diagram.ForcedPublish(self.context)

    def _add_environment(self):
        ground_shape = Box(2.5, 2.5, 0.04)
        X_WG = RigidTransform([0.0, 0.0, -0.02])

        self.plant.RegisterVisualGeometry(
            self.plant.world_body(),
            X_WG,
            ground_shape,
            "ground_visual",
            np.array([0.75, 0.75, 0.75, 1.0], dtype=float)
        )

        self.plant.RegisterCollisionGeometry(
            self.plant.world_body(),
            X_WG,
            ground_shape,
            "ground_collision",
            CoulombFriction(0.9, 0.8)
        )

    def solve_ik(self, target_position, target_rotation=None):
        raise NotImplementedError("solve_ik() jeszcze nie jest zaimplementowane.")

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
        self.plant.SetPositions(self.plant_context, self.robot_model_instance, self.q_home)
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