import numpy as np
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.geometry import MeshcatVisualizer, StartMeshcat

class DrakeArmController:
    def __init__(self, urdf_path: str, end_effector_frame_name: str, use_meshcat: bool = True):
        self.builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=0.0)#time_step=0.0 -bez symulacji po czasie
        Parser(self.plant).AddModels(urdf_path)
        self.plant.Finalize()
        if use_meshcat:
            self.meshcat = StartMeshcat()
            MeshcatVisualizer.AddToBuilder(self.builder, self.scene_graph, self.meshcat)
        self.diagram = self.builder.Build()
        self.context = self.diagram.CreateDefaultContext()
        self.plant_context = self.diagram.GetMutableSubsystemContext(self.plant, self.context)
        self.ee_frame = self.plant.GetFrameByName(end_effector_frame_name)
        self.num_positions = self.plant.num_positions()#ile osi
        self.q_home = np.zeros(self.num_positions)

    def solve_ik(self, target_position, target_rotation=None):
        pass

    def update_visualization(self, q):
        q = np.asarray(q, dtype=float).reshape(-1)

        if q.shape[0] != self.num_positions:
            raise ValueError(f"Expected q of length {self.num_positions}, got {q.shape[0]}")

        self.plant.SetPositions(self.plant_context, q)
        self.diagram.ForcedPublish(self.context)

    def get_joint_angles_deg(self, q):
        q = np.asarray(q, dtype=float).reshape(-1)

        if q.shape[0] != self.num_positions:
            raise ValueError(f"Expected q of length {self.num_positions}, got {q.shape[0]}")

        return np.degrees(q)