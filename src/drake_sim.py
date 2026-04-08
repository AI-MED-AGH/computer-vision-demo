<<<<<<< issue-6
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.math import RotationMatrix
from pydrake.solvers import Solve
=======
import numpy as np

from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.multibody.parsing import Parser
from pydrake.geometry import MeshcatVisualizer, StartMeshcat, Box
from pydrake.math import RigidTransform
>>>>>>> dev


class DrakeArmController:
    def __init__(self, urdf_path: str, end_effector_frame_name: str, use_meshcat: bool = True):
<<<<<<< issue-6
        
        pass

    def solve_ik(self, target_position, target_rotation=None):
        '''
        #1 -> Creating an IK problem for our virtual robot we pass on the robot model (plant) and its current state (context)
        #2 -> Getting the (q) whose values ​​the solver should find for us
        #3 -> identifies the zero point of the entire virtual world in the simulation and the moving center point of the robot's gripper. 
              It then imposes a stringent condition on the equation system, forcing the gripper to be positioned exactly at your target point by setting its minimum and maximum allowable positions to exactly the same value.
        #4 -> We check whether the user has provided any target rotation at all, and if so, we impose another constraint on the solver. We limit the allowable angular error (theta_bound) to zero, meaning the gripper must position itself EXACTLY at the specified angle in the world frame.
        #5 -> First, we extract an array of the robot's current joint angles from the virtual model. We then pass this array to the math program (ik.prog()), forcing the solver to start its calculations from this safe location.
        #6 -> We invoke the main optimization engine of the Drake environment, dropping in our prepared mathematical program. The engine performs the heavy calculations underneath, and the entire summary of this operation (whether it was successful and what the resulting numbers were) is packed into the result variable.
        #7 -> We check the calculator's internal success flag to ensure it has found a physically possible solution.
        '''

        #1
        ik = InverseKinematics(self.plant, self.plant_context)

        #2
        q = ik.q()

        #3
        world_frame = self.plant.world_frame()
        ee_frame = self.plant.GetFrameByName(self.end_effector_frame_name)

        ik.AddPositionConstraint(
            frameB=ee_frame,
            p_BQ=[0, 0, 0],              
            frameA=world_frame,
            p_AQ_lower=target_position, 
            p_AQ_upper=target_position  
        )

        #4 
        if target_rotation is not None:
            ik.AddOrientationConstraint(
                frameAbar=world_frame,
                R_AbarA=RotationMatrix(target_rotation), 
                frameBbar=ee_frame,
                R_BbarB=RotationMatrix(),               
                theta_bound=0.0                         
            )
        
        #5
        current_q = self.plant.GetPositions(self.plant_context)
        ik.prog().SetInitialGuess(q, current_q)

        #6
        result = Solve(ik.prog())
        
        #7
        if result.is_success(): return result.GetSolution(q)
        else: return None
=======
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
>>>>>>> dev

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