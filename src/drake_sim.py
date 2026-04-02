from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.math import RotationMatrix
from pydrake.solvers import Solve


class DrakeArmController:
    def __init__(self, urdf_path: str, end_effector_frame_name: str, use_meshcat: bool = True):
        
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

    def update_visualization(self, q):

        pass

    def get_joint_angles_deg(self, q):

        pass