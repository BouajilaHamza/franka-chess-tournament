import pybullet as p
import pybullet_data
from ompl import base as ob
from ompl import geometric as og
import numpy as np
import time
import logging 



logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
# --- CONSTANTS & CONFIGURATION ---

# Scene Definition
ROBOT_URDF = "franka_panda/panda.urdf" 
ACTIVE_JOINTS = [0, 1, 2, 3, 4, 5, 6] # The 7 revolute joints of the Panda arm
EE_LINK_INDEX = 7                     # Panda's last moving link for IK
TIMEOUT_SECONDS = 10.0

# Start State (Safe initial configuration)
START_STATE = [0.0, -0.4, 0.0, -2.0, 0.0, 1.6, 0.785]

# Goal Definition (The target EE Pose)
TARGET_EE_POSITION = [0.6, 0.3, 0.5]  # The center of the Green Box
# Orientation: Gripper pointing straight down (Euler: [0, -pi, 0] or [0, pi, 0])
TARGET_EE_ORIENTATION_QUAT = p.getQuaternionFromEuler([np.pi/2, 0, np.pi/2]) 

# Obstacle Definition (The Red Box)
OBSTACLE_POSITION = [0.3, 0.0, 0.5]
OBSTACLE_EXTENTS = [0.05, 0.5, 0.1]

# Global IDs
ROBOT_ID = None
OBSTACLE_ID = None
TARGET_ID = None


def setup_pybullet():
    """Initializes PyBullet and loads the robot, obstacle, and target."""
    global ROBOT_ID, OBSTACLE_ID, TARGET_ID
    
    # 1. Initialize Simulation
    p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=30, cameraPitch=-20, cameraTargetPosition=[0.5, 0, 0.5])
    p.loadURDF("plane.urdf")
    # 2. Load Robot
    ROBOT_ID = p.loadURDF(ROBOT_URDF, useFixedBase=True)
    
    # 3. Create Obstacle (Red Box)
    col_shape_red = p.createCollisionShape(p.GEOM_BOX, halfExtents=OBSTACLE_EXTENTS)
    vis_shape_red = p.createVisualShape(p.GEOM_BOX, halfExtents=OBSTACLE_EXTENTS, rgbaColor=[1, 0, 0, 1])
    OBSTACLE_ID = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col_shape_red, 
                                    baseVisualShapeIndex=vis_shape_red, basePosition=OBSTACLE_POSITION)
    
    # 4. Create Target (Green Box)
    # The goal is defined by the center of this box, representing the EE target pose.
    col_shape_green = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.01, 0.01])
    vis_shape_green = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.01, 0.01], rgbaColor=[0, 1, 0, 1])
    TARGET_ID = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col_shape_green, 
                                  baseVisualShapeIndex=vis_shape_green, basePosition=TARGET_EE_POSITION)
    
    print("[PyBullet] Simulation scene created: Red Box (Obstacle) at", OBSTACLE_POSITION, "and Green Box (Goal) at", TARGET_EE_POSITION)


# --- OMPL BRIDGES (State Space and Validity Checker) ---

class PyBulletStateValidityChecker(ob.StateValidityChecker):
    """
    Position Validity: Checks if a sampled state (joint configuration) collides with the scene.
    """
    def __init__(self, si, robot_id, active_joints):
        super(PyBulletStateValidityChecker, self).__init__(si)
        self.robot_id = robot_id
        self.active_joints = active_joints

    def isValid(self, state):
        """Returns True if the robot in this state is collision-free."""
        
        # 1. Convert and set the joint angles in PyBullet (teleport)
        joint_angles = [state[i] for i in range(len(self.active_joints))]
        for i, angle in zip(self.active_joints, joint_angles):
            p.resetJointState(self.robot_id, i, angle)

        # 2. Check for collisions (Robot <-> World)
        # Note: We rely on the implicit collision check between the robot and OBSTACLE_ID
        contacts = p.getContactPoints(bodyA=self.robot_id)
        
        # Filter contacts to include only actual collisions (ignoring ground or other non-planning bodies if necessary)
        # For simplicity, we just check if any contact exists with the Red Box.
        for contact in contacts:
            if contact[2] == OBSTACLE_ID: # bodyB is the Red Box
                # print("  [Check] State is INVALID: Collision with Red Box.")
                return False
            
        return True # No illegal collisions detected.


def set_state_space(robot_id, active_joints):
    """Defines the configuration space (C-space) for OMPL."""
    num_joints = len(active_joints)
    space = ob.RealVectorStateSpace(num_joints)
    bounds = ob.RealVectorBounds(num_joints)
    
    for i, joint_idx in enumerate(active_joints):
        joint_info = p.getJointInfo(robot_id, joint_idx)
        lower_limit, upper_limit = joint_info[8], joint_info[9]
        bounds.setLow(i, lower_limit)
        bounds.setHigh(i, upper_limit)
        
    space.setBounds(bounds)
    return space




class IKGoal(ob.GoalSampleableRegion): # <--- CHANGE THIS BASE CLASS
    """
    Goal Region: Defines the goal as the set of joint configurations 
    that achieve the target End-Effector pose AND are collision-free.
    """
    def __init__(self, si, ee_pos, ee_orn, ee_link, robot_id, active_joints):
        # Initialize the base class
        super(IKGoal, self).__init__(si)
        # ... rest of __init__ remains the same ...
        self.si = si
        self.ee_pos = ee_pos
        self.ee_orn = ee_orn
        self.ee_link = ee_link
        self.robot_id = robot_id
        self.active_joints = active_joints

    def sampleGoal(self, state):
        # ... (Your existing, correct IK sampling logic) ...
        # NOTE: For IK, it's generally best to return True only if a VALID state is found
        
        # 1. Solve IK in PyBullet
        ik_solution_full = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.ee_link,
            targetPosition=self.ee_pos,
            targetOrientation=self.ee_orn,
            maxNumIterations=1000,
            residualThreshold=1e-7
        )
        logger.debug(f"IK Solution: {ik_solution_full}")
        if not ik_solution_full:
             return False # IK failed
        
        # 2. Transfer the active joint values to the OMPL state
        ik_solution_active = [ik_solution_full[i] for i in self.active_joints]
        for i, val in enumerate(ik_solution_active):
            state[i] = val

        # 3. Check for Position Validity (Crucial step!)
        return self.si.getStateValidityChecker().isValid(state)

    def maxSampleCount(self):
        """
        RRTConnect requires this method from GoalSampleableRegion.
        Since the IK solutions form a continuous set, return a large number.
        """
        return 10000000 
    
    # RRTConnect does not strictly require distanceGoal() for sampling, 
    # but for completeness in a GoalRegion, it is usually necessary.
    # We omit it here as the sampleable version is often sufficient for RRTConnect.

    def isSatisfied(self, state):
        # Check if the state is within the goal region threshold (required by base Goal class)
        # This implementation requires calculating distance, which is complex for IK.
        # For simplicity and RRTConnect compatibility, we often rely on sampling:
        return False # The planner relies on sampleGoal() and tree connection instead.
# --- MAIN EXECUTION ---

def run_ompl_planning_ik():
    setup_pybullet()
    
    # 1. Define the State Space
    space = set_state_space(ROBOT_ID, ACTIVE_JOINTS)
    si = ob.SpaceInformation(space)
    
    # 2. Give OMPL the validity checker (Bridge 1)
    validity_checker = PyBulletStateValidityChecker(si, ROBOT_ID, ACTIVE_JOINTS)
    si.setStateValidityChecker(validity_checker)
    
    si.setStateValidityCheckingResolution(0.005) # Check every 0.5% of the total range

    # **FIX:** Set a fine resolution for motion checking.
    # This tells OMPL that no two states in a motion segment should be farther apart 
    # than this ratio of the state space extent. A small value like 0.01 (1%) 
    # or 0.005 (0.5%) often ensures safety in complex scenes.
    # You must also explicitly set the Motion Validator for OMPL to use the resolution effectively.
    motion_validator = ob.DiscreteMotionValidator(si)
    si.setMotionValidator(motion_validator)
    si.setup() # Initialize the Space Information
    
    # 3. Define the Problem Definition
    pdef = ob.ProblemDefinition(si)
    
    # Set the Start State
    start_ompl = ob.State(space)
    for i, val in enumerate(START_STATE):
        start_ompl[i] = val
    pdef.addStartState(start_ompl)
    
    # Set the Goal Region using the IKGoal class (Bridge 2)
    ik_goal = IKGoal(si, TARGET_EE_POSITION, TARGET_EE_ORIENTATION_QUAT, 
                     EE_LINK_INDEX, ROBOT_ID, ACTIVE_JOINTS)
    pdef.setGoal(ik_goal)
    
    
    # 4. Choose and Setup the Planner (RRTConnect)
    planner = og.RRTConnect(si)
    planner.setProblemDefinition(pdef)
    planner.setup()
    
    # 5. Search for a path
    print(f"\n[OMPL] 🧩 Searching for a path to the Green Box (max {TIMEOUT_SECONDS}s)...")
    solved = planner.solve(TIMEOUT_SECONDS)

    if solved:
        print("[OMPL] 🎉 Safe path found! Planning complete.")
        path = pdef.getSolutionPath()
        path.interpolate(100) # Smooth and discretize the path into 50 steps
        logger.info(f"Path Length: {path.getStateCount()}")
        # 6. Execute the route
        print("[PyBullet] 🤖 Executing safe trajectory...")
        for state_index in range(path.getStateCount()):
            state = path.getState(state_index)
            joint_angles = [state[j] for j in range(space.getDimension())]
            
            # Use PyBullet's motor control for smooth, non-instantaneous motion
            p.setJointMotorControlArray(
                bodyUniqueId=ROBOT_ID,
                jointIndices=ACTIVE_JOINTS,
                controlMode=p.POSITION_CONTROL,
                targetPositions=joint_angles,
                forces=[500] * len(ACTIVE_JOINTS) # Apply force to move motors
            )
            time.sleep(0.05) # Control speed
        
        print("[PyBullet] ✅ Execution complete.")
    else:
        print("[OMPL] ❌ Could not find a path. The IK pose might be unreachable or highly constrained by the Red Box.")

    # Keep GUI open for a moment
    time.sleep(100)
    p.disconnect()

if __name__ == '__main__':
    run_ompl_planning_ik()