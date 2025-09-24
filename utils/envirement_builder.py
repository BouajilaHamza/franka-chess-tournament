import logging
import pybullet as p
import pybullet_data
from configs.config import config 
from utils.simulation import wait



logger = logging.getLogger(__name__)

def setup_simulation():
    """Initialize PyBullet, load environment, and configure settings."""
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(config.simulation.step_delay)
    p.setRealTimeSimulation(0)
    logger.info("Simulation initialized.")
    return physics_client

def load_plane():
    """Load the ground plane."""
    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(plane_id, -1, lateralFriction=1.0)
    logger.debug("Ground plane loaded.")
    return plane_id

def load_robot(position, orientation):
    """Load the robot and configure its joints for better grasp stability."""
    robot_id = p.loadURDF(
        
        config.robot.first.urdf,
        basePosition=position,
        baseOrientation=orientation,
        useFixedBase=True
    )

    # Identify arm revolute joints
    arm_joint_indices = [
        i for i in range(p.getNumJoints(robot_id))
        if p.getJointInfo(robot_id, i)[2] == p.JOINT_REVOLUTE
    ][:config.robot.first.num_arm_joints]

    # Set damping on those arm joints (reduce jitter)
    for joint_idx in arm_joint_indices:
        p.changeDynamics(robot_id,
                         linkIndex=joint_idx,
                         linearDamping=0.0,
                         angularDamping=config.simulation.default_joint_damping)

    # Identify gripper finger joints (assuming last 2 revolute joints are fingers)
    gripper_indices = [
        i for i in range(p.getNumJoints(robot_id))
        if p.getJointInfo(robot_id, i)[2] == p.JOINT_REVOLUTE
    ][-2:]



    # These dynamics settings help with grasp friction
    for finger_link in gripper_indices:
        p.changeDynamics(robot_id,
                         linkIndex=finger_link,
                         lateralFriction=1.5,
                         spinningFriction=0.1,
                         rollingFriction=0.1,
                         restitution=0.0,
                         contactStiffness=30000.0,
                         contactDamping=1000.0)
    
    # Optional: also set friction on the pawn / object you want to grip
    # assuming you load it elsewhere and have its bodyId and link index (or base link = -1)
    # p.changeDynamics(pawn_id, -1, lateralFriction=1.5, restitution=0.0, etc.)

    # Improve solver & simulation parameters
    p.setPhysicsEngineParameter(numSolverIterations=150)
    # You might also set other parameters if needed, like ERP, CFM, etc.

    logger.info("Robot loaded and configured with high friction fingers.")
    return robot_id, arm_joint_indices, gripper_indices


def load_environment():
    """Load the chess board."""
    try:
        board_id = p.loadURDF(config.environment.board_urdf, basePosition=config.environment.board_position,
                              baseOrientation=config.environment.board_orientation, useFixedBase=True)
        logger.info(f"Environment loaded (Board ID: {board_id}).")
        return board_id
    except p.error as e:
        logger.error(f"Error loading board URDF '{config.environment.board_urdf}': {e}")
        return None

# do the same for the gripper finger links (use correct link index)

# optional: set contact stiffness/damping (newer PyBullet builds)
def load_pawn(position):
    """Load a pawn at a given position."""
    try:
        pawn_id = p.loadURDF(config.object.pawn_urdf, basePosition=position, useFixedBase=False)
        # --- Key Change 2: Increase Pawn Friction ---
        # Apply to all links of the pawn (assuming single link URDF, link index 0)
        p.changeDynamics(pawn_id, -1, lateralFriction=3, spinningFriction=3, rollingFriction=3) # Increased from 1.0
        p.changeDynamics(pawn_id, -1, contactStiffness=1e4, contactDamping=1e3)
        # Let the pawn settle briefly
        wait(config.simulation.settle_steps // 2)
        logger.info(f"Pawn loaded (ID: {pawn_id}) at {position} with increased friction.")
        return pawn_id, position
    except p.error as e:
        logger.error(f"Error loading pawn URDF '{config.object.pawn_urdf}': {e}")
        return None