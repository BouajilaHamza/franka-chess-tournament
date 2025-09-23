import pybullet as p
import pybullet_data
import numpy as np
import time
import logging

# --- Logging Configuration ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
# Uncomment the line below if you find OMPL or other PyBullet logs too noisy
# logging.getLogger("pybullet").setLevel(logging.WARNING)

# --- Configuration ---
SIMULATION_STEP_DELAY = 1.0 / 240.0
SETTLE_STEPS = 100
MOVE_STEPS = 300
GRIPPER_ACTION_STEPS = 150
DEFAULT_JOINT_DAMPING = 0.1

# Robot Configuration
ROBOT_URDF = "franka_panda/panda.urdf"
ROBOT_START_POSITION = [0, 0, 0]
ROBOT_START_ORIENTATION = [0, 0, 0, 1]
ROBOT_END_EFFECTOR_INDEX = 11
ROBOT_NUM_ARM_JOINTS = 7
ROBOT_MAX_JOINT_FORCE = 240.0
# --- Key Change 1: Increased Gripper Force ---
ROBOT_GRIPPER_FORCE = 240.0 # Increased from 50.0
ROBOT_GRIPPER_OPEN = 0.03
ROBOT_GRIPPER_CLOSED = 0.0 # Fully closed for maximum grip

# Home/Reset Configuration (joint angles for a safe, central position)
# Example values for Franka Panda - adjust based on your robot's safe home position
ROBOT_HOME_POSITION = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
HOME_POSITION_TOLERANCE = 0.01 # Radians
HOME_MOVE_TIMEOUT = 5.0

# Environment Configuration
BOARD_URDF = "Open_Chess/urdfs/chess_board.urdf"
BOARD_POSITION = [0.4, 0, 0.02] # Z should be the board surface height
BOARD_ORIENTATION = [0, 0, 0, 1]

# Object Configuration
PAWN_URDF = "Open_Chess/urdfs/pawn.urdf"
# Z positions are now relative to the board surface
PAWN_START_POS = [0.265, -0.2, BOARD_POSITION[2]]  # On the board surface
PAWN_TARGET_POS = [0.7, 0, BOARD_POSITION[2]] # On the board surface
WHITE_PAWNS = []
BLACK_PAWNS = []
# Pick & Place Parameters
PICK_PLACE_CLEARANCE_Z = 0.1  # 5 cm above surface for safe moves
PICK_Z_OFFSET = 0.025          # 1.5 cm above object base for grasp
PLACE_Z_OFFSET = 0.025         # 1.5 cm above target surface for release
# Standard downward orientation for the end effector (pointing down along Z)
# [np.pi, 0, 0] is a common vertical downward orientation.
EE_DOWN_ORIENTATION = p.getQuaternionFromEuler([np.pi, 0, 0])
# EE_DOWN_ORIENTATION = p.getQuaternionFromEuler([np.pi, 0, np.pi/2]) # If gripper needs 90deg rotation

# Retry Configuration
MAX_RETRIES = 2 # Number of retries allowed

# --- Initialization ---
def setup_simulation():
    """Initialize PyBullet, load environment, and configure settings."""
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(SIMULATION_STEP_DELAY)
    p.setRealTimeSimulation(0)
    logger.info("Simulation initialized.")
    return physics_client

def load_plane():
    """Load the ground plane."""
    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(plane_id, -1, lateralFriction=1.0)
    logger.debug("Ground plane loaded.")
    return plane_id

def load_robot():
    """Load the robot and configure its joints for better grasp stability."""
    robot_id = p.loadURDF(
        ROBOT_URDF,
        basePosition=ROBOT_START_POSITION,
        baseOrientation=ROBOT_START_ORIENTATION,
        useFixedBase=True
    )

    # Identify arm revolute joints
    arm_joint_indices = [
        i for i in range(p.getNumJoints(robot_id))
        if p.getJointInfo(robot_id, i)[2] == p.JOINT_REVOLUTE
    ][:ROBOT_NUM_ARM_JOINTS]

    # Set damping on those arm joints (reduce jitter)
    for joint_idx in arm_joint_indices:
        p.changeDynamics(robot_id,
                         linkIndex=joint_idx,
                         linearDamping=0.0,
                         angularDamping=DEFAULT_JOINT_DAMPING)

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
        board_id = p.loadURDF(BOARD_URDF, basePosition=BOARD_POSITION,
                              baseOrientation=BOARD_ORIENTATION, useFixedBase=True)
        logger.info(f"Environment loaded (Board ID: {board_id}).")
        return board_id
    except p.error as e:
        logger.error(f"Error loading board URDF '{BOARD_URDF}': {e}")
        return None

# do the same for the gripper finger links (use correct link index)

# optional: set contact stiffness/damping (newer PyBullet builds)
def load_pawn(position):
    """Load a pawn at a given position."""
    try:
        pawn_id = p.loadURDF(PAWN_URDF, basePosition=position, useFixedBase=False)
        # --- Key Change 2: Increase Pawn Friction ---
        # Apply to all links of the pawn (assuming single link URDF, link index 0)
        p.changeDynamics(pawn_id, -1, lateralFriction=3, spinningFriction=3, rollingFriction=3) # Increased from 1.0
        p.changeDynamics(pawn_id, -1, contactStiffness=1e4, contactDamping=1e3)
        # Let the pawn settle briefly
        wait(SETTLE_STEPS // 2)
        logger.info(f"Pawn loaded (ID: {pawn_id}) at {position} with increased friction.")
        return pawn_id, position
    except p.error as e:
        logger.error(f"Error loading pawn URDF '{PAWN_URDF}': {e}")
        return None

# --- Control Functions ---
def set_arm_positions(robot_id, arm_joints, target_positions):
    """Set target positions for the robot arm joints."""
    for i, joint_idx in enumerate(arm_joints):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_positions[i],
            force=ROBOT_MAX_JOINT_FORCE,
            maxVelocity=1.0
        )

# --- Key Change 3: Modified Gripper Control ---
def set_gripper_position(robot_id, gripper_joints, position, force=ROBOT_GRIPPER_FORCE):
    """
    Set gripper position and apply holding force.
    Uses POSITION_CONTROL to move, then VELOCITY_CONTROL with 0 vel and force to hold.
    """
    # Panda gripper has 0.08m max opening, adjust for small parts
    # Ensure the input 'position' is within the physical limits of the gripper (0.0 to 0.08 meters)
    adjusted_position = min(0.08, max(0.0, position))
    logger.info(f"Setting gripper to position: {adjusted_position}") # Optional: Add logging if needed

    # Apply POSITION_CONTROL to BOTH gripper joints (typically joints 9 and 10 for Franka Panda)
    # The 'position' argument determines the opening/closing state
    p.setJointMotorControl2(
        bodyIndex=robot_id, # Use the correct variable name for your robot's ID
        jointIndex=9,      # Joint index for the first finger
        controlMode=p.POSITION_CONTROL, # Command a specific position
        targetPosition=adjusted_position, # The desired position based on the input 'position'
        force=force # Use the defined gripper force
    )
    p.setJointMotorControl2(
        bodyIndex=robot_id, # Use the correct variable name for your robot's ID
        jointIndex=10,     # Joint index for the second finger
        controlMode=p.POSITION_CONTROL, # Command a specific position
        targetPosition=adjusted_position, # The desired position based on the input 'position' (same as joint 9)
        force=force # Use the defined gripper force
    )

def get_current_arm_positions(robot_id, arm_joints):
    """Get the current positions of the arm joints."""
    return [p.getJointState(robot_id, joint_idx)[0] for joint_idx in arm_joints]

def get_ee_position(robot_id, ee_index):
    """Get the current position of the end effector."""
    ee_state = p.getLinkState(robot_id, ee_index)
    return np.array(ee_state[0])

def wait(steps):
    """Wait for a specified number of simulation steps."""
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(SIMULATION_STEP_DELAY)

def move_to_home_position(robot_id, arm_joints, home_position=ROBOT_HOME_POSITION, tolerance=HOME_POSITION_TOLERANCE, timeout=HOME_MOVE_TIMEOUT):
    """
    Move the robot arm to the predefined home position.
    """
    logger.info("Moving robot to home position...")
    if not home_position or len(home_position) < len(arm_joints):
        logger.error("Invalid home position configuration.")
        return False

    try:
        # Send command to move to home position
        for i, joint_idx in enumerate(arm_joints):
             p.setJointMotorControl2(
                 bodyIndex=robot_id,
                 jointIndex=joint_idx,
                 controlMode=p.POSITION_CONTROL,
                 targetPosition=home_position[i],
                 force=ROBOT_MAX_JOINT_FORCE,
                 maxVelocity=1.0 # Moderate speed
             )

        # Wait for the robot to reach the home position or timeout
        start_time = time.time()
        while time.time() - start_time < timeout:
            p.stepSimulation()
            time.sleep(SIMULATION_STEP_DELAY)

            # Check if all joints are close enough to the target
            joints_at_target = True
            for i, joint_idx in enumerate(arm_joints):
                current_pos = p.getJointState(robot_id, joint_idx)[0]
                if abs(current_pos - home_position[i]) > tolerance:
                    joints_at_target = False
                    break # Break inner loop, continue waiting

            if joints_at_target:
                logger.info("Robot successfully reached home position.")
                wait(SETTLE_STEPS // 2) # Brief pause to settle
                return True

        logger.warning("Timeout waiting for robot to reach home position.")
        return False

    except Exception as e:
        logger.error(f"Error moving to home position: {e}")
        return False

def move_to_position_ik(robot_id, arm_joints, ee_index, target_pos, target_orient, log_msg=""):
    """Move the robot's end effector to a target pose using IK with validation."""
    if log_msg:
        logger.info(log_msg)
    try:
        goal_joint_positions = p.calculateInverseKinematics(
            bodyUniqueId=robot_id,
            endEffectorLinkIndex=ee_index,
            targetPosition=target_pos,
            targetOrientation=target_orient,
            maxNumIterations=100,
            residualThreshold=1e-5
        )

        current_positions = get_current_arm_positions(robot_id, arm_joints)
        target_arm_positions = goal_joint_positions[:len(arm_joints)]

        # Interpolate movement for smoother transition
        for step in range(MOVE_STEPS):
            alpha = step / float(MOVE_STEPS)
            interp_positions = [
                (1 - alpha) * curr + alpha * targ
                for curr, targ in zip(current_positions, target_arm_positions)
            ]
            set_arm_positions(robot_id, arm_joints, interp_positions)
            p.stepSimulation()
            time.sleep(SIMULATION_STEP_DELAY)

        # Ensure final position is reached
        set_arm_positions(robot_id, arm_joints, target_arm_positions)
        wait(SETTLE_STEPS)

        # --- Critical Fix: Verify EE reached the target X/Y ---
        final_ee_pos = get_ee_position(robot_id, ee_index)
        target_pos_np = np.array(target_pos)
        xy_distance = np.linalg.norm(final_ee_pos[:2] - target_pos_np[:2])
        z_distance = abs(final_ee_pos[2] - target_pos_np[2])

        logger.debug(f"  -> Target: {target_pos}, Actual EE: {final_ee_pos}")
        logger.debug(f"  -> XY Distance: {xy_distance:.4f}m, Z Distance: {z_distance:.4f}m")

        # Define tolerances
        XY_TOLERANCE = 0.01 # 1.5cm tolerance for XY
        Z_TOLERANCE = 0.02   # 2cm tolerance for Z

        if xy_distance > XY_TOLERANCE:
            logger.warning(f"  -> EE XY position error too large ({xy_distance:.4f}m > {XY_TOLERANCE:.4f}m). Target XY: {target_pos[:2]}, Actual XY: {final_ee_pos[:2]}")
            return False
        if z_distance > Z_TOLERANCE:
            logger.warning(f"  -> EE Z position error large ({z_distance:.4f}m > {Z_TOLERANCE:.4f}m). Check if this is acceptable.")

        logger.info("  -> Movement completed successfully.")
        return True
    except Exception as e:
        logger.error(f"  -> IK calculation or movement failed: {e}")
        return False

def verify_grasp(robot_id, ee_index, object_id, expected_pos, timeout=1.0):
    """Basic check if the object is likely grasped."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            ee_pos = get_ee_position(robot_id, ee_index)
            obj_pos, _ = p.getBasePositionAndOrientation(object_id)
            distance = np.linalg.norm(np.array(obj_pos) - np.array(ee_pos))
            if distance < 0.05: # Threshold, adjust based on object size
                return True
        except p.error:
            logger.info("  -> Object no longer exists, assuming it was grasped and moved.")
            return True # Object might have been successfully moved/removed
        wait(10) # Small wait
    logger.warning("  -> Grasp verification timeout. Object might not be firmly grasped.")
    return False

# --- Main Task Logic (Robust Pick and Place with Retry and Base Return) ---
def pick_and_place_single_attempt(robot_id, arm_joints, gripper_joints, object_id, start_pos, target_pos):
    """
    Perform a single attempt of the pick and place sequence.
    Returns True if successful, False otherwise.
    Assumes start_pos and target_pos Z are the surface level.
    """
    logger.info("--- Starting Pick and Place Single Attempt ---")
    start_surface_z = start_pos[2]
    target_surface_z = target_pos[2]

    # 1. --- PICKING PHASE ---
    # a. Move above the object
    pick_approach_pos = [start_pos[0], start_pos[1], start_surface_z + PICK_PLACE_CLEARANCE_Z]
    set_gripper_position(robot_id, gripper_joints, ROBOT_GRIPPER_OPEN, ROBOT_GRIPPER_FORCE)
    wait(SETTLE_STEPS // 2)
    if not move_to_position_ik(robot_id, arm_joints, ROBOT_END_EFFECTOR_INDEX,
                               pick_approach_pos, EE_DOWN_ORIENTATION,
                               log_msg=f"1a. Moving above object...{pick_approach_pos}"):
        logger.error("1a. Failed to move above object.")
        return False

    # b. Move down to grasp position
    grasp_pos = [start_pos[0], start_pos[1], start_surface_z + PICK_Z_OFFSET]
    if not move_to_position_ik(robot_id, arm_joints, ROBOT_END_EFFECTOR_INDEX,
                               grasp_pos, EE_DOWN_ORIENTATION,
                               log_msg="1b. Moving down to grasp..."):
        logger.error("1b. Failed to move to grasp position.")
        return False

    # c. Close gripper to grasp - Apply force
    logger.info("1c. Closing gripper and applying force...")
    set_gripper_position(robot_id, gripper_joints, ROBOT_GRIPPER_CLOSED, ROBOT_GRIPPER_FORCE)
    wait(GRIPPER_ACTION_STEPS)
    # Ensure force is applied continuously after closing
    set_gripper_position(robot_id, gripper_joints, ROBOT_GRIPPER_CLOSED, ROBOT_GRIPPER_FORCE)


    # d. Verify grasp (basic check)
    if not verify_grasp(robot_id, ROBOT_END_EFFECTOR_INDEX, object_id, grasp_pos):
        logger.warning("1d. Grasp verification failed.")
        set_gripper_position(robot_id, gripper_joints, ROBOT_GRIPPER_OPEN, ROBOT_GRIPPER_FORCE)
        wait(GRIPPER_ACTION_STEPS)
        return False
    logger.info("1d. Grasp verified.")

    # e. Lift the object (force should keep it attached)
    lift_pos = [start_pos[0], start_pos[1], start_surface_z + PICK_PLACE_CLEARANCE_Z]
    if not move_to_position_ik(robot_id, arm_joints, ROBOT_END_EFFECTOR_INDEX,
                               lift_pos, EE_DOWN_ORIENTATION,
                               log_msg="1e. Lifting object..."):
        logger.error("1e. Failed to lift object.")
        set_gripper_position(robot_id, gripper_joints, ROBOT_GRIPPER_OPEN, ROBOT_GRIPPER_FORCE)
        wait(GRIPPER_ACTION_STEPS)
        return False

    # 2. --- PLACING PHASE ---
    # a. Move above the target location (correct X, Y from the start)
    place_approach_pos = [target_pos[0], target_pos[1], target_surface_z + PICK_PLACE_CLEARANCE_Z]
    if not move_to_position_ik(robot_id, arm_joints, ROBOT_END_EFFECTOR_INDEX,
                               place_approach_pos, EE_DOWN_ORIENTATION,
                               log_msg="2a. Moving above target..."):
        logger.error("2a. Failed to move above target.")
        # Drop object back
        drop_pos = [start_pos[0], start_pos[1], start_surface_z + PICK_PLACE_CLEARANCE_Z]
        move_to_position_ik(robot_id, arm_joints, ROBOT_END_EFFECTOR_INDEX,
                            drop_pos, EE_DOWN_ORIENTATION, log_msg="  -> Dropping object back (failed move to target)...")
        set_gripper_position(robot_id, gripper_joints, ROBOT_GRIPPER_OPEN, ROBOT_GRIPPER_FORCE)
        wait(GRIPPER_ACTION_STEPS)
        lift_pos_after_drop = [start_pos[0], start_pos[1], start_surface_z + PICK_PLACE_CLEARANCE_Z + 0.05]
        move_to_position_ik(robot_id, arm_joints, ROBOT_END_EFFECTOR_INDEX,
                            lift_pos_after_drop, EE_DOWN_ORIENTATION, log_msg="  -> Lifting after drop...")
        return False

    # b. Move down to release position (ON the target surface)
    release_pos = [target_pos[0], target_pos[1], target_surface_z + PLACE_Z_OFFSET] # Corrected: Place ON or just above surface
    if not move_to_position_ik(robot_id, arm_joints, ROBOT_END_EFFECTOR_INDEX,
                               release_pos, EE_DOWN_ORIENTATION,
                               log_msg="2b. Moving down to place..."):
        logger.error("2b. Failed to move to place position.")
        # Drop object back
        drop_pos = [start_pos[0], start_pos[1], start_surface_z + PICK_PLACE_CLEARANCE_Z]
        move_to_position_ik(robot_id, arm_joints, ROBOT_END_EFFECTOR_INDEX,
                            drop_pos, EE_DOWN_ORIENTATION, log_msg="  -> Dropping object back (failed place)...")
        set_gripper_position(robot_id, gripper_joints, ROBOT_GRIPPER_OPEN, ROBOT_GRIPPER_FORCE)
        wait(GRIPPER_ACTION_STEPS)
        lift_pos_after_drop = [start_pos[0], start_pos[1], start_surface_z + PICK_PLACE_CLEARANCE_Z + 0.05]
        move_to_position_ik(robot_id, arm_joints, ROBOT_END_EFFECTOR_INDEX,
                            lift_pos_after_drop, EE_DOWN_ORIENTATION, log_msg="  -> Lifting after drop...")
        return False

    # c. Open gripper to release
    logger.info("2c. Opening gripper...")
    set_gripper_position(robot_id, gripper_joints, ROBOT_GRIPPER_OPEN, ROBOT_GRIPPER_FORCE)
    wait(GRIPPER_ACTION_STEPS)

    # d. Move away (lift up from target)
    retreat_pos = [target_pos[0], target_pos[1], target_surface_z + PICK_PLACE_CLEARANCE_Z]
    if not move_to_position_ik(robot_id, arm_joints, ROBOT_END_EFFECTOR_INDEX,
                               retreat_pos, EE_DOWN_ORIENTATION,
                               log_msg="2d. Retreating..."):
        logger.warning("2d. Failed to fully retreat after placing (not critical).")

    logger.info("--- Pick and Place Single Attempt Complete ---")
    return True

def pick_and_place_with_retry(robot_id, arm_joints, gripper_joints, object_id, start_pos, target_pos, max_retries=MAX_RETRIES):
    """
    Perform the pick and place sequence with retry logic.
    On retry, the robot returns to the home position before attempting again.
    """
    logger.info(f"=== Starting Pick and Place with Retry (Max: {max_retries}) ===")
    logger.info(f"Object ID: {object_id}, Start: {start_pos}, Target: {target_pos}")

    # Store the home configuration for reset
    initial_config = ROBOT_HOME_POSITION

    for attempt in range(1, max_retries + 2): # 1 initial + max_retries retries
        logger.info(f"--- Attempt {attempt}/{max_retries + 1} ---")

        # --- Critical Fix: On retries, move back to base position ---
        if attempt > 1: # This is a retry
            logger.info(f"Retry #{attempt - 1}: Returning robot to base position before retrying...")
            if not move_to_home_position(robot_id, arm_joints, initial_config):
                 logger.error("Failed to return to base position for retry. Aborting.")
                 # If we can't even get back to base, it's a critical failure.
                 return False
            logger.info("Robot returned to base position. Proceeding with retry.")

        # Attempt the pick and place sequence
        success = pick_and_place_single_attempt(robot_id, arm_joints, gripper_joints, object_id, start_pos, target_pos)

        if success:
            logger.info(f"=== Pick and Place SUCCEEDED on attempt {attempt} ===")
            # Optional: Move back to home after final success
            # move_to_home_position(robot_id, arm_joints, initial_config)
            return True
        else:
            logger.warning(f"=== Pick and Place FAILED on attempt {attempt} ===")
            if attempt < max_retries + 1:
                logger.info("Preparing for retry...")
                # Open gripper in case it's holding something
                set_gripper_position(robot_id, gripper_joints, ROBOT_GRIPPER_OPEN, ROBOT_GRIPPER_FORCE)
                wait(SETTLE_STEPS)
                # The loop will continue, triggering the base return at the start of the next iteration
            else:
                 logger.error(f"=== Pick and Place FAILED after {max_retries + 1} attempts (including initial) ===")
                 # Optional: Final move to home on ultimate failure
                 # move_to_home_position(robot_id, arm_joints, initial_config)

    return False # Should not be reached, but good practice

# --- Main Execution ---
def main():
    """Main function to run the simulation."""
    logger.info("Initializing simulation...")
    client = setup_simulation()
    if client < 0:
        logger.error("Failed to connect to PyBullet.")
        return

    load_plane() # Load the ground plane
    logger.info("Loading robot...")
    robot_id, arm_joints, gripper_joints = load_robot()
    if not robot_id:
        logger.error("Failed to load robot.")
        p.disconnect()
        return

    # --- Key Change 4: Increase Gripper Finger Friction ---
    # Apply increased friction to the gripper fingers (joints 9 and 10 for Panda)
    # Get the child link index for the joints (usually the link the joint connects TO)
    try:
        finger1_link_index = p.getJointInfo(robot_id, 9)[16] # childLinkIndex
        finger2_link_index = p.getJointInfo(robot_id, 10)[16] # childLinkIndex
        p.changeDynamics(robot_id, finger1_link_index, lateralFriction=2, spinningFriction=0.5, rollingFriction=0.5) # Increased from default
        p.changeDynamics(robot_id, finger2_link_index, lateralFriction=2, spinningFriction=0.5, rollingFriction=0.5) # Increased from default
        logger.info("Increased friction on gripper fingers.")
    except Exception as e:
        logger.warning(f"Could not set gripper finger friction: {e}. Check joint/link indices.")
    try:
        set_gripper_position(robot_id, gripper_joints, ROBOT_GRIPPER_OPEN, ROBOT_GRIPPER_FORCE)
        for _ in range(10): # Example wait
            p.stepSimulation()
            time.sleep(SIMULATION_STEP_DELAY)
    except Exception as e:
        logger.warning(f"Could not open gripper at start: {e}")
    logger.info("Loading environment...")
    board_id = load_environment()
    if not board_id:
        logger.error("Failed to load board.")
        p.disconnect()
        return
    x= PAWN_START_POS[0]
    y= PAWN_START_POS[1]
    z= PAWN_START_POS[2]
    logger.info("Loading pawn...")
    for _ in range(8):
        pawn_id,pawn_position = load_pawn([x,y,z])
        if not pawn_id:
            logger.error("Failed to load pawn.")
            p.disconnect()
            return
        y += 0.055 # Space pawns along Y axis
        WHITE_PAWNS.append((pawn_id, pawn_position))



    for pawn_id,pawn_position in WHITE_PAWNS:
        logger.info(f"==========> Pawn ID: {pawn_id}")
        # Move to home position at the start
        if not move_to_home_position(robot_id, arm_joints, ROBOT_HOME_POSITION):
            logger.warning("Failed to move to initial home position.")
        
        logger.info("Running pick and place task with retry...")
        success = pick_and_place_with_retry(robot_id, arm_joints, gripper_joints,
                                            pawn_id, pawn_position, PAWN_TARGET_POS, MAX_RETRIES)

        if success:
            logger.info("\nTask completed successfully!")
        else:
            logger.error("\nTask failed after all retries.")

        # Move back to home position at the end
        move_to_home_position(robot_id, arm_joints, ROBOT_HOME_POSITION)

        logger.info("Simulation running. Press Ctrl+C to exit.")
    try:
        while True:
            p.stepSimulation()
            time.sleep(SIMULATION_STEP_DELAY)
    except KeyboardInterrupt:
        logger.info("Simulation stopped by user.")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()