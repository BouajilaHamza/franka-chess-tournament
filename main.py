import pybullet as p
import numpy as np
import time
import logging
from simulation.builder import EnvironmentBuilder
from utils.simulation import wait
from configs.config import config
# --- Logging Configuration ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
# Uncomment the line below if you find OMPL or other PyBullet logs too noisy
# logging.getLogger("pybullet").setLevel(logging.WARNING)




# --- Control Functions ---
def set_arm_positions(robot_id, arm_joints, target_positions):
    """Set target positions for the robot arm joints."""
    for i, joint_idx in enumerate(arm_joints):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_positions[i],
            force=config.robot.first.max_joint_force,
            maxVelocity=1.0
        )

# --- Key Change 3: Modified Gripper Control ---
def set_gripper_position(robot_id, gripper_joints, position, force=config.robot.first.gripper_force):
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


def move_to_home_position(robot_id, arm_joints, home_position=config.robot.first.home_position, tolerance=config.robot.first.home_position_tolerance, timeout=config.robot.first.home_move_timeout):
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
                 force=config.robot.first.max_joint_force,
                 maxVelocity=1.0 # Moderate speed
             )

        # Wait for the robot to reach the home position or timeout
        start_time = time.time()
        while time.time() - start_time < timeout:
            p.stepSimulation()
            time.sleep(config.simulation.step_delay)

            # Check if all joints are close enough to the target
            joints_at_target = True
            for i, joint_idx in enumerate(arm_joints):
                current_pos = p.getJointState(robot_id, joint_idx)[0]
                if abs(current_pos - home_position[i]) > tolerance:
                    joints_at_target = False
                    break # Break inner loop, continue waiting

            if joints_at_target:
                logger.info("Robot successfully reached home position.")
                wait(config.simulation.settle_steps // 2) # Brief pause to settle
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
        for step in range(config.simulation.move_steps):
            alpha = step / float(config.simulation.move_steps)
            interp_positions = [
                (1 - alpha) * curr + alpha * targ
                for curr, targ in zip(current_positions, target_arm_positions)
            ]
            set_arm_positions(robot_id, arm_joints, interp_positions)
            p.stepSimulation()
            time.sleep(config.simulation.step_delay)

        # Ensure final position is reached
        set_arm_positions(robot_id, arm_joints, target_arm_positions)
        wait(config.simulation.settle_steps)

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
    pick_approach_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
    set_gripper_position(robot_id, gripper_joints, config.robot.first.gripper_open, config.robot.first.gripper_force) # Ensure gripper is open
    wait(config.simulation.settle_steps // 2)
    if not move_to_position_ik(robot_id, arm_joints, config.robot.first.end_effector_index,
                               pick_approach_pos, config.pick_place.ee_down_orientation,
                               log_msg=f"1a. Moving above object...{pick_approach_pos}"):
        logger.error("1a. Failed to move above object.")
        return False

    # b. Move down to grasp position
    grasp_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.pick_z_offset]
    if not move_to_position_ik(robot_id, arm_joints, config.robot.first.end_effector_index,
                               grasp_pos, config.pick_place.ee_down_orientation,
                               log_msg="1b. Moving down to grasp..."):
        logger.error("1b. Failed to move to grasp position.")
        return False

    # c. Close gripper to grasp - Apply force
    logger.info("1c. Closing gripper and applying force...")
    set_gripper_position(robot_id, gripper_joints, config.robot.first.gripper_closed, config.robot.first.gripper_force) # Ensure gripper is open
    wait(config.simulation.gripper_action_steps)
    # Ensure force is applied continuously after closing
    set_gripper_position(robot_id, gripper_joints, config.robot.first.gripper_closed, config.robot.first.gripper_force) # Ensure gripper is open


    # d. Verify grasp (basic check)
    if not verify_grasp(robot_id, config.robot.first.end_effector_index, object_id, grasp_pos):
        logger.warning("1d. Grasp verification failed.")
        set_gripper_position(robot_id, gripper_joints, config.robot.first.gripper_open, config.robot.first.gripper_force) # Ensure gripper is open
        wait(config.simulation.gripper_action_steps)
        return False
    logger.info("1d. Grasp verified.")

    # e. Lift the object (force should keep it attached)
    lift_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
    if not move_to_position_ik(robot_id, arm_joints, config.robot.first.end_effector_index,
                               lift_pos, config.pick_place.ee_down_orientation,
                               log_msg="1e. Lifting object..."):
        logger.error("1e. Failed to lift object.")
        set_gripper_position(robot_id, gripper_joints, config.robot.first.gripper_open, config.robot.first.gripper_force) # Ensure gripper is open
        wait(config.simulation.gripper_action_steps)
        return False

    # 2. --- PLACING PHASE ---
    # a. Move above the target location (correct X, Y from the start)
    place_approach_pos = [target_pos[0], target_pos[1], target_surface_z + config.pick_place.clearance_z]
    if not move_to_position_ik(robot_id, arm_joints, config.robot.first.end_effector_index,
                               place_approach_pos, config.pick_place.ee_down_orientation,
                               log_msg="2a. Moving above target..."):
        logger.error("2a. Failed to move above target.")
        # Drop object back
        drop_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
        move_to_position_ik(robot_id, arm_joints, config.robot.first.end_effector_index,
                            drop_pos, config.pick_place.ee_down_orientation, log_msg="  -> Dropping object back (failed move to target)...")
        set_gripper_position(robot_id, gripper_joints, config.robot.first.gripper_open, config.robot.first.gripper_force) # Ensure gripper is open
        wait(config.simulation.gripper_action_steps)
        lift_pos_after_drop = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z + 0.05]
        move_to_position_ik(robot_id, arm_joints, config.robot.first.end_effector_index,
                            lift_pos_after_drop, config.pick_place.ee_down_orientation, log_msg="  -> Lifting after drop...")
        return False

    # b. Move down to release position (ON the target surface)
    release_pos = [target_pos[0], target_pos[1], target_surface_z + config.pick_place.place_z_offset] # Corrected: Place ON or just above surface
    if not move_to_position_ik(robot_id, arm_joints, config.robot.first.end_effector_index,
                               release_pos, config.pick_place.ee_down_orientation,
                               log_msg="2b. Moving down to place..."):
        logger.error("2b. Failed to move to place position.")
        # Drop object back
        drop_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
        move_to_position_ik(robot_id, arm_joints, config.robot.first.end_effector_index,
                            drop_pos, config.pick_place.ee_down_orientation, log_msg="  -> Dropping object back (failed place)...")
        set_gripper_position(robot_id, gripper_joints, config.robot.first.gripper_open, config.robot.first.gripper_force) # Ensure gripper is open
        wait(config.simulation.gripper_action_steps)
        lift_pos_after_drop = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z + 0.05]
        move_to_position_ik(robot_id, arm_joints, config.robot.first.end_effector_index,
                            lift_pos_after_drop, config.pick_place.ee_down_orientation, log_msg="  -> Lifting after drop...")
        return False

    # c. Open gripper to release
    logger.info("2c. Opening gripper...")
    set_gripper_position(robot_id, gripper_joints, config.robot.first.gripper_open, config.robot.first.gripper_force) # Ensure gripper is open
    wait(config.simulation.gripper_action_steps)

    # d. Move away (lift up from target)
    retreat_pos = [target_pos[0], target_pos[1], target_surface_z + config.pick_place.clearance_z]
    if not move_to_position_ik(robot_id, arm_joints, config.robot.first.end_effector_index,
                               retreat_pos, config.pick_place.ee_down_orientation,
                               log_msg="2d. Retreating..."):
        logger.warning("2d. Failed to fully retreat after placing (not critical).")

    logger.info("--- Pick and Place Single Attempt Complete ---")
    return True

def pick_and_place_with_retry(robot_id, arm_joints, gripper_joints, object_id, start_pos, target_pos, max_retries=config.task.max_retries):
    """
    Perform the pick and place sequence with retry logic.
    On retry, the robot returns to the home position before attempting again.
    """
    logger.info(f"=== Starting Pick and Place with Retry (Max: {max_retries}) ===")
    logger.info(f"Object ID: {object_id}, Start: {start_pos}, Target: {target_pos}")

    # Store the home configuration for reset
    initial_config = config.robot.first.home_position

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
                set_gripper_position(robot_id, gripper_joints, config.robot.first.gripper_open, config.robot.first.gripper_force) # Ensure gripper is open
                wait(config.simulation.settle_steps)
                # The loop will continue, triggering the base return at the start of the next iteration
            else:
                 logger.error(f"=== Pick and Place FAILED after {max_retries + 1} attempts (including initial) ===")
                 # Optional: Final move to home on ultimate failure
                 # move_to_home_position(robot_id, arm_joints, initial_config)

    return False # Should not be reached, but good practice

# --- Main Execution ---
def main():
    """Main function to run the simulation."""
    # --- Example Usage (Conceptual) ---
    builder = EnvironmentBuilder()
    env_components = (builder
                      .connect()
                      .load_ground_plane()
                      .load_robots()
                      .load_chess_board()
                      .load_pieces() # This calls _define_square_mapping internally
                      .build()
                      )
    
    board_id = env_components['board_id']
    robot1_id = env_components['robot1']['id']
    robot1_arm_joints = env_components['robot1']['arm_joints']
    robot1_gripper_joints = env_components['robot1']['gripper_joints']
    robot2_id = env_components['robot2']['id']
    # ... etc
    piece_locations = env_components['piece_ids']
    square_coords = env_components['square_to_world_coords']
    logger.info(f"Loaded board ID: {board_id}, Robot1 ID: {robot1_id}, Robot2 ID: {robot2_id}")
    logger.info(f"Piece Locations: {piece_locations}")
    logger.info(f"Square Coordinates: {square_coords}")
    logger.info("robot1_arm_joints: " + str(robot1_arm_joints))
    logger.info("robot1_gripper_joints: " + str(robot1_gripper_joints))
    # logger.info("Initializing simulation...")
    # client = setup_simulation()
    # if client < 0:
    #     logger.error("Failed to connect to PyBullet.")
    #     return

    # load_plane() # Load the ground plane
    # logger.info("Loading first robot...")
    # robot_id, arm_joints, gripper_joints = load_robot(config.robot.first.start_position, config.robot.first.start_orientation)
    # if not robot_id:
    #     logger.error("Failed to load robot.")
    #     p.disconnect()
    #     return
    
    # logger.info("Loading second robot...")
    # second_robot_id, second_arm_joints, second_gripper_joints = load_robot(config.robot.second.start_position, config.robot.second.start_orientation)
    # if not robot_id:
    #     logger.error("Failed to load robot.")
    #     p.disconnect()
    #     return

    # --- Key Change 4: Increase Gripper Finger Friction ---
    # Apply increased friction to the gripper fingers (joints 9 and 10 for Panda)
    # Get the child link index for the joints (usually the link the joint connects TO)
    # try:
    #     finger1_link_index = p.getJointInfo(robot_id, 9)[16] # childLinkIndex
    #     finger2_link_index = p.getJointInfo(robot_id, 10)[16] # childLinkIndex
    #     p.changeDynamics(robot_id, finger1_link_index, lateralFriction=2, spinningFriction=0.5, rollingFriction=0.5) # Increased from default
    #     p.changeDynamics(robot_id, finger2_link_index, lateralFriction=2, spinningFriction=0.5, rollingFriction=0.5) # Increased from default
    #     logger.info("Increased friction on gripper fingers.")
    # except Exception as e:
    #     logger.warning(f"Could not set gripper finger friction: {e}. Check joint/link indices.")
    # try:
    #     set_gripper_position(robot_id, gripper_joints, config.robot.first.gripper_open, config.robot.first.gripper_force) # Ensure gripper is open
    #     for _ in range(10): # Example wait
    #         p.stepSimulation()
    #         time.sleep(config.simulation.step_delay)
    # except Exception as e:
    #     logger.warning(f"Could not open gripper at start: {e}")
    # logger.info("Loading environment...")
    # board_id = load_environment()
    # if not board_id:
    #     logger.error("Failed to load board.")
    #     p.disconnect()
    #     return
    # x= config.object.pawn_start_pos[0]
    # y= config.object.pawn_start_pos[1]
    # z= config.object.pawn_start_pos[2]
    # logger.info("Loading white pieces ...")
    # for _ in range(8):
    #     pawn_id,pawn_position = load_pawn([x,y,z])
    #     if not pawn_id:
    #         logger.error("Failed to load pawn.")
    #         p.disconnect()
    #         return
    #     y += 0.055 # Space pawns along Y axis
    #     config.object.white_pawns.append((pawn_id, pawn_position))



    # for pawn_id,pawn_position in config.object.white_pawns:
    #     logger.info(f"==========> Pawn ID: {pawn_id}")
    #     # Move to home position at the start
    #     if not move_to_home_position(robot_id, arm_joints, config.robot.first.home_position):
    #         logger.warning("Failed to move to initial home position.")
        
    #     logger.info("Running pick and place task with retry...")
    #     success = pick_and_place_with_retry(robot_id, arm_joints, gripper_joints,
    #                                         pawn_id, pawn_position, config.object.pawn_target_pos, config.task.max_retries)

    #     if success:
    #         logger.info("\nTask completed successfully!")
    #     else:
    #         logger.error("\nTask failed after all retries.")

    #     # Move back to home position at the end
    #     move_to_home_position(robot_id, arm_joints, config.robot.first.home_position)

    #     logger.info("Simulation running. Press Ctrl+C to exit.")
    try:
        while True:
            p.stepSimulation()
            time.sleep(config.simulation.step_delay)
    except KeyboardInterrupt:
        logger.info("Simulation stopped by user.")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()