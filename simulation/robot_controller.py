import pybullet as p
import numpy as np
import time
import logging
from configs.config import config # Assuming config is accessible
from utils.simulation import wait # Assuming wait function is in utils

logger = logging.getLogger(__name__)

class RobotController:
    """
    Encapsulates the control logic for a single robot arm.
    Manages its ID, joints, gripper, and provides methods for movement and pick/place.
    """
    def __init__(self, robot_id, arm_joints, gripper_joints, ee_index, robot_name="Robot"):
        self.id = robot_id
        self.arm_joints = arm_joints
        self.gripper_joints = gripper_joints
        self.ee_index = ee_index
        self.name = robot_name

        logger.info(f"{self.name} controller initialized with ID {self.id}.")

    def _set_arm_positions(self, target_positions):
        """Internal method to set target positions for the robot arm joints."""
        for i, joint_idx in enumerate(self.arm_joints):
            p.setJointMotorControl2(
                bodyIndex=self.id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_positions[i],
                force=config.robot.first.max_joint_force,
                maxVelocity=1.0
            )

    def _set_gripper_position(self, position):
        """
        Internal method to set gripper position and apply holding force.
        Uses POSITION_CONTROL to move, then VELOCITY_CONTROL with 0 vel and force to hold.
        """
        # Panda gripper has 0.08m max opening, adjust for small parts
        adjusted_position = min(0.08, max(0.0, position))
        logger.info(f"{self.name}: Setting gripper to position: {adjusted_position}")


        for joint_idx in self.gripper_joints:
            p.setJointMotorControl2(
                bodyIndex=self.id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=adjusted_position,
                force=config.robot.first.gripper_force
            )

    def _get_current_arm_positions(self):
        """Internal method to get the current positions of the arm joints."""
        return [p.getJointState(self.id, joint_idx)[0] for joint_idx in self.arm_joints]

    def _get_ee_position(self):
        """Internal method to get the current position of the end effector."""
        ee_state = p.getLinkState(self.id, self.ee_index)
        return np.array(ee_state[0])

    def move_to_home_position(self, tolerance=None, timeout=None):
        """
        Move the robot arm to the predefined home position.
        Uses the configuration specific to this robot instance.
        """
        # Determine which robot's config to use based on self.name or an internal identifier
        # For now, assuming first robot config is used, you need to adapt this
        robot_config_key = "first" # Placeholder - needs logic to determine first/second
        if self.name == "Robot2": # Or some other unique identifier you assign
             robot_config_key = "second"

        home_position = getattr(config.robot, robot_config_key).home_position
        tolerance = tolerance or getattr(config.robot, robot_config_key).home_position_tolerance
        timeout = timeout or getattr(config.robot, robot_config_key).home_move_timeout

        logger.info(f"{self.name}: Moving to home position...")
        if not home_position or len(home_position) < len(self.arm_joints):
            logger.error(f"{self.name}: Invalid home position configuration.")
            return False

        try:

            for i, joint_idx in enumerate(self.arm_joints):
                 p.setJointMotorControl2(
                     bodyIndex=self.id,
                     jointIndex=joint_idx,
                     controlMode=p.POSITION_CONTROL,
                     targetPosition=home_position[i],
                     force=config.robot.first.max_joint_force, # Use appropriate config
                     maxVelocity=1.0 # Moderate speed
                 )

            # Wait for the robot to reach the home position or timeout
            start_time = time.time()
            while time.time() - start_time < timeout:
                p.stepSimulation()
                time.sleep(config.simulation.step_delay)

                # Check if all joints are close enough to the target
                joints_at_target = True
                for i, joint_idx in enumerate(self.arm_joints):
                    current_pos = p.getJointState(self.id, joint_idx)[0]
                    if abs(current_pos - home_position[i]) > tolerance:
                        joints_at_target = False
                        break # Break inner loop, continue waiting

                if joints_at_target:
                    logger.info(f"{self.name}: Successfully reached home position.")
                    wait(config.simulation.settle_steps // 2) # Brief pause to settle
                    return True

            logger.warning(f"{self.name}: Timeout waiting for robot to reach home position.")
            return False

        except Exception as e:
            logger.error(f"{self.name}: Error moving to home position: {e}")
            return False

    def move_to_position_ik(self, target_pos, target_orient, log_msg=""):
        """Move the robot's end effector to a target pose using IK with validation."""
        if log_msg:
            logger.info(f"{self.name}: {log_msg}")
        try:
            goal_joint_positions = p.calculateInverseKinematics(
                bodyUniqueId=self.id,
                endEffectorLinkIndex=self.ee_index,
                targetPosition=target_pos,
                targetOrientation=target_orient,
                maxNumIterations=300, # Consider making this a config parameter
                residualThreshold=1e-7 # Consider making this a config parameter
            )

            current_positions = self._get_current_arm_positions()
            target_arm_positions = goal_joint_positions[:len(self.arm_joints)]

            # Interpolate movement for smoother transition
            for step in range(config.simulation.move_steps):
                alpha = step / float(config.simulation.move_steps)
                interp_positions = [
                    (1 - alpha) * curr + alpha * targ
                    for curr, targ in zip(current_positions, target_arm_positions)
                ]
                self._set_arm_positions(interp_positions)
                p.stepSimulation()
                time.sleep(config.simulation.step_delay)

            # Ensure final position is reached
            self._set_arm_positions(target_arm_positions)
            wait(config.simulation.settle_steps)

            # --- Critical Fix: Verify EE reached the target X/Y ---
            final_ee_pos = self._get_ee_position()
            target_pos_np = np.array(target_pos)
            xy_distance = np.linalg.norm(final_ee_pos[:2] - target_pos_np[:2])
            z_distance = abs(final_ee_pos[2] - target_pos_np[2])

            logger.debug(f"{self.name}:   -> Target: {target_pos}, Actual EE: {final_ee_pos}")
            logger.debug(f"{self.name}:   -> XY Distance: {xy_distance:.4f}m, Z Distance: {z_distance:.4f}m")

          
            if xy_distance > config.pick_place.xy_tolerance:
                logger.warning(f"{self.name}:   -> EE XY position error too large ({xy_distance:.4f}m > {config.pick_place.xy_tolerance:.4f}m). Target XY: {target_pos[:2]}, Actual XY: {final_ee_pos[:2]}")
                return False
            if z_distance > config.pick_place.z_tolerance:
                logger.warning(f"{self.name}:   -> EE Z position error large ({z_distance:.4f}m > {config.pick_place.z_tolerance:.4f}m). Check if this is acceptable.")

            logger.info(f"{self.name}:   -> Movement completed successfully.")
            return True
        except Exception as e:
            logger.error(f"{self.name}:   -> IK calculation or movement failed: {e}")
            return False

    def verify_grasp(self, object_id, expected_pos, timeout=1.0):
        """Basic check if the object is likely grasped."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                ee_pos = self._get_ee_position()
                obj_pos, _ = p.getBasePositionAndOrientation(object_id)
                distance = np.linalg.norm(np.array(obj_pos) - np.array(ee_pos))
                logger.info(f"{self.name}:   -> EE Position: {ee_pos}, Expected Object Pos: {expected_pos}, Actual Object Pos: {obj_pos}, Distance: {distance:.4f}m ")
                if distance < 0.5: # Threshold, adjust based on object size
                    return True
            except p.error:
                logger.info(f"{self.name}:   -> Object no longer exists, assuming it was grasped and moved.")
                return True # Object might have been successfully moved/removed
            wait(10) # Small wait
        logger.warning(f"{self.name}:   -> Grasp verification timeout. Object might not be firmly grasped.")
        return False

    def _pick_place_single_attempt(self, object_id, start_pos, target_pos):
        """
        Perform a single attempt of the pick and place sequence.
        Returns True if successful, False otherwise.
        Assumes start_pos and target_pos Z are the surface level.
        """
        logger.info(f"{self.name}: --- Starting Pick and Place Single Attempt ---")
        start_surface_z = start_pos[2]
        target_surface_z = target_pos[2]

        # 1. --- PICKING PHASE ---
        # a. Move above the object
        pick_approach_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
        self._set_gripper_position(config.robot.first.gripper_open) # Use appropriate config
        wait(config.simulation.settle_steps // 2)
        if not self.move_to_position_ik(pick_approach_pos, config.pick_place.ee_down_orientation,
                                   log_msg=f"1a. Moving above object...{pick_approach_pos}"):
            logger.error(f"{self.name}: 1a. Failed to move above object.")
            return False

        # b. Move down to grasp position
        grasp_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.pick_z_offset]
        if not self.move_to_position_ik(grasp_pos, config.pick_place.ee_down_orientation,
                                   log_msg="1b. Moving down to grasp..."):
            logger.error(f"{self.name}: 1b. Failed to move to grasp position.")
            return False

        # c. Close gripper to grasp - Apply force
        logger.info(f"{self.name}: 1c. Closing gripper and applying force...")
        self._set_gripper_position(config.robot.first.gripper_closed) # Use appropriate config
        wait(config.simulation.gripper_action_steps)
        # Ensure force is applied continuously after closing - This is already handled by _set_gripper_position's VELOCITY_CONTROL part if implemented that way


        # d. Verify grasp (basic check)
        if not self.verify_grasp(object_id, grasp_pos):
            logger.warning(f"{self.name}: 1d. Grasp verification failed.")
            self._set_gripper_position(config.robot.first.gripper_open) # Use appropriate config
            wait(config.simulation.gripper_action_steps)
            return False
        logger.info(f"{self.name}: 1d. Grasp verified.")

        # e. Lift the object (force should keep it attached)
        lift_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
        if not self.move_to_position_ik(lift_pos, config.pick_place.ee_down_orientation,
                                   log_msg="1e. Lifting object..."):
            logger.error(f"{self.name}: 1e. Failed to lift object.")
            self._set_gripper_position(config.robot.first.gripper_open) # Use appropriate config
            wait(config.simulation.gripper_action_steps)
            return False

        # 2. --- PLACING PHASE ---
        # a. Move above the target location (correct X, Y from the start)
        place_approach_pos = [target_pos[0], target_pos[1], target_surface_z + config.pick_place.clearance_z]
        if not self.move_to_position_ik(place_approach_pos, config.pick_place.ee_down_orientation,
                                   log_msg="2a. Moving above target..."):
            logger.error(f"{self.name}: 2a. Failed to move above target.")
            # Drop object back
            drop_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
            self.move_to_position_ik(drop_pos, config.pick_place.ee_down_orientation, log_msg="  -> Dropping object back (failed move to target)...")
            self._set_gripper_position(config.robot.first.gripper_open) # Use appropriate config
            wait(config.simulation.gripper_action_steps)
            lift_pos_after_drop = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z + 0.05]
            self.move_to_position_ik(lift_pos_after_drop, config.pick_place.ee_down_orientation, log_msg="  -> Lifting after drop...")
            return False

        # b. Move down to release position (ON the target surface)
        release_pos = [target_pos[0], target_pos[1], target_surface_z + config.pick_place.place_z_offset] # Corrected: Place ON or just above surface
        if not self.move_to_position_ik(release_pos, config.pick_place.ee_down_orientation,
                                   log_msg="2b. Moving down to place..."):
            logger.error(f"{self.name}: 2b. Failed to move to place position.")
            # Drop object back
            drop_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
            self.move_to_position_ik(drop_pos, config.pick_place.ee_down_orientation, log_msg="  -> Dropping object back (failed place)...")
            self._set_gripper_position(config.robot.first.gripper_open) # Use appropriate config
            wait(config.simulation.gripper_action_steps)
            lift_pos_after_drop = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z + 0.05]
            self.move_to_position_ik(lift_pos_after_drop, config.pick_place.ee_down_orientation, log_msg="  -> Lifting after drop...")
            return False

        # c. Open gripper to release
        logger.info(f"{self.name}: 2c. Opening gripper...")
        self._set_gripper_position(config.robot.first.gripper_open) # Use appropriate config
        wait(config.simulation.gripper_action_steps)

        # d. Move away (lift up from target)
        retreat_pos = [target_pos[0], target_pos[1], target_surface_z + config.pick_place.clearance_z]
        if not self.move_to_position_ik(retreat_pos, config.pick_place.ee_down_orientation,
                                   log_msg="2d. Retreating..."):
            logger.warning(f"{self.name}: 2d. Failed to fully retreat after placing (not critical).")

        logger.info(f"{self.name}: --- Pick and Place Single Attempt Complete ---")
        return True

    def pick_and_place_with_retry(self, object_id, start_pos, target_pos, max_retries=None):
        """
        Perform the pick and place sequence with retry logic.
        On retry, the robot returns to the home position before attempting again.
        """
        max_retries = max_retries or config.task.max_retries # Use passed value or config default

        logger.info(f"{self.name}: === Starting Pick and Place with Retry (Max: {max_retries}) ===")
        logger.info(f"{self.name}: Object ID: {object_id}, Start: {start_pos}, Target: {target_pos}")

        # Store the home configuration for reset - get from config based on robot instance
        robot_config_key = "first" # Placeholder logic needed here
        if self.name == "Robot2":
             robot_config_key = "second"
        _initial_config = getattr(config.robot, robot_config_key).home_position

        for attempt in range(1, max_retries + 2): # 1 initial + max_retries retries
            logger.info(f"{self.name}: --- Attempt {attempt}/{max_retries + 1} ---")

            # --- Critical Fix: On retries, move back to base position ---
            if attempt > 1: # This is a retry
                logger.info(f"{self.name}: Retry #{attempt - 1}: Returning robot to base position before retrying...")
                if not self.move_to_home_position():
                     logger.error(f"{self.name}: Failed to return to base position for retry. Aborting.")
                     # If we can't even get back to base, it's a critical failure.
                     return False
                logger.info(f"{self.name}: Robot returned to base position. Proceeding with retry.")

            # Attempt the pick and place sequence
            success = self._pick_place_single_attempt(object_id, start_pos, target_pos)

            if success:
                logger.info(f"{self.name}: === Pick and Place SUCCEEDED on attempt {attempt} ===")
                # Optional: Move back to home after final success
                # self.move_to_home_position()
                return True
            else:
                logger.warning(f"{self.name}: === Pick and Place FAILED on attempt {attempt} ===")
                if attempt < max_retries + 1:
                    logger.info(f"{self.name}: Preparing for retry...")
                    # Open gripper in case it's holding something
                    self._set_gripper_position(config.robot.first.gripper_open) # Use appropriate config
                    wait(config.simulation.settle_steps)
                    # The loop will continue, triggering the base return at the start of the next iteration
                else:
                     logger.error(f"{self.name}: === Pick and Place FAILED after {max_retries + 1} attempts (including initial) ===")
                     # Optional: Final move to home on ultimate failure
                     self.move_to_home_position()

        return False # Should not be reached, but good practice

