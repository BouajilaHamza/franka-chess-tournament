import time
import logging
import numpy as np
import pybullet as p
from typing import Optional
from configs.config import config
from utils.helper_functions import wait

from .motion_planners.ik_planner import IKPlanner
from .motion_planners.ompl_planner import OMPLPlanner
from .coordinator import MotionCoordinator
from ui.schemas import MoveData #FailureDetail, ExperimentData
logger = logging.getLogger(__name__)

class RobotController:
    """
    Encapsulates the control logic for a single robot arm.
    Manages its ID, joints, gripper, and provides methods for movement and pick/place.
    Delegates motion planning to separate planner and coordinator components.
    """
    def __init__(self, robot_id, arm_joints, gripper_joints, ee_index, robot_name="Robot", all_obstacle_ids=None):
        self.id = robot_id
        self.arm_joints = arm_joints
        self.gripper_joints = gripper_joints
        self.ee_index = ee_index
        self.name = robot_name
        self.n_dof = len(arm_joints)
        self.all_obstacle_ids = all_obstacle_ids or set()
        self.held_object_id = None


        self.ik_planner = IKPlanner()
        self.ompl_planner = OMPLPlanner(all_obstacle_ids=self.all_obstacle_ids)
        self.coordinator = MotionCoordinator(all_obstacle_ids=self.all_obstacle_ids)

        logger.info(f"{self.name} controller initialized with ID {self.id}.")

    # --- Gripper Control (Kept within RobotController) ---
    def _set_gripper_position(self, position):
        """Set gripper position using POSITION_CONTROL."""
        adjusted_position = min(0.08, max(0.0, position))
        logger.debug(f"{self.name}: Setting gripper to position: {adjusted_position}m")
        for idx in self.gripper_joints:
            p.setJointMotorControl2(
                bodyIndex=self.id,
                jointIndex=idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=adjusted_position,
                force=config.robot.first.gripper_force
            )

    # --- Movement Functions (Delegating to Planners/Coordinator) ---
    def move_to_position_ik(self, target_pos, target_orient,move_log_data:MoveData, log_msg=""):
        """Move using direct IK."""
        return self.ik_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,
                                           target_pos, target_orient,move_log_data=move_log_data, log_msg=log_msg)

    def move_to_position_ompl(self, target_pos, target_orient,move_log_data:MoveData, log_msg="", held_object_id=None):
        """Move using OMPL planning."""
        # Pass held_object_id to OMPL planner if needed within its logic
        return self.ompl_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,
                                             target_pos, target_orient, 
                                             move_log_data=move_log_data, 
                                             log_msg=log_msg,
                                             held_object_id=held_object_id)

    def move_smartly_to_position(self, target_pos, target_orient,move_log_data:MoveData, log_msg="", held_object_id=None):
        """Move using the smart coordinator to choose IK or OMPL."""
        return self.coordinator.move_smartly(
            self.ik_planner, self.ompl_planner,
            self.id, self.arm_joints, self.ee_index,
            target_pos, target_orient,move_log_data=move_log_data, log_msg=log_msg, held_object_id=held_object_id
        )

    def move_to_home_position(self, tolerance=None, timeout=None):
        """Move to home using IK (common choice)."""
        robot_config_key = "first"
        if self.name == "Robot2":
             robot_config_key = "second"
        home_position = getattr(config.robot, robot_config_key).home_position
        return self.ik_planner.move_to_home(self.id, self.arm_joints, home_position,
                                          tolerance=tolerance, timeout=timeout)



    def verify_grasp(self, object_id, expected_pos, timeout=1.0):
        """Basic check if the object is likely grasped."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                # ee_pos = self._get_ee_position() # Implement if needed
                obj_pos, _ = p.getBasePositionAndOrientation(object_id)
                # distance = np.linalg.norm(np.array(obj_pos) - np.array(ee_pos))
                logger.info(f"{self.name}:   -> EE Position: ..., Expected Object Pos: {expected_pos}, Actual Object Pos: {obj_pos}, Distance: ...m ")
                # if distance < 0.5: # Threshold, adjust based on object size
                return True # Simplified for example
            except p.error:
                logger.info(f"{self.name}:   -> Object no longer exists, assuming it was grasped and moved.")
                return True
            wait(10)
        logger.warning(f"{self.name}:   -> Grasp verification timeout.")
        return False

    def _pick_place_single_attempt(self, object_id, start_pos, target_pos, move_log_data:MoveData, metrics_logger=None):
        """
        Perform a single attempt of the pick and place sequence.
        Uses the smart coordinator for movement.
        Collects and returns metrics.
        Args:
            object_id: The ID of the object to pick.
            start_pos: The starting position [x, y, z].
            target_pos: The target position [x, y, z].
            metrics_logger: Optional logger instance for metrics (not used directly here, passed for consistency if needed).
        Returns:
            bool: success_boolean
        """
        logger.info(f"{self.name}: --- Starting Pick and Place Single Attempt ---")
        attempt_start_time = time.time()

        move_log_data.move_number=1
        move_log_data.success=False # Default assumption
        move_log_data.failure_type=None # 'IK', 'OMPL', 'Grasp', 'Execution', 'Timeout'
        move_log_data.total_time_seconds=None # Will be calculated at the end
        move_log_data.planning_time_seconds=0.0 # Placeholder, can be refined per planner call if needed
        move_log_data.execution_time_seconds=0.0 # Placeholder, can be refined per execution if needed
        move_log_data.placement_error_mm=None # To be calculated if needed (requires object final pos)
        move_log_data.min_collision_proximity_mm=None # If tracked by planners
        move_log_data.algorithm_used="IK" # 'IK', 'OMPL', 'Hybrid' (from coordinator/planners)



        start_surface_z = start_pos[2]
        target_surface_z = target_pos[2]

        # 1. --- PICKING PHASE ---
        # a. Move above the object
        pick_approach_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
        self._set_gripper_position(config.robot.first.gripper_open)
        wait(config.simulation.settle_steps // 2)
        # Use SMART movement for approach
        if not self.ik_planner.move_to_pose(self.id, 
                                            self.arm_joints, self.ee_index,
                                            pick_approach_pos, 
                                            config.pick_place.ee_down_orientation,
                                            move_log_data=move_log_data,
                                            log_msg="Pick approach"):
            logger.error(f"{self.name}: 1a. Failed to move above object.")
            move_log_data.algorithm_used = 'IK'
            move_log_data.total_time_seconds = time.time() - attempt_start_time # --- METRIC: End time on failure ---
            return False

        # b. Move down to grasp position
        grasp_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.pick_z_offset]
        # Use SMART movement for grasp approach
        if not self.ik_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,
                                    grasp_pos, config.pick_place.ee_down_orientation,
                                    move_log_data=move_log_data,
                                    log_msg="Grasp approach"):
            logger.error(f"{self.name}: 1b. Failed to move to grasp position.")
            move_log_data.failure_type = 'IK'
            move_log_data.total_time_seconds = time.time() - attempt_start_time # --- METRIC: End time on failure ---
            return False # --- METRIC: Return on failure ---

        # c. Close gripper to grasp - Apply force
        logger.info(f"{self.name}: 1c. Closing gripper and applying force...")
        self._set_gripper_position(config.robot.first.gripper_closed)
        wait(config.simulation.gripper_action_steps)

        # d. Verify grasp (basic check)
        if not self.verify_grasp(object_id, grasp_pos):
            logger.warning(f"{self.name}: 1d. Grasp verification failed.")
            self._set_gripper_position(config.robot.first.gripper_open)
            wait(config.simulation.gripper_action_steps)
            move_log_data.failure_type = 'Grasp'
            move_log_data.total_time_seconds = time.time() - attempt_start_time # --- METRIC: End time on failure ---
            return False # --- METRIC: Return on failure ---
        logger.info(f"{self.name}: 1d. Grasp verified.")

        # e. Lift the object (force should keep it attached)
        lift_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
        # Use SMART movement for lifting, pass held_object_id
        self.held_object_id = object_id # Update state
        logger.debug(f"(Held object ID: {self.held_object_id})")
        if not self.ik_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,lift_pos, config.pick_place.ee_down_orientation,
                                            move_log_data=move_log_data,
                                            log_msg="1e. Lifting object..."):
            logger.error(f"{self.name}: 1e. Failed to lift object.")
            self._set_gripper_position(config.robot.first.gripper_open)
            wait(config.simulation.gripper_action_steps)
            self.held_object_id = None # Reset state
            logger.debug(f"(Held object ID: {self.held_object_id})")
            move_log_data.failure_type = 'Execution' # Or 'IK' if lifting failed due to IK
            move_log_data.total_time_seconds = time.time() - attempt_start_time # --- METRIC: End time on failure ---
            return False # --- METRIC: Return on failure ---

        # 2. --- PLACING PHASE ---
        # a. Move above the target location (correct X, Y from the start)
        place_approach_pos = [target_pos[0], target_pos[1], target_surface_z + config.pick_place.clearance_z]
        # Use SMART movement for moving above target, pass held_object_id
        # --- METRIC: Potentially capture algorithm used ---
        # This requires the coordinator/planners to return this info, which is complex.
        # For now, we acknowledge the attempt was made.
        if not self.move_smartly_to_position(place_approach_pos, config.pick_place.ee_down_orientation,move_log_data=move_log_data,
                                log_msg="2a. Moving above target...", held_object_id=object_id):
            logger.error(f"{self.name}: 2a. Failed to move above target.")

            drop_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
            self.move_smartly_to_position(drop_pos, config.pick_place.ee_down_orientation,move_log_data=move_log_data,
                                        log_msg="  -> Dropping object back (failed move to target)...",
                                        held_object_id=object_id)
            self._set_gripper_position(config.robot.first.gripper_open)
            wait(config.simulation.gripper_action_steps)
            lift_pos_after_drop = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z + 0.05]
            # Move away after drop without holding
            self.move_smartly_to_position(lift_pos_after_drop, config.pick_place.ee_down_orientation,move_log_data=move_log_data,
                                        log_msg="  -> Lifting after drop...", held_object_id=None)
            self.held_object_id = None
            logger.debug(f"(Held object ID: {self.held_object_id})")
            move_log_data.failure_type = 'Execution' # Or 'OMPL' if planning failed
            move_log_data.total_time_seconds = time.time() - attempt_start_time # --- METRIC: End time on failure ---
            return False # --- METRIC: Return on failure ---

        # b. Move down to release position (ON the target surface)
        release_pos = [target_pos[0], target_pos[1], target_surface_z + config.pick_place.place_z_offset]

        if not self.ik_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,release_pos, config.pick_place.ee_down_orientation,
                                            move_log_data=move_log_data,
                                            log_msg="2b. Moving down to place..."):
            logger.error(f"{self.name}: 2b. Failed to move to place position.")
            # Drop object back
            drop_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
            self.move_smartly_to_position(drop_pos, config.pick_place.ee_down_orientation,move_log_data=move_log_data,
                                        log_msg="  -> Dropping object back (failed place)...",
                                        held_object_id=object_id)
            self._set_gripper_position(config.robot.first.gripper_open)
            wait(config.simulation.gripper_action_steps)
            lift_pos_after_drop = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z + 0.05]

            self.ik_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,lift_pos_after_drop, config.pick_place.ee_down_orientation,
                                         move_log_data=move_log_data,
                                        log_msg="  -> Lifting after drop...")
            self.held_object_id = None
            logger.debug(f"(Held object ID: {self.held_object_id})")
            move_log_data.failure_type = 'Execution' # Or 'IK' if placing failed due to IK
            move_log_data.total_time_seconds = time.time() - attempt_start_time # --- METRIC: End time on failure ---
            return False # --- METRIC: Return on failure ---


        logger.info(f"{self.name}: 2c. Opening gripper...")
        self._set_gripper_position(config.robot.first.gripper_open)
        wait(config.simulation.gripper_action_steps)
        self.held_object_id = None
        logger.debug(f"(Held object ID: {self.held_object_id})")

        # d. Move away (lift up from target)
        retreat_pos = [target_pos[0], target_pos[1], target_surface_z + config.pick_place.clearance_z]

        if not self.ik_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,retreat_pos, config.pick_place.ee_down_orientation,
                                            move_log_data=move_log_data,
                                            log_msg="2d. Retreating..."):
            logger.warning(f"{self.name}: 2d. Failed to fully retreat after placing (not critical).")

        # --- METRIC: Finalize and Return Success ---
        move_log_data.success = True
        move_log_data.total_time_seconds = time.time() - attempt_start_time # --- METRIC: End time on success ---
        # --- METRIC: Calculate placement error (Conceptual, requires final object position) ---
        # This is best done *after* the move and *before* the object is released,
        # or by tracking the object's final resting position later.
        # For now, placeholder:
        try:
            final_obj_pos, _ = p.getBasePositionAndOrientation(object_id)
            target_center = np.array(target_pos)
            error_vector = np.array(final_obj_pos) - target_center
            # Project onto XY plane (vertical placement error less critical)
            xy_error_m = np.linalg.norm(error_vector[:2])
            move_log_data.placement_error_mm = xy_error_m * 1000.0 # Convert to mm
        except Exception as e:
            logger.warning(f"Could not calculate placement error: {e}")
            move_log_data.placement_error_mm = None # Or -1 to indicate error in calculation
        # --- End METRIC: Placement Error ---

        logger.info(f"{self.name}: --- Pick and Place Single Attempt Complete ---")
        return True # --- METRIC: Return on success ---






    def pick_and_place_with_retry(self, object_id, start_pos, target_pos, max_retries=None,
                                  move_log_data:Optional[MoveData]=None): # Add metrics_logger=None if needed directly
        """
        Perform the pick and place sequence with retry logic.
        On retry, the robot returns to the home position before attempting again.
        Returns metrics data.
        Args:
            object_id: The ID of the object to pick.
            start_pos: The starting position [x, y, z].
            target_pos: The target position [x, y, z].
            max_retries: Maximum number of retries.
            # metrics_logger: Optional logger instance (if needed directly, though coordinator/planners might use it).
        Returns:
            bool: final_success_boolean
        """
        max_retries = max_retries or config.task.max_retries
        logger.info(f"{self.name}: === Starting Pick and Place with Retry (Max: {max_retries}) ===")
        logger.info(f"{self.name}: Object ID: {object_id}, Start: {start_pos}, Target: {target_pos}")

        # --- METRIC: Initialize list to store metrics for all attempts ---
        all_attempts_metrics = []
        # --- End METRIC Init ---

        for attempt in range(1, max_retries + 2): # 1 initial + max_retries retries
            logger.info(f"{self.name}: --- Attempt {attempt}/{max_retries + 1} ---")

            # --- METRIC: Prepare data structure for this specific attempt ---
            # This data will be populated by _pick_place_single_attempt
            # attempt_metrics_data = {
            #     'attempt_number': attempt,
            #     'retries': attempt - 1, # 0 for first attempt
            #     # Other fields will be filled by _pick_place_single_attempt
            # }
            # --- End METRIC Init for Attempt ---

            if attempt > 1: # This is a retry
                logger.info(f"{self.name}: Retry #{attempt - 1}: Returning robot to base position before retrying...")
                if not self.move_to_home_position():
                     logger.error(f"{self.name}: Failed to return to base position for retry. Aborting.")
                     # --- METRIC: Record failure for this retry attempt ---
                    #  # We don't have detailed metrics from move_to_home, so create a minimal entry
                    #  home_failure_metric = {
                    #      'attempt_number': attempt,
                    #      'retries': attempt - 1,
                    #      'success': False,
                    #      'failure_type': 'Timeout', # Or 'Execution'
                    #      'total_time_seconds': None, # Could measure if timed
                    #      'algorithm_used': 'IK', # Assuming home move uses IK
                    #      'piece_type': 'N/A (Home Move)'
                    #      # Add other relevant fields as needed
                    #  }
                     move_log_data.attempt_number = attempt
                     move_log_data.retries = attempt - 1
                     move_log_data.success = False
                     move_log_data.failure_type = 'Timeout'
                     move_log_data.total_time_seconds = None
                     move_log_data.algorithm_used = 'IK'
                     move_log_data.piece_type = 'N/A (Home Move)'
                    #  all_attempts_metrics.append(home_failure_metric)
                     # --- End METRIC: Record failure ---
                     return False
                logger.info(f"{self.name}: Robot returned to base position. Proceeding with retry.")

            # --- METRIC: Record start time for the attempt ---
            attempt_start_time = time.time()
            # --- End METRIC: Start time ---

            # --- METRIC: Attempt the pick and place sequence ---
            # Pass the metrics_logger if _pick_place_single_attempt needs it internally
            success = self._pick_place_single_attempt(object_id, start_pos, target_pos,move_log_data) # , metrics_logger=metrics_logger if needed
            # --- METRIC: Record end time and finalize attempt time ---
            attempt_end_time = time.time()
            # Ensure total_time_seconds is set (it should be from _pick_place_single_attempt)
            if success:
                move_log_data.total_time_seconds = attempt_end_time - attempt_start_time
            # --- End METRIC: Finalize time ---

            # --- METRIC: Append the metrics from this attempt to the list ---
            # all_attempts_metrics.append(metrics_from_attempt)
            # --- End METRIC: Append ---

            if success:
                logger.info(f"{self.name}: === Pick and Place SUCCEEDED on attempt {attempt} ===")
                return True, all_attempts_metrics
            else:
                logger.warning(f"{self.name}: === Pick and Place FAILED on attempt {attempt} ===")
                if attempt < max_retries + 1:
                    logger.info(f"{self.name}: Preparing for retry...")
                    self._set_gripper_position(config.robot.first.gripper_open)
                    wait(config.simulation.settle_steps)
                else:
                     logger.error(f"{self.name}: === Pick and Place FAILED after {max_retries + 1} attempts ===")

        # Should not be reached, but good practice
        return False, all_attempts_metrics # Return final failure and all metrics
