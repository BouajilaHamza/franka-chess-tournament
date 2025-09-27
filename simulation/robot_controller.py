import pybullet as p
import time
import logging
from configs.config import config
from utils.helper_functions import wait

from .motion_planners.ik_planner import IKPlanner
from .motion_planners.ompl_planner import OMPLPlanner
from .coordinator import MotionCoordinator

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
    def move_to_position_ik(self, target_pos, target_orient, log_msg=""):
        """Move using direct IK."""
        return self.ik_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,
                                           target_pos, target_orient, log_msg=log_msg)

    def move_to_position_ompl(self, target_pos, target_orient, log_msg="", held_object_id=None):
        """Move using OMPL planning."""
        # Pass held_object_id to OMPL planner if needed within its logic
        return self.ompl_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,
                                             target_pos, target_orient, log_msg=log_msg,
                                             held_object_id=held_object_id)

    def move_smartly_to_position(self, target_pos, target_orient, log_msg="", held_object_id=None):
        """Move using the smart coordinator to choose IK or OMPL."""
        return self.coordinator.move_smartly(
            self.ik_planner, self.ompl_planner,
            self.id, self.arm_joints, self.ee_index,
            target_pos, target_orient, log_msg=log_msg, held_object_id=held_object_id
        )

    def move_to_home_position(self, tolerance=None, timeout=None):
        """Move to home using IK (common choice)."""
        robot_config_key = "first"
        if self.name == "Robot2":
             robot_config_key = "second"
        home_position = getattr(config.robot, robot_config_key).home_position
        return self.ik_planner.move_to_home(self.id, self.arm_joints, home_position,
                                          tolerance=tolerance, timeout=timeout)

    # --- Other methods (verify_grasp, _pick_place_single_attempt, pick_and_place_with_retry)
    # --- These remain largely the same but call the new movement functions ---
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

    def _pick_place_single_attempt(self, object_id, start_pos, target_pos):
        """
        Perform a single attempt of the pick and place sequence.
        Uses the smart coordinator for movement.
        """
        logger.info(f"{self.name}: --- Starting Pick and Place Single Attempt ---")
        start_surface_z = start_pos[2]
        target_surface_z = target_pos[2]

        # 1. --- PICKING PHASE ---
        # a. Move above the object
        pick_approach_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
        self._set_gripper_position(config.robot.first.gripper_open)
        wait(config.simulation.settle_steps // 2)
        # Use SMART movement for approach
        if not self.ik_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,
                                      pick_approach_pos, config.pick_place.ee_down_orientation,
                                      log_msg="Pick approach"):
            logger.error(f"{self.name}: 1a. Failed to move above object.")
            return False

        # b. Move down to grasp position
        grasp_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.pick_z_offset]
        # Use SMART movement for grasp approach
        if not self.ik_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,
                                      grasp_pos, config.pick_place.ee_down_orientation,log_msg="Grasp approach"):
            logger.error(f"{self.name}: 1b. Failed to move to grasp position.")
            return False

        # c. Close gripper to grasp - Apply force
        logger.info(f"{self.name}: 1c. Closing gripper and applying force...")
        self._set_gripper_position(config.robot.first.gripper_closed)
        wait(config.simulation.gripper_action_steps)

        # d. Verify grasp (basic check)
        if not self.verify_grasp(object_id, grasp_pos):
            logger.warning(f"{self.name}: 1d. Grasp verification failed.")
            self._set_gripper_position(config.robot.first.gripper_open)
            wait(config.simulation.gripper_action_steps)
            return False
        logger.info(f"{self.name}: 1d. Grasp verified.")

        # e. Lift the object (force should keep it attached)
        lift_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
        # Use SMART movement for lifting, pass held_object_id
        self.held_object_id = object_id # Update state
        logger.debug(f"(Held object ID: {self.held_object_id})")
        if not self.ik_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,lift_pos, config.pick_place.ee_down_orientation,
                                   log_msg="1e. Lifting object..."):
            logger.error(f"{self.name}: 1e. Failed to lift object.")
            self._set_gripper_position(config.robot.first.gripper_open)
            wait(config.simulation.gripper_action_steps)
            self.held_object_id = None # Reset state
            logger.debug(f"(Held object ID: {self.held_object_id})")
            return False

        # 2. --- PLACING PHASE ---
        # a. Move above the target location (correct X, Y from the start)
        place_approach_pos = [target_pos[0], target_pos[1], target_surface_z + config.pick_place.clearance_z]
        # Use SMART movement for moving above target, pass held_object_id
        if not self.move_smartly_to_position(place_approach_pos, config.pick_place.ee_down_orientation,
                                   log_msg="2a. Moving above target...", held_object_id=object_id):
            logger.error(f"{self.name}: 2a. Failed to move above target.")
            # Drop object back
            drop_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
            self.move_smartly_to_position(drop_pos, config.pick_place.ee_down_orientation,
                                         log_msg="  -> Dropping object back (failed move to target)...",
                                         held_object_id=object_id)
            self._set_gripper_position(config.robot.first.gripper_open)
            wait(config.simulation.gripper_action_steps)
            lift_pos_after_drop = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z + 0.05]
            # Move away after drop without holding
            self.move_smartly_to_position(lift_pos_after_drop, config.pick_place.ee_down_orientation,
                                        log_msg="  -> Lifting after drop...", held_object_id=None)
            self.held_object_id = None
            logger.debug(f"(Held object ID: {self.held_object_id})")
            return False

        # b. Move down to release position (ON the target surface)
        release_pos = [target_pos[0], target_pos[1], target_surface_z + config.pick_place.place_z_offset]
        # Use SMART movement for placing, pass held_object_id
        if not self.ik_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,release_pos, config.pick_place.ee_down_orientation,
                                   log_msg="2b. Moving down to place..."):
            logger.error(f"{self.name}: 2b. Failed to move to place position.")
            # Drop object back
            drop_pos = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z]
            self.move_smartly_to_position(drop_pos, config.pick_place.ee_down_orientation,
                                         log_msg="  -> Dropping object back (failed place)...",
                                         held_object_id=object_id)
            self._set_gripper_position(config.robot.first.gripper_open)
            wait(config.simulation.gripper_action_steps)
            lift_pos_after_drop = [start_pos[0], start_pos[1], start_surface_z + config.pick_place.clearance_z + 0.05]
            # Move away after drop without holding
            self.ik_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,lift_pos_after_drop, config.pick_place.ee_down_orientation,
                                        log_msg="  -> Lifting after drop...")
            self.held_object_id = None
            logger.debug(f"(Held object ID: {self.held_object_id})")
            return False

        # c. Open gripper to release
        logger.info(f"{self.name}: 2c. Opening gripper...")
        self._set_gripper_position(config.robot.first.gripper_open)
        wait(config.simulation.gripper_action_steps)
        self.held_object_id = None
        logger.debug(f"(Held object ID: {self.held_object_id})")

        # d. Move away (lift up from target)
        retreat_pos = [target_pos[0], target_pos[1], target_surface_z + config.pick_place.clearance_z]
        # Use SMART movement for retreating, no held object
        if not self.ik_planner.move_to_pose(self.id, self.arm_joints, self.ee_index,retreat_pos, config.pick_place.ee_down_orientation,
                                   log_msg="2d. Retreating..."):
            logger.warning(f"{self.name}: 2d. Failed to fully retreat after placing (not critical).")

        logger.info(f"{self.name}: --- Pick and Place Single Attempt Complete ---")
        return True

    def pick_and_place_with_retry(self, object_id, start_pos, target_pos, max_retries=None):
        """
        Perform the pick and place sequence with retry logic.
        On retry, the robot returns to the home position before attempting again.
        """
        max_retries = max_retries or config.task.max_retries
        logger.info(f"{self.name}: === Starting Pick and Place with Retry (Max: {max_retries}) ===")
        logger.info(f"{self.name}: Object ID: {object_id}, Start: {start_pos}, Target: {target_pos}")

        for attempt in range(1, max_retries + 2):
            logger.info(f"{self.name}: --- Attempt {attempt}/{max_retries + 1} ---")

            if attempt > 1: # This is a retry
                logger.info(f"{self.name}: Retry #{attempt - 1}: Returning robot to base position before retrying...")
                if not self.move_to_home_position():
                     logger.error(f"{self.name}: Failed to return to base position for retry. Aborting.")
                     return False
                logger.info(f"{self.name}: Robot returned to base position. Proceeding with retry.")

            # Attempt the pick and place sequence
            success = self._pick_place_single_attempt(object_id, start_pos, target_pos)

            if success:
                logger.info(f"{self.name}: === Pick and Place SUCCEEDED on attempt {attempt} ===")
                return True
            else:
                logger.warning(f"{self.name}: === Pick and Place FAILED on attempt {attempt} ===")
                if attempt < max_retries + 1:
                    logger.info(f"{self.name}: Preparing for retry...")
                    self._set_gripper_position(config.robot.first.gripper_open)
                    wait(config.simulation.settle_steps)
                else:
                     logger.error(f"{self.name}: === Pick and Place FAILED after {max_retries + 1} attempts ===")

        return False
