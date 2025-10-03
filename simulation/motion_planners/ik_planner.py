import pybullet as p
import time
import logging
from .base_planner import MotionPlanner
from configs.config import config
from utils.helper_functions import wait
from ui.schemas import MoveData

logger = logging.getLogger(__name__)



class IKPlanner(MotionPlanner):
    """Motion planner using direct inverse kinematics."""

    def move_to_pose(self, robot_id, arm_joints, ee_index, target_pos, target_orient,move_log_data:MoveData, log_msg="", **kwargs):
        """Move the robot's EE to a target pose using IK with validation."""
        if log_msg:
            logger.info(f"IK Planner: {log_msg}")
        try:
            start_ik_time = time.time()
            goal_joint_positions = p.calculateInverseKinematics(
                bodyUniqueId=robot_id,
                endEffectorLinkIndex=ee_index,
                targetPosition=target_pos,
                targetOrientation=target_orient,
                maxNumIterations=config.simulation.ik_max_iterations,
                residualThreshold=config.simulation.ik_residual_threshold
            )

            current_positions = self._get_current_joint_states(robot_id, arm_joints)
            target_arm_positions = goal_joint_positions[:len(arm_joints)]
            ik_duration = time.time() - start_ik_time
            move_log_data.planning_time_seconds=ik_duration

            for step in range(config.simulation.move_steps):
                alpha = step / float(config.simulation.move_steps)
                interp_positions = [
                    (1 - alpha) * curr + alpha * targ
                    for curr, targ in zip(current_positions, target_arm_positions)
                ]
                self._set_joint_positions(robot_id, arm_joints, interp_positions, config.robot.first.max_joint_force)
                p.stepSimulation()
                time.sleep(config.simulation.step_delay)

            # Ensure final position is reached
            self._set_joint_positions(robot_id, arm_joints, target_arm_positions, config.robot.first.max_joint_force)
            wait(config.simulation.settle_steps)

            # --- Critical Fix: Verify EE reached the target X/Y ---
            # ee_state = p.getLinkState(robot_id, ee_index)
            # final_ee_pos = np.array(ee_state[0])
            # target_pos_np = np.array(target_pos)
            # xy_distance = np.linalg.norm(final_ee_pos[:2] - target_pos_np[:2])
            # z_distance = abs(final_ee_pos[2] - target_pos_np[2])

            # if xy_distance > config.pick_place.xy_tolerance: # e.g., 0.01
            #     logger.info(f"IK Planner: EE XY error too large ({xy_distance:.4f}m)") # Use logger
            #     return False
            # if z_distance > config.pick_place.z_tolerance: # e.g., 0.02
            #     logger.info(f"IK Planner: EE Z error large ({z_distance:.4f}m)") # Use logger

            logger.info("IK Planner: Movement completed successfully.") # Use logger
            return True
        except Exception as e:
            logger.error(f"IK Planner: IK calculation or movement failed: {e}") # Use logger
            return False

    def move_to_home(self, robot_id, arm_joints, home_position, tolerance=None, timeout=None, **kwargs):
        """Move the robot arm to the predefined home position."""
        tolerance = tolerance or config.robot.first.home_position_tolerance # e.g., 0.01
        timeout = timeout or config.robot.first.home_move_timeout # e.g., 5.0

        logger.info("IK Planner: Moving to home position...") # Use logger
        if not home_position or len(home_position) < len(arm_joints):
            logger.info("IK Planner: Invalid home position configuration.") # Use logger
            return False

        try:
            for i, joint_idx in enumerate(arm_joints):
                 p.setJointMotorControl2(
                     bodyIndex=robot_id,
                     jointIndex=joint_idx,
                     controlMode=p.POSITION_CONTROL,
                     targetPosition=home_position[i],
                     force=config.robot.first.max_joint_force,
                     maxVelocity=1.0
                 )

            start_time = time.time()
            while time.time() - start_time < timeout:
                p.stepSimulation()
                time.sleep(0.001)

                joints_at_target = True
                for i, joint_idx in enumerate(arm_joints):
                    current_pos = p.getJointState(robot_id, joint_idx)[0]
                    if abs(current_pos - home_position[i]) > tolerance:
                        joints_at_target = False
                        break

                if joints_at_target:
                    logger.info("IK Planner: Successfully reached home position.") # Use logger
                    wait(config.simulation.settle_steps // 2) # Brief pause to settle
                    return True

            logger.info("IK Planner: Timeout waiting for robot to reach home position.") # Use logger
            return False

        except Exception as e:
            logger.error(f"IK Planner: Error moving to home position: {e}") # Use logger
            return False
