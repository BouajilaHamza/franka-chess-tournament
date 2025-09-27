import pybullet as p
import numpy as np
import logging
from simulation.motion_planners.ompl_planner import OMPLPlanner
from simulation.motion_planners.ik_planner import IKPlanner
from configs.config import config

logger = logging.getLogger(__name__)

class MotionCoordinator:
    """Coordinates movement decisions between different planners."""

    def __init__(self, all_obstacle_ids=None):
        self.all_obstacle_ids = all_obstacle_ids or set()
        # Safety distance threshold for direct path check (adjust in config)
        self.safety_dist_threshold = config.planning.safety_distance_threshold

    def _point_to_line_seg_distance(self, point, line_start, line_end):
        """
        Calculate the shortest distance between a point and a line segment.
        Args:
            point (array-like): The point (x, y, z).
            line_start (array-like): The start point of the line segment (x, y, z).
            line_end (array-like): The end point of the line segment (x, y, z).
        Returns:
            float: The shortest distance between the point and the line segment.
        """
        line_vec = np.array(line_end) - np.array(line_start)
        point_vec = np.array(point) - np.array(line_start)
        line_len_sq = np.dot(line_vec, line_vec)
        if line_len_sq == 0.0:
            return np.linalg.norm(point - line_start)
        t = max(0, min(1, np.dot(point_vec, line_vec) / line_len_sq))
        projection = np.array(line_start) + t * line_vec
        return np.linalg.norm(np.array(point) - projection)

    def is_direct_path_safe(self, robot_id, ee_index, target_pos, held_object_id=None):
        """
        Check if a direct linear path from current EE pos to target is safe.
        """
        try:
            current_ee_pos = p.getLinkState(robot_id, ee_index)[0]
            target_ee_pos = np.array(target_pos)

            trajectory_start = np.array(current_ee_pos)
            trajectory_end = target_ee_pos
            logger.debug(f"Coordinator: Checking direct path from {trajectory_start} to {trajectory_end} is safe.")
            logger.debug(f"Coordinator: All obstacles: {self.all_obstacle_ids}")
            for obstacle_id in self.all_obstacle_ids:
                if obstacle_id == held_object_id:
                    continue
                try:
                    obstacle_pos, _ = p.getBasePositionAndOrientation(obstacle_id)
                    obstacle_pos = np.array(obstacle_pos)
                    dist = self._point_to_line_seg_distance(obstacle_pos, trajectory_start, trajectory_end)
                    logger.debug(f"Coordinator: Obstacle {obstacle_id} distance: {dist:.3f}m")
                    if dist < self.safety_dist_threshold:
                        logger.info(f"Coordinator: Obstacle {obstacle_id} too close ({dist:.3f}m). Using OMPL.")
                        return False
                except p.error as e:
                    logger.info(f"Coordinator: p.error {e}")
                    continue # Obstacle might have been removed
            logger.info(f"Coordinator: No obstacles within {self.safety_dist_threshold}m. Using IK.")
            return True
        except Exception as e:
            logger.error(f"Coordinator: Error in safety check: {e}. Defaulting to OMPL.")
            return False # Default to OMPL if check fails

    def move_smartly(self, ik_planner:IKPlanner, ompl_planner:OMPLPlanner, robot_id, arm_joints, ee_index, target_pos, target_orient,
                     log_msg="", held_object_id=None):
        """
        Intelligently choose between IK and OMPL based on obstacle proximity.
        """
        # Decide based on safety check
        if self.is_direct_path_safe(robot_id, ee_index, target_pos, held_object_id):
            logger.info(f"Coordinator: Smart Move - Direct path safe, using IK. {log_msg}")
            return ik_planner.move_to_pose(robot_id, arm_joints, ee_index, target_pos, target_orient,
                                            log_msg=f"(Smart IK) {log_msg}")
        else:
            logger.info(f"Coordinator: Smart Move - Obstacle risk, using OMPL. {log_msg}")
            return ompl_planner.move_to_pose(robot_id, arm_joints, ee_index, target_pos, target_orient,
                                             log_msg=f"(Smart OMPL) {log_msg}", held_object_id=held_object_id)
