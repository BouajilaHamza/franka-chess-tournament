import pybullet as p
import numpy as np
from configs.config import config


class MotionCoordinator:
    """Coordinates movement decisions between different planners."""

    def __init__(self, all_obstacle_ids=None):
        self.all_obstacle_ids = all_obstacle_ids or set()
        # Safety distance threshold for direct path check (adjust in config)
        self.safety_dist_threshold = config.planning.safety_distance_threshold

    def _point_to_line_seg_distance(self, point, line_start, line_end):
        """Calculate the shortest distance between a point and a line segment."""
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

            for obstacle_id in self.all_obstacle_ids:
                if obstacle_id == held_object_id:
                    continue
                try:
                    obstacle_pos, _ = p.getBasePositionAndOrientation(obstacle_id)
                    obstacle_pos = np.array(obstacle_pos)
                    dist = self._point_to_line_seg_distance(obstacle_pos, trajectory_start, trajectory_end)
                    if dist < self.safety_dist_threshold:
                        print(f"Coordinator: Obstacle {obstacle_id} too close ({dist:.3f}m). Using OMPL.") # Use logger
                        return False
                except p.error:
                    continue # Obstacle might have been removed
            print(f"Coordinator: No obstacles within {self.safety_dist_threshold}m. Using IK.") # Use logger
            return True
        except Exception as e:
            print(f"Coordinator: Error in safety check: {e}. Defaulting to OMPL.") # Use logger
            return False # Default to OMPL if check fails

    def move_smartly(self, ik_planner, ompl_planner, robot_id, arm_joints, ee_index, target_pos, target_orient,
                     log_msg="", held_object_id=None):
        """
        Intelligently choose between IK and OMPL based on obstacle proximity.
        """
        # Decide based on safety check
        if self.is_direct_path_safe(robot_id, ee_index, target_pos, held_object_id):
            print(f"Coordinator: Smart Move - Direct path safe, using IK. {log_msg}") # Use logger
            return ik_planner.move_to_pose(robot_id, arm_joints, ee_index, target_pos, target_orient,
                                            log_msg=f"(Smart IK) {log_msg}")
        else:
            print(f"Coordinator: Smart Move - Obstacle risk, using OMPL. {log_msg}") # Use logger
            return ompl_planner.move_to_pose(robot_id, arm_joints, ee_index, target_pos, target_orient,
                                             log_msg=f"(Smart OMPL) {log_msg}", held_object_id=held_object_id)
