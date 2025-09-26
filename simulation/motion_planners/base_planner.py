import abc
import pybullet as p

class MotionPlanner(abc.ABC):
    """Abstract base class for motion planners."""

    @abc.abstractmethod
    def move_to_pose(self, robot_id, arm_joints, ee_index, target_pos, target_orient, **kwargs):
        """
        Move the robot's end effector to a target pose.

        Args:
            robot_id: The PyBullet body ID of the robot.
            arm_joints: List of joint indices for the arm.
            ee_index: Link index of the end effector.
            target_pos: Target position [x, y, z].
            target_orient: Target orientation [x, y, z, w] (quaternion).
            **kwargs: Additional arguments (e.g., held_object_id).

        Returns:
            bool: True if movement was successful, False otherwise.
        """
        pass

    @abc.abstractmethod
    def move_to_home(self, robot_id, arm_joints, home_position, **kwargs):
        """
        Move the robot arm to its home position.

        Args:
            robot_id: The PyBullet body ID of the robot.
            arm_joints: List of joint indices for the arm.
            home_position: List of joint angles for the home position.
            **kwargs: Additional arguments.

        Returns:
            bool: True if movement was successful, False otherwise.
        """
        pass

    def _get_current_joint_states(self, robot_id, arm_joints):
        """Helper to get current joint positions."""
        return [p.getJointState(robot_id, idx)[0] for idx in arm_joints]

    def _set_joint_positions(self, robot_id, arm_joints, target_positions, force=240.0):
        """Helper to set joint positions."""
        for i, idx in enumerate(arm_joints):
             p.setJointMotorControl2(
                 bodyIndex=robot_id,
                 jointIndex=idx,
                 controlMode=p.POSITION_CONTROL,
                 targetPosition=target_positions[i],
                 force=force,
                 maxVelocity=1.0
             )

    def _is_joint_config_valid(self, robot_id, arm_joints, joint_config):
        """Check if a joint configuration is within limits."""
        if len(joint_config) != len(arm_joints):
            return False
        for i, joint_idx in enumerate(arm_joints):
            info = p.getJointInfo(robot_id, joint_idx)
            lower_limit, upper_limit = info[8], info[9]
            if not (lower_limit <= joint_config[i] <= upper_limit):
                return False
        return True
