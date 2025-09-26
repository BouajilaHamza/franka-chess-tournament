# simulation/motion_planners/ompl_planner.py
import pybullet as p
import time
from ompl import base as ob
from ompl import geometric as og
from .base_planner import MotionPlanner
from .ik_planner import IKPlanner
from configs.config import config
from utils.simulation import wait

class OMPLPlanner(MotionPlanner):
    """Motion planner using OMPL for collision-free path planning."""

    def __init__(self, all_obstacle_ids=None):
        self.all_obstacle_ids = all_obstacle_ids or set()
        self.path_visualizer = PathVisualizer() # Optional: separate class for visualization

    class _PandaValidityChecker(ob.StateValidityChecker):
        """OMPL validity checker for the robot in the PyBullet environment."""
        def __init__(self, space_info, robot_id, arm_joints, n_dof, all_obstacle_ids):
            super().__init__(space_info)
            self.si_ = space_info
            self.robot_id = robot_id
            self.arm_joints = arm_joints
            self.n_dof = n_dof
            self.all_obstacle_ids = all_obstacle_ids
            self.held_object_id = None
            self.lower_limits, self.upper_limits = self._get_joint_limits_internal()

        def _get_joint_limits_internal(self):
            lower_limits = []
            upper_limits = []
            for idx in self.arm_joints:
                info = p.getJointInfo(self.robot_id, idx)
                lower_limits.append(info[8])
                upper_limits.append(info[9])
            return lower_limits, upper_limits

        def isValid(self, state):
            joint_positions = [state[i] for i in range(self.n_dof)]

            # Check joint limits
            for i in range(self.n_dof):
                if not (self.lower_limits[i] <= joint_positions[i] <= self.upper_limits[i]):
                    return False

            # Check for collisions
            saved_states = [p.getJointState(self.robot_id, idx)[0] for idx in self.arm_joints]
            try:
                for i, idx in enumerate(self.arm_joints):
                    p.resetJointState(self.robot_id, idx, joint_positions[i])

                # Check robot body collisions (excluding ground and held object)
                contacts_robot = p.getContactPoints(bodyA=self.robot_id)
                for contact in contacts_robot:
                    body_b_id = contact[2]
                    if body_b_id != 0 and body_b_id != self.held_object_id and body_b_id in self.all_obstacle_ids:
                        return False

                # Check held object collisions (if any)
                if self.held_object_id is not None:
                    contacts_held = p.getContactPoints(bodyA=self.held_object_id)
                    for contact in contacts_held:
                        body_b_id = contact[2]
                        if body_b_id != self.robot_id and body_b_id != 0 and body_b_id in self.all_obstacle_ids:
                            return False
                return True
            finally:
                for i, idx in enumerate(self.arm_joints):
                    p.resetJointState(self.robot_id, idx, saved_states[i])

    def _create_state_space(self, robot_id, arm_joints):
        """Define the OMPL state space for the robot."""
        n_dof = len(arm_joints)
        space = ob.RealVectorStateSpace(n_dof)
        bounds = ob.RealVectorBounds(n_dof)
        for i, joint_idx in enumerate(arm_joints):
            info = p.getJointInfo(robot_id, joint_idx)
            bounds.setLow(i, info[8])
            bounds.setHigh(i, info[9])
        space.setBounds(bounds)
        return space

    def _plan_motion(self, robot_id, arm_joints, ee_index, start_config, target_pos, target_orient, held_object_id=None):
        """Plan a collision-free path using OMPL."""
        print("OMPL Planner: Planning motion...") # Use logger
        n_dof = len(arm_joints)
        space = self._create_state_space(robot_id, arm_joints)
        si = ob.SpaceInformation(space)
        validity_checker = self._PandaValidityChecker(si, robot_id, arm_joints, n_dof, self.all_obstacle_ids)
        validity_checker.held_object_id = held_object_id
        si.setStateValidityChecker(validity_checker)
        si.setStateValidityCheckingResolution(0.005)
        motion_validator = ob.DiscreteMotionValidator(si)
        si.setMotionValidator(motion_validator)
        space.setup()
        si.setup()

        # Calculate goal joint configuration using IK
        try:
            goal_joint_positions = p.calculateInverseKinematics(
                bodyUniqueId=robot_id,
                endEffectorLinkIndex=ee_index,
                targetPosition=target_pos,
                targetOrientation=target_orient,
                maxNumIterations=1000,
                residualThreshold=1e-6
            )
            goal_config = list(goal_joint_positions[:n_dof])
        except Exception as e:
            print(f"OMPL Planner: IK failed for goal pose: {e}") # Use logger
            return None

        if not self._is_joint_config_valid(robot_id, arm_joints, start_config) or not self._is_joint_config_valid(robot_id, arm_joints, goal_config):
             print("OMPL Planner: Invalid start or goal configuration.") # Use logger
             return None

        try:
            pdef = ob.ProblemDefinition(si)
            start_ompl = ob.State(space)
            goal_ompl = ob.State(space)
            for i in range(n_dof):
                 start_ompl[i] = start_config[i]
                 goal_ompl[i] = goal_config[i]
            pdef.setStartAndGoalStates(start_ompl, goal_ompl)

            planner = og.RRTstar(si) # Or RRTConnect
            planner.setRange(0.5) # Adjust as needed
            planner.setProblemDefinition(pdef)
            planner.setup()

            solved = planner.solve(config.simulation.ompl_timeout) # e.g., 2.0

            if solved:
                print("OMPL Planner: Found collision-free path!") # Use logger
                path = pdef.getSolutionPath()
                if path:
                    path.interpolate(15) # Interpolate for smoother motion
                    waypoints = []
                    for i in range(path.getStateCount()):
                        state = path.getState(i)
                        waypoint = [state[j] for j in range(n_dof)]
                        waypoints.append(waypoint)
                    return waypoints
                else:
                    print("OMPL Planner: Returned an empty path.") # Use logger
            else:
                print("OMPL Planner: Could not find a path.") # Use logger
            return None
        except Exception as e:
            print(f"OMPL Planner: Planning error: {e}") # Use logger
            return None

    def _execute_path(self, robot_id, arm_joints, waypoints):
        """Execute a planned path by moving through the waypoints."""
        if not waypoints:
            print("OMPL Planner: No waypoints to execute.") # Use logger
            return False

        print(f"OMPL Planner: Executing path with {len(waypoints)} waypoints.") # Use logger
        for i, waypoint in enumerate(waypoints):
            if not self._is_joint_config_valid(robot_id, arm_joints, waypoint):
                print(f"OMPL Planner: Invalid waypoint {i} during execution.") # Use logger
                return False
            self._set_joint_positions(robot_id, arm_joints, waypoint, config.robot.first.max_joint_force)
            for _ in range(2):
                 p.stepSimulation()
                 time.sleep(config.simulation.step_delay)
        print("OMPL Planner: Path execution completed.") # Use logger
        return True

    def move_to_pose(self, robot_id, arm_joints, ee_index, target_pos, target_orient, log_msg="", held_object_id=None, **kwargs):
        """Move the robot's EE to a target pose using OMPL for collision-free path planning."""
        if log_msg:
            print(f"OMPL Planner: {log_msg}") # Use logger

        start_config = self._get_current_joint_states(robot_id, arm_joints)
        if not self._is_joint_config_valid(robot_id, arm_joints, start_config):
             print("OMPL Planner: Current state is invalid.") # Use logger

        waypoints = self._plan_motion(robot_id, arm_joints, ee_index, start_config, target_pos, target_orient, held_object_id=held_object_id)

        if waypoints:
            # self.path_visualizer.visualize_path(robot_id, arm_joints, ee_index, waypoints) # Optional
            print("OMPL Planner: Using collision-free path.") # Use logger
            success = self._execute_path(robot_id, arm_joints, waypoints)
            wait(config.simulation.settle_steps)
            # self.path_visualizer.clear_path() # Optional
            return success
        else:
            print("OMPL Planner: Failed, using direct IK control as fallback.") # Use logger
            # Fallback to IK
            ik_planner = IKPlanner() # Or pass an instance
            return ik_planner.move_to_pose(robot_id, arm_joints, ee_index, target_pos, target_orient, log_msg="OMPL Fallback IK")
        return False

    def move_to_home(self, robot_id, arm_joints, home_position, tolerance=None, timeout=None, **kwargs):
        """Move the robot arm to the predefined home position."""
        # Delegate to IK planner for simplicity, or implement OMPL-based homing if needed.
        ik_planner = IKPlanner() # Or pass an instance
        return ik_planner.move_to_home(robot_id, arm_joints, home_position, tolerance, timeout)


class PathVisualizer:
    """Handles visualization of planned paths."""
    def __init__(self):
        self.line_ids = []

    def visualize_path(self, robot_id, arm_joints, ee_index, waypoints, color=[0, 1, 0]):
        """Draw debug lines between waypoints."""
        self.clear_path()
        if len(waypoints) < 2:
            return

        ee_positions = []
        saved_states = [p.getJointState(robot_id, idx)[0] for idx in arm_joints]
        try:
            for waypoint in waypoints:
                 for i, idx in enumerate(arm_joints):
                     p.resetJointState(robot_id, idx, waypoint[i])
                 ee_state = p.getLinkState(robot_id, ee_index)
                 ee_positions.append(list(ee_state[0]))
        finally:
            for i, idx in enumerate(arm_joints):
                p.resetJointState(robot_id, idx, saved_states[i])

        for i in range(len(ee_positions) - 1):
            start_pos = ee_positions[i]
            end_pos = ee_positions[i+1]
            line_id = p.addUserDebugLine(start_pos, end_pos, lineColorRGB=color, lineWidth=2)
            self.line_ids.append(line_id)

    def clear_path(self):
        """Remove existing debug lines."""
        for line_id in self.line_ids:
            p.removeUserDebugItem(line_id)
        self.line_ids = []
