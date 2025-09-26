import pybullet as p
import pybullet_data
import numpy as np
import time
import math
from ompl import base as ob
from ompl import geometric as og
import traceback

# --- Configuration ---
SIM_TIMESTEP = 1.0 / 240.0
ROBOT_URDF = "franka_panda/panda.urdf"
OBSTACLE_URDF = "cube.urdf"  # PyBullet's built-in cube URDF

# Robot starting position
ROBOT_START_POS = [0, 0, 0]
ROBOT_START_ORN = [0, 0, 0, 1]

# Robot's end-effector link index (Franka Panda hand)
EE_LINK_INDEX = 11

# OMPL Planner settings
PLANNER_TIMEOUT = 5.0  # seconds
MAX_PLANNING_ITERATIONS = 5000

# Execution settings
EXECUTION_ITERATIONS_PER_WAYPOINT = 5
EXECUTION_VELOCITY = 0.5

# --- PyBullet Setup ---
def setup_pybullet():
    """Initialize PyBullet, load plane, robot, and obstacles."""
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(SIM_TIMESTEP)

    # Load ground plane
    p.loadURDF("plane.urdf")

    # Load robot
    robot_id = p.loadURDF(ROBOT_URDF, ROBOT_START_POS, ROBOT_START_ORN, useFixedBase=True)

    # Get arm joint indices (first 7 revolute joints)
    num_joints = p.getNumJoints(robot_id)
    arm_joints = []
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        if joint_info[2] != p.JOINT_FIXED:  # Exclude fixed joints
            arm_joints.append(i)
        if len(arm_joints) == 7:  # Stop after 7 arm joints
            break

    if len(arm_joints) < 7:
        print("Error: Could not find 7 arm joints.")
        p.disconnect()
        exit()

    print(f"Loaded robot with {len(arm_joints)} arm joints: {arm_joints}")
    
    # Debug: Print actual joint limits
    print("Actual joint limits from PyBullet:")
    for i, joint_idx in enumerate(arm_joints):
        joint_info = p.getJointInfo(robot_id, joint_idx)
        print(f"  Joint {i} (idx {joint_idx}): limits = [{joint_info[8]}, {joint_info[9]}] (type: {joint_info[2]})")
    
    return physicsClient, robot_id, arm_joints

# --- OMPL Setup ---
def setup_ompl(robot_id, arm_joints):
    """Create OMPL space, state validity checker, and space information."""
    n_dof = len(arm_joints)

    # Define the state space
    space = ob.RealVectorStateSpace(n_dof)
    bounds = ob.RealVectorBounds(n_dof)
    
    # Get actual joint limits from PyBullet and store them
    for i in range(n_dof):
        joint_info = p.getJointInfo(robot_id, arm_joints[i])
        lower_limit = joint_info[8]
        upper_limit = joint_info[9]
        
        bounds.setLow(i, lower_limit)
        bounds.setHigh(i, upper_limit)
    
    space.setBounds(bounds)

    # Rest of the function remains the same...
    si = ob.SpaceInformation(space)

    class PandaValidityChecker(ob.StateValidityChecker):
        def __init__(self, space_info, robot_id, arm_joints):
            super().__init__(space_info)
            self.robot_id = robot_id
            self.arm_joints = arm_joints
            self.n_dof = len(arm_joints)
            self.lower_limits = []
            self.upper_limits = []
            
            for j_idx in self.arm_joints:
                j_info = p.getJointInfo(self.robot_id, j_idx)
                lower = j_info[8]
                upper = j_info[9]
                
                self.lower_limits.append(lower)
                self.upper_limits.append(upper)

        def isValid(self, state):
            # Check joint limits first
            for i in range(self.n_dof):
                joint_pos = state[i]
                if not (self.lower_limits[i] <= joint_pos <= self.upper_limits[i]):
                    return False

            # Check for collisions by temporarily setting the robot's state
            saved_states = [p.getJointState(self.robot_id, j_idx)[0] for j_idx in self.arm_joints]
            try:
                for i, j_idx in enumerate(self.arm_joints):
                    p.resetJointState(self.robot_id, j_idx, state[i])
                # Check for collisions
                contact_points = p.getContactPoints(bodyA=self.robot_id)
                has_collision = len(contact_points) > 0
                return not has_collision
            finally:
                # Always restore the original joint states
                for i, j_idx in enumerate(self.arm_joints):
                    p.resetJointState(self.robot_id, j_idx, saved_states[i])

    validity_checker = PandaValidityChecker(si, robot_id, arm_joints)
    si.setStateValidityChecker(validity_checker)
    space.setup()
    si.setup()

    return space, si, validity_checker

# --- Helper Functions ---
def get_current_joint_positions(robot_id, arm_joints):
    """Get the current positions of the arm joints."""
    return [p.getJointState(robot_id, j_idx)[0] for j_idx in arm_joints]

def set_joint_positions(robot_id, arm_joints, target_positions):
    """Set target positions for the robot arm joints."""
    print(f"Target joint positions: {target_positions}")
    for i, j_idx in enumerate(arm_joints):
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=j_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_positions[i],
            force=240.0,  # Default Panda force
            maxVelocity=EXECUTION_VELOCITY
        )

def calculate_ik(robot_id, ee_link_index, target_pos, target_orn):
    """Calculate inverse kinematics for the end-effector."""
    try:
        joint_positions = p.calculateInverseKinematics(
            bodyUniqueId=robot_id,
            endEffectorLinkIndex=ee_link_index,
            targetPosition=target_pos,
            targetOrientation=target_orn,
            maxNumIterations=100,
            residualThreshold=1e-5
        )
        return list(joint_positions)
    except:
        print("  -> IK calculation failed.")
        return None

def draw_path_on_sim(robot_id, ee_link_index, waypoints, arm_joints):
    """Draw debug lines between EE positions of the waypoints."""
    p.removeAllUserDebugItems()  # Clear previous lines
    if len(waypoints) < 2:
        return

    # Get EE positions for each waypoint by setting joints and querying EE state
    ee_positions = []
    saved_states = [p.getJointState(robot_id, j_idx)[0] for j_idx in arm_joints]

    try:
        for waypoint in waypoints:
             for i, j_idx in enumerate(arm_joints):
                 p.resetJointState(robot_id, j_idx, waypoint[i])
             ee_state = p.getLinkState(robot_id, ee_link_index)
             ee_positions.append(list(ee_state[0]))  # Store EE position
    finally:
        # Always restore the original joint states
        for i, j_idx in enumerate(arm_joints):
            p.resetJointState(robot_id, j_idx, saved_states[i])

    # Draw lines between consecutive EE positions
    for i in range(len(ee_positions) - 1):
        start_pos = ee_positions[i]
        end_pos = ee_positions[i+1]
        p.addUserDebugLine(start_pos, end_pos, lineColorRGB=[0, 1, 0], lineWidth=2.0)  # Green line

def evaluate_path_quality(path):
    """Evaluate path quality based on length and smoothness."""
    if path is None or path.getStateCount() < 2:
        return float('inf')
    
    # Calculate total path length
    total_length = 0.0
    for i in range(path.getStateCount() - 1):
        state1 = path.getState(i)
        state2 = path.getState(i + 1)
        # Calculate Euclidean distance between consecutive states
        dist = 0.0
        for j in range(path.getSpaceInformation().getStateSpace().getDimension()):
            diff = state1[j] - state2[j]
            dist += diff * diff
        total_length += math.sqrt(dist)
    
    return total_length

def validate_state_in_ompl_space(si, start_state, space, arm_joints, start_config):
    """Debug function to validate state in OMPL space."""
    print("Debug: Validating start state in OMPL space...")
    print(f"  State dimension: {space.getDimension()}")
    
    # Get bounds from the space
    bounds = space.getBounds()
    print(f"  Space bounds: low = {[bounds.low[i] for i in range(space.getDimension())]}")
    print(f"  Space bounds: high = {[bounds.high[i] for i in range(space.getDimension())]}")
    print(f"  Start config: {start_config}")
    
    all_valid = True
    for i in range(space.getDimension()):
        val = start_state[i]
        low = bounds.low[i]
        high = bounds.high[i]
        is_valid = low <= val <= high
        print(f"  Joint {i}: value={val}, bounds=[{low}, {high}], valid={is_valid}")
        if not is_valid:
            print(f"    ERROR: Joint {i} value {val} is out of bounds [{low}, {high}]")
            all_valid = False
    
    if all_valid:
        print("  All joint values are within bounds!")
    return all_valid

def plan_with_algorithm(si, start_state, goal_state, planner_type, timeout=PLANNER_TIMEOUT):
    """Plan using a specific algorithm."""
    try:
        print(f"Trying planner: {planner_type.__name__}")
        
        # Create problem definition
        pdef = ob.ProblemDefinition(si)
        pdef.setStartAndGoalStates(start_state, goal_state)

        # Create planner
        planner = planner_type(si)
        planner.setProblemDefinition(pdef)
        planner.setup()

        # Solve the problem
        solved = planner.solve(timeout)

        if solved:
            path = pdef.getSolutionPath()
            path.interpolate()  # Add more intermediate states
            path_quality = evaluate_path_quality(path)
            print(f"  -> Success! Path quality: {path_quality:.4f}")
            return path, path_quality, planner_type.__name__
        else:
            print("  -> Failed to find a path")
            return None, float('inf'), planner_type.__name__
    
    except Exception as e:
        print(f"  -> Error with {planner_type.__name__}: {str(e)}")
        print(f"  -> Traceback: {traceback.format_exc()}")
        return None, float('inf'), planner_type.__name__

# --- Main Execution ---
def main():
    """Main function to run the OMPL demonstration."""
    print("Initializing PyBullet and OMPL...")
    client, robot_id, arm_joints = setup_pybullet()
    
    # Create red obstacles (cubes) between robot and goal
    red_obstacles = []
    # Place red obstacles in a line between start and goal - lower and closer to robot
    for i in range(3):
        obstacle_pos = [0.35 + i * 0.05, 0.0, 0.05]  # x positions: 0.35, 0.40, 0.45, lower z
        obstacle_id = p.loadURDF(
            OBSTACLE_URDF,
            obstacle_pos,
            [0, 0, 0, 1],
            globalScaling=0.08,
            useFixedBase=True
        )
        # Set color to red
        p.changeVisualShape(obstacle_id, -1, rgbaColor=[1, 0, 0, 1])  # Red
        red_obstacles.append(obstacle_id)
        print(f"Added red obstacle at {obstacle_pos}")

    # Create blue goal object
    blue_goal_pos = [0.55, 0.0, 0.15]  # Beyond the red obstacles, higher z
    blue_goal_id = p.loadURDF(
        OBSTACLE_URDF,
        blue_goal_pos,
        [0, 0, 0, 1],
        globalScaling=0.08,
        useFixedBase=True
    )
    # Set color to blue
    p.changeVisualShape(blue_goal_id, -1, rgbaColor=[0, 0, 1, 1])  # Blue
    print(f"Added blue goal at {blue_goal_pos}")

    space, si, validity_checker = setup_ompl(robot_id, arm_joints)

    # Define Start and Goal EE poses - NOW WITHIN ROBOT REACH
    # Start: Before the red obstacles, at a more achievable position
    start_ee_pos = [0.25, 0.0, 0.25]  # Closer to robot, higher up
    start_ee_orn = p.getQuaternionFromEuler([math.pi, 0, 0])  # Pointing down

    # Goal: Beyond the red obstacles, at a more achievable position
    goal_ee_pos = [0.55, 0.0, 0.25]  # Higher up, achievable
    goal_ee_orn = p.getQuaternionFromEuler([math.pi, 0, 0])  # Pointing down

    print(f"Start EE Pose: Pos={start_ee_pos}, Orn={start_ee_orn}")
    print(f"Goal EE Pose: Pos={goal_ee_pos}, Orn={goal_ee_orn}")

    # Calculate Start and Goal Joint Configurations using IK
    start_config = calculate_ik(robot_id, EE_LINK_INDEX, start_ee_pos, start_ee_orn)
    goal_config = calculate_ik(robot_id, EE_LINK_INDEX, goal_ee_pos, goal_ee_orn)

    if start_config is None or goal_config is None:
        print("Error: Could not calculate IK for start or goal pose. Trying alternative positions...")
        
        # Try even more conservative positions
        start_ee_pos = [0.3, 0.0, 0.3]  # Even more conservative
        goal_ee_pos = [0.45, 0.0, 0.3]  # Even more conservative
        
        print(f"Trying alternative positions:")
        print(f"New Start EE Pose: Pos={start_ee_pos}, Orn={start_ee_orn}")
        print(f"New Goal EE Pose: Pos={goal_ee_pos}, Orn={goal_ee_orn}")
        
        start_config = calculate_ik(robot_id, EE_LINK_INDEX, start_ee_pos, start_ee_orn)
        goal_config = calculate_ik(robot_id, EE_LINK_INDEX, goal_ee_pos, goal_ee_orn)
        
        if start_config is None or goal_config is None:
            print("Error: Could not calculate IK for even conservative positions. Exiting.")
            p.disconnect()
            return

    start_config = start_config[:len(arm_joints)]
    goal_config = goal_config[:len(arm_joints)]

    print(f"Start Joint Config: {start_config}")
    print(f"Goal Joint Config: {goal_config}")

    # --- Validate start state before OMPL planning ---
    print("\n--- Validating start state in OMPL space ---")
    start_state = ob.State(space)
    goal_state = ob.State(space)
    for i in range(len(arm_joints)):
        start_state[i] = start_config[i]
        goal_state[i] = goal_config[i]
    
    # Validate the start state
    is_valid = validate_state_in_ompl_space(si, start_state, space, arm_joints, start_config)
    if not is_valid:
        print("ERROR: Start state is invalid in OMPL space!")
        print("Consider adjusting start/goal positions to be more achievable by the robot.")
        p.disconnect()
        return
    
    # Also validate goal state
    print("\n--- Validating goal state in OMPL space ---")
    is_goal_valid = validate_state_in_ompl_space(si, goal_state, space, arm_joints, goal_config)
    if not is_goal_valid:
        print("ERROR: Goal state is invalid in OMPL space!")
        print("Consider adjusting start/goal positions to be more achievable by the robot.")
        p.disconnect()
        return
    
    print("Both start and goal states validation passed!")

    # --- OMPL Planning with Multiple Algorithms ---
    print("\nStarting OMPL planning with multiple algorithms...")
    
    # List of planners to try
    planners = [
        og.RRTstar,
        og.RRT,
        og.RRTConnect,
        og.LBTRRT,
        og.PRM,
        og.LazyRRT,
        og.SBL,
        og.KPIECE1,
        og.BKPIECE1,
        og.LBKPIECE1
    ]

    best_path = None
    best_quality = float('inf')
    best_planner_name = ""
    results = []

    for planner_type in planners:
        try:
            path, quality, planner_name = plan_with_algorithm(si, start_state, goal_state, planner_type)
            results.append((planner_name, quality, path is not None))
            
            if path is not None and quality < best_quality:
                best_path = path
                best_quality = quality
                best_planner_name = planner_name
                print(f"  -> New best path found with {planner_name} (quality: {quality:.4f})")
        except Exception as e:
            print(f"  -> Exception in {planner_type.__name__}: {str(e)}")
            results.append((planner_type.__name__, float('inf'), False))

    # Print results summary
    print("\n--- Planning Results Summary ---")
    for planner_name, quality, success in results:
        status = "SUCCESS" if success else "FAILED"
        quality_str = f"{quality:.4f}" if quality != float('inf') else "N/A"
        print(f"{planner_name:12} | {status:7} | Quality: {quality_str}")
    
    print(f"\nBest planner: {best_planner_name} with quality: {best_quality:.4f}")

    if best_path is not None:
        print(f"OMPL found a collision-free path using {best_planner_name}!")
        waypoints = []
        for i in range(best_path.getStateCount()):
            state = best_path.getState(i)
            waypoint = [state[j] for j in range(len(arm_joints))]
            waypoints.append(waypoint)

        print(f"Planned path with {len(waypoints)} waypoints.")

        # Visualize the path in PyBullet
        draw_path_on_sim(robot_id, EE_LINK_INDEX, waypoints, arm_joints)

        # Execute the path
        print("Executing the planned path...")
        for i, waypoint in enumerate(waypoints):
            print(f"  Moving to waypoint {i+1}/{len(waypoints)}")
            set_joint_positions(robot_id, arm_joints, waypoint)
            for _ in range(EXECUTION_ITERATIONS_PER_WAYPOINT):
                p.stepSimulation()
                time.sleep(SIM_TIMESTEP)

        print("Path execution completed.")
        
        # Show final EE position
        final_ee_state = p.getLinkState(robot_id, EE_LINK_INDEX)
        print(f"Final EE position: {final_ee_state[0]}")
        print(f"Goal EE position: {goal_ee_pos}")
        distance_to_goal = math.sqrt(sum([(a-b)**2 for a, b in zip(final_ee_state[0], goal_ee_pos)]))
        print(f"Distance to goal: {distance_to_goal:.4f}")
        
    else:
        print("OMPL could not find a path with any algorithm.")
        print("Consider adjusting start/goal positions or checking collision setup.")

    # Keep simulation running
    print("Simulation running. Close the window to exit.")
    try:
        while p.isConnected():
            p.stepSimulation()
            time.sleep(SIM_TIMESTEP)
    except KeyboardInterrupt:
        print("Simulation stopped by user.")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()