import pybullet as p
import pybullet_data
from ompl import base as ob
from ompl import geometric as og
import time

# --- Setup PyBullet ---
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load ground, board, chess pieces, robot URDFs
plane = p.loadURDF("plane.urdf")
board = p.loadURDF("./assets/urdfs/chess_board.urdf", basePosition=[0.5,0,0])
# load pieces, store their body IDs in a list
piece_ids = []
# e.g. piece_ids.append(p.loadURDF(...))  etc.

robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
num_joints = p.getNumJoints(robot)
# (Probably only first 7 joints are movable; the fingers or others may be ignored)

# Optionally: enable collision for self collisions in URDF loading, etc.

# --- Build OMPL state space ---
dof = 7  # or however many joints you're planning over
space = ob.RealVectorStateSpace(dof)

bounds = ob.RealVectorBounds(dof)
for i in range(dof):
    info = p.getJointInfo(robot, i)
    print(info)
    lower = info[8]  # joint lower limit
    upper = info[9]  # joint upper limit
    bounds.setLow(i, lower)
    bounds.setHigh(i, upper)
space.setBounds(bounds)

si = ob.SpaceInformation(space)

# --- Validity checker function ---
def is_state_valid(state):
    # state is an OMPL state, of type RealVectorStateSpace.StateType
    # Extract joint angle vector
    joint_vals = [state[i] for i in range(dof)]
    # Apply to PyBullet
    for j in range(dof):
        p.resetJointState(robot, j, joint_vals[j])
    # If the robot “holds” a chess piece, attach or move it accordingly
    # (you should make that part of the world state)

    # Check collisions using PyBullet
    # getContactPoints returns contact information
    contacts = p.getContactPoints()
    if len(contacts) > 0:
        return False
    # Optionally, you could check getClosestPoints for clearance margin

    return True

# Wrap into an OMPL validity checker
valid_fn = ob.StateValidityCheckerFn(is_state_valid)
si.setStateValidityChecker(valid_fn)

# Optionally adjust how fine the motion checking is
si.setStateValidityCheckingResolution(0.01)  # check 1% intervals along edges

si.setup()

# --- Define start & goal states ---
start = ob.State(space)
goal = ob.State(space)

# Fill start / goal joint values (you may compute via IK or prior knowledge)
# Example:
for i in range(dof):
    start[i] = 0.0
    goal[i] = 0.5  # some target configuration

pdef = ob.ProblemDefinition(si)
pdef.setStartAndGoalStates(start, goal)

planner = og.RRTConnect(si)
planner.setProblemDefinition(pdef)
planner.setup()

# --- Solve ---
timeout = 5.0
solved = planner.solve(timeout)
if solved:
    path = pdef.getSolutionPath()
    print("Path found with", path.getStateCount(), "states")
    # You can do path simplification, smoothing if desired
    # Execute the path
    for i in range(path.getStateCount()):
        st = path.getState(i)
        # set robot to that configuration
        for j in range(dof):
            p.resetJointState(robot, j, st[j])
        p.stepSimulation()
        time.sleep(0.05)
else:
    print("No solution found")

# Keep GUI alive
while True:
    p.stepSimulation()
    time.sleep(0.01)
