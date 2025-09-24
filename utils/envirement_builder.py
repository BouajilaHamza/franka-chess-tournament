import logging
import numpy as np
import pybullet_data
import pybullet as p

from configs.config import config 
from utils.simulation import wait



logger = logging.getLogger(__name__)

def setup_simulation():
    """Initialize PyBullet, load environment, and configure settings."""
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(config.simulation.step_delay)
    p.setRealTimeSimulation(0)
    logger.info("Simulation initialized.")
    return physics_client

def load_plane():
    """Load the ground plane."""
    plane_id = p.loadURDF("plane.urdf")
    p.changeDynamics(plane_id, -1, lateralFriction=1.0)
    logger.debug("Ground plane loaded.")
    return plane_id

def load_robot(position, orientation):
    """Load the robot and configure its joints for better grasp stability."""
    robot_id = p.loadURDF(
        
        config.robot.first.urdf,
        basePosition=position,
        baseOrientation=orientation,
        useFixedBase=True
    )

    # Identify arm revolute joints
    arm_joint_indices = [
        i for i in range(p.getNumJoints(robot_id))
        if p.getJointInfo(robot_id, i)[2] == p.JOINT_REVOLUTE
    ][:config.robot.first.num_arm_joints]

    # Set damping on those arm joints (reduce jitter)
    for joint_idx in arm_joint_indices:
        p.changeDynamics(robot_id,
                         linkIndex=joint_idx,
                         linearDamping=0.0,
                         angularDamping=config.simulation.default_joint_damping)

    # Identify gripper finger joints (assuming last 2 revolute joints are fingers)
    gripper_indices = [
        i for i in range(p.getNumJoints(robot_id))
        if p.getJointInfo(robot_id, i)[2] == p.JOINT_REVOLUTE
    ][-2:]



    # These dynamics settings help with grasp friction
    for finger_link in gripper_indices:
        p.changeDynamics(robot_id,
                         linkIndex=finger_link,
                         lateralFriction=1.5,
                         spinningFriction=0.1,
                         rollingFriction=0.1,
                         restitution=0.0,
                         contactStiffness=30000.0,
                         contactDamping=1000.0)
    
    # Optional: also set friction on the pawn / object you want to grip
    # assuming you load it elsewhere and have its bodyId and link index (or base link = -1)
    # p.changeDynamics(pawn_id, -1, lateralFriction=1.5, restitution=0.0, etc.)

    # Improve solver & simulation parameters
    p.setPhysicsEngineParameter(numSolverIterations=150)
    # You might also set other parameters if needed, like ERP, CFM, etc.

    logger.info("Robot loaded and configured with high friction fingers.")
    return robot_id, arm_joint_indices, gripper_indices


# def load_environment():
#     """Load the chess board."""
#     try:
#         board_id = p.loadURDF(config.environment.board_urdf, basePosition=config.environment.board_position,
#                               baseOrientation=config.environment.board_orientation, useFixedBase=True)
#         logger.info(f"Environment loaded (Board ID: {board_id}).")
#         return board_id
#     except p.error as e:
#         logger.error(f"Error loading board URDF '{config.environment.board_urdf}': {e}")
#         return None

# do the same for the gripper finger links (use correct link index)

# # optional: set contact stiffness/damping (newer PyBullet builds)
# def load_pawn(position):
#     """Load a pawn at a given position."""
#     try:
#         pawn_id = p.loadURDF(config.object.pawn_urdf, basePosition=position, useFixedBase=False)
#         # --- Key Change 2: Increase Pawn Friction ---
#         # Apply to all links of the pawn (assuming single link URDF, link index 0)
#         p.changeDynamics(pawn_id, -1, lateralFriction=3, spinningFriction=3, rollingFriction=3) # Increased from 1.0
#         p.changeDynamics(pawn_id, -1, contactStiffness=1e4, contactDamping=1e3)
#         # Let the pawn settle briefly
#         wait(config.simulation.settle_steps // 2)
#         logger.info(f"Pawn loaded (ID: {pawn_id}) at {position} with increased friction.")
#         return pawn_id, position
#     except p.error as e:
#         logger.error(f"Error loading pawn URDF '{config.object.pawn_urdf}': {e}")
#         return None
    




class EnvironmentBuilder:
    """
    Builds the PyBullet simulation environment for the Chess Robot Simulation.
    Loads the plane, two robots, the chess board, and all 32 chess pieces in their initial positions.
    Also defines the mapping between 3D world coordinates and chess notation.
    """
    def __init__(self):
        self.physics_client = None
        self.plane_id = None
        self.board_id = None
        self.robot1_id = None
        self.robot1_arm_joints = []
        self.robot1_gripper_joints = []
        self.robot2_id = None
        self.robot2_arm_joints = []
        self.robot2_gripper_joints = []
        self.piece_ids = {} # Dictionary to store piece_id -> square mapping
        self.square_to_world_coords = {} # Dictionary mapping chess notation (e.g., 'a1') to 3D world coordinates [x, y, z]
        self.is_built = False

    def connect(self):
        """Initialize PyBullet connection."""
        if self.physics_client is not None:
            logger.warning("Physics client already connected. Disconnecting first.")
            p.disconnect(self.physics_client)

        self.physics_client = p.connect(p.GUI)
        if self.physics_client < 0:
            raise RuntimeError("Failed to connect to PyBullet.")

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(config.simulation.step_delay)
        p.setRealTimeSimulation(0) # Control the simulation loop manually
        logger.info("PyBullet connected and configured.")
        return self # Return self for method chaining

    def load_ground_plane(self):
        """Load the ground plane."""
        if self.physics_client is None:
            raise RuntimeError("Physics client not connected. Call connect() first.")
        self.plane_id = load_plane() # Use your existing function
        return self

    def load_robots(self):
        """Load the two Franka Panda arms."""
        if self.physics_client is None:
            raise RuntimeError("Physics client not connected. Call connect() first.")

        # Load Robot 1 (e.g., White, positioned on one side)
        robot1_pos = config.robot.first.start_position
        robot1_orient = config.robot.first.start_orientation
        self.robot1_id, self.robot1_arm_joints, self.robot1_gripper_joints = load_robot(robot1_pos, robot1_orient)

        # Load Robot 2 (e.g., Black, positioned on the opposite side)
        robot2_pos = config.robot.second.start_position # Assuming your config has second robot settings
        robot2_orient = config.robot.second.start_orientation
        self.robot2_id, self.robot2_arm_joints, self.robot2_gripper_joints = load_robot(robot2_pos, robot2_orient)

        logger.info("Two robots loaded and configured.")
        return self

    def load_chess_board(self):
        """Load the chess board."""
        if self.physics_client is None:
            raise RuntimeError("Physics client not connected. Call connect() first.")
        self.board_id = p.loadURDF( config.environment.board_urdf, 
                                    basePosition=config.environment.board_position,
                                    baseOrientation=config.environment.board_orientation, 
                                    useFixedBase=True)
        logger.info(f"Environment loaded (Board ID: {self.board_id}).")    
        if self.board_id is None:
            raise RuntimeError("Failed to load the chess board.")
        return self

    def _define_square_mapping(self):
        """Define the mapping between chess notation and 3D world coordinates."""
        board_pos = np.array(config.environment.board_position)
        _board_orient_quat = config.environment.board_orientation
        # For a standard board, we can calculate the square centers based on its position and a fixed square size.
        # Assume the board URDF is centered at board_pos, and the board is 0.4m x 0.4m (8 squares * 0.05m each)
        # Adjust the calculation based on your actual board URDF dimensions and orientation if needed.
        square_size = config.environment.board_square_size # Assuming you add this to your config
        board_width = 8 * square_size
        board_start_x = board_pos[0] - board_width / 2.0 + square_size / 2.0 # Center of first square (a-file)
        board_start_y = board_pos[1] - board_width / 2.0 + square_size / 2.0 # Center of first square (1st rank)

        for rank_idx in range(8): # Ranks 1 to 8 (PyBullet Y-axis)
            for file_idx in range(8): # Files a to h (PyBullet X-axis)
                file_char = chr(ord('a') + file_idx)
                rank_char = str(rank_idx + 1)
                square_name = f"{file_char}{rank_char}"

                x = board_start_x + file_idx * square_size
                y = board_start_y + rank_idx * square_size
                z = board_pos[2] + config.object.piece_height / 2.0 # Approximate Z height for piece base

                self.square_to_world_coords[square_name] = [x, y, z]

        logger.debug(f"Defined square mapping: {list(self.square_to_world_coords.keys())[:5]}...") # Log first 5 keys
        return self

    def load_pieces(self):
        """Load all 32 chess pieces in their initial positions."""
        if self.physics_client is None:
            raise RuntimeError("Physics client not connected. Call connect() first.")
        if not self.square_to_world_coords:
             self._define_square_mapping() # Ensure mapping is defined first

        # Define initial piece positions (standard chess starting position)
        # Format: {square_name: piece_type}
        initial_positions = {
            'a1': 'rook_w', 'b1': 'knight_w', 'c1': 'bishop_w', 'd1': 'queen_w', 'e1': 'king_w', 'f1': 'bishop_w', 'g1': 'knight_w', 'h1': 'rook_w',
            'a2': 'pawn_w', 'b2': 'pawn_w', 'c2': 'pawn_w', 'd2': 'pawn_w', 'e2': 'pawn_w', 'f2': 'pawn_w', 'g2': 'pawn_w', 'h2': 'pawn_w',
            'a7': 'pawn_b', 'b7': 'pawn_b', 'c7': 'pawn_b', 'd7': 'pawn_b', 'e7': 'pawn_b', 'f7': 'pawn_b', 'g7': 'pawn_b', 'h7': 'pawn_b',
            'a8': 'rook_b', 'b8': 'knight_b', 'c8': 'bishop_b', 'd8': 'queen_b', 'e8': 'king_b', 'f8': 'bishop_b', 'g8': 'knight_b', 'h8': 'rook_b',
        }

        for square, piece_type in initial_positions.items():
            world_pos = self.square_to_world_coords[square]
            # Determine URDF path based on piece type
            urdf_path = self._get_piece_urdf_path(piece_type)
            if not urdf_path:
                logger.error(f"URDF path not found for piece type: {piece_type}")
                continue # Skip this piece

            # Load the piece using the new generic function
            piece_id = self._load_piece_at_position(urdf_path, world_pos)
            if piece_id is not None:
                self.piece_ids[piece_id] = square # Map piece ID back to square name
                logger.info(f"Loaded {piece_type} on square {square} at {world_pos} (ID: {piece_id})")
            else:
                logger.error(f"Failed to load {piece_type} on square {square} at {world_pos}")

        logger.info("All pieces loaded.")
        return self

    def _get_piece_urdf_path(self, piece_type):
        """Map piece type string to its URDF file path."""
        # This assumes you have separate URDFs for each piece type (e.g., pawn_w.urdf, king_b.urdf)
        # Update the mapping based on your actual URDF filenames.
        urdf_map = {
            'pawn_w': config.object.pawn_urdf, # Or config.object.pawn_white_urdf
            'rook_w': config.object.rook_urdf, # Add these to your config
            'knight_w': config.object.knight_urdf,
            'bishop_w': config.object.bishop_urdf,
            'queen_w': config.object.queen_urdf,
            'king_w': config.object.king_urdf,
            'pawn_b': config.object.pawn_urdf, # Or config.object.pawn_black_urdf
            'rook_b': config.object.rook_urdf,
            'knight_b': config.object.knight_urdf,
            'bishop_b': config.object.bishop_urdf,
            'queen_b': config.object.queen_urdf,
            'king_b': config.object.king_urdf,
        }
        return urdf_map.get(piece_type)

    def _load_piece_at_position(self, urdf_path, position):
        """Load a single piece of a given type at a specific 3D position."""
        try:
            piece_id = p.loadURDF(urdf_path, basePosition=position, useFixedBase=False)
            # Apply dynamics properties for better simulation (similar to load_pawn)
            p.changeDynamics(piece_id, -1,
                            lateralFriction=config.object.piece_lateral_friction,
                            spinningFriction=config.object.piece_spinning_friction,
                            rollingFriction=config.object.piece_rolling_friction,
                            restitution=config.object.piece_restitution)
            p.changeDynamics(piece_id, -1,
                            contactStiffness=config.object.piece_contact_stiffness,
                            contactDamping=config.object.piece_contact_damping)
            # Let the piece settle briefly
            wait(config.simulation.settle_steps // 4) # Shorter settle for pieces
            return piece_id
        except p.error as e:
            logger.error(f"Error loading piece URDF '{urdf_path}' at {position}: {e}")
            return None

    def build(self):
        """Finalize the environment setup."""
        builder_components = [  self.physics_client, self.plane_id, self.board_id,
                                self.robot1_id, self.robot2_id, self.square_to_world_coords]
        for component in builder_components:
            if component is None or (isinstance(component, dict) and not component):
                raise RuntimeError(f"Environment is not fully configured. Call {component} first.")


        self.is_built = True
        logger.info("Environment built successfully.")
        # Return a dictionary containing all necessary IDs/components and mappings
        return {
            'physics_client': self.physics_client,
            'board_id': self.board_id,
            'robot1': {
                'id': self.robot1_id,
                'arm_joints': self.robot1_arm_joints,
                'gripper_joints': self.robot1_gripper_joints,
            },
            'robot2': {
                'id': self.robot2_id,
                'arm_joints': self.robot2_arm_joints,
                'gripper_joints': self.robot2_gripper_joints,
            },
            'piece_ids': self.piece_ids.copy(), # Return a copy to prevent external modification
            'square_to_world_coords': self.square_to_world_coords.copy(), # Return a copy
            }