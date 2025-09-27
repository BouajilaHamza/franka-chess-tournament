import json
from pydantic import BaseModel, Field
import pybullet as p
from typing import List



with open('configs/config.json', 'r') as f:
    CONFIG = json.load(f)


def _get_quaternion(euler_list: List[float]) -> List[float]:
    """Helper function to convert Euler angles to quaternion."""
    return p.getQuaternionFromEuler(euler_list)



class SimulationConfig(BaseModel):
    step_delay: float = Field(default=1.0 / 240.0, description="Simulation step delay in seconds")
    settle_steps: int = Field(default=100, ge=0, description="Steps to wait for objects to settle")
    move_steps: int = Field(default=300, ge=0, description="Steps for IK movement phases")
    gripper_action_steps: int = Field(default=150, ge=0, description="Steps for gripper open/close actions")
    default_joint_damping: float = Field(default=0.1, ge=0.0, description="Default joint damping")
    ompl_timeout: float = Field(default=5.0, ge=0.0, description="OMPL planning timeout in seconds")
    ik_max_iterations: int = Field(default=1000, ge=0, description="Maximum number of IK iterations")
    ik_residual_threshold: float = Field(default=1e-7, ge=0.0, description="IK residual threshold")

class SingleRobotConfig(BaseModel):
    urdf: str = Field(default="franka_panda/panda.urdf", description="URDF file path for the robot")
    start_position: List[float] = Field(default=[0, 0, 0], min_items=3, max_items=3, description="Starting position [x, y, z]")
    start_orientation: List[float] = Field(default=[0, 0, 0, 1], min_items=4, max_items=4, description="Starting orientation [x, y, z, w]")
    
    end_effector_index: int = Field(default=11, ge=0, description="Link index of the end effector")
    num_arm_joints: int = Field(default=7, ge=1, description="Number of arm joints")
    max_joint_force: float = Field(default=240.0, gt=0.0, description="Maximum force for arm joints")
    
    gripper_force: float = Field(default=240.0, gt=0.0, description="Force applied by the gripper")
    gripper_open: float = Field(default=0.025, ge=0.0, le=0.08, description="Position when gripper is open")
    gripper_closed: float = Field(default=0.0, ge=0.0, le=0.08, description="Position when gripper is closed")
    gripper_lateral_friction: float = Field(default=1.5, gt=0.0, description="Lateral friction for gripper fingers")
    gripper_spinning_friction: float = Field(default=0.1, gt=0.0, description="Spinning friction for gripper fingers")
    gripper_rolling_friction: float = Field(default=0.1, gt=0.0, description="Rolling friction for gripper fingers")
    gripper_restitution: float = Field(default=0.0, ge=0.0, description="Restitution (bounciness) for gripper fingers")
    gripper_contact_stiffness: float = Field(default=30000.0, gt=0.0, description="Contact stiffness for gripper fingers")
    gripper_contact_damping: float = Field(default=1000.0, gt=0.0, description="Contact damping for gripper fingers")
    
    home_position: List[float] = Field(default=[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], min_items=7, max_items=7, description="Joint angles for the home position")
    home_position_tolerance: float = Field(default=0.01, gt=0.0, description="Tolerance for reaching home position")
    home_move_timeout: float = Field(default=5.0, gt=0.0, description="Timeout for moving to home position (seconds)")


class RobotConfig(BaseModel):
    gripper_indices: List[int] = Field(default=[9, 10], description="List of gripper finger joint indices")
    first: SingleRobotConfig
    second: SingleRobotConfig


class EnvironmentConfig(BaseModel):
    board_urdf: str = Field(default="Open_Chess/urdfs/chess_board.urdf", description="URDF file path for the board")
    board_position: List[float] = Field(default=[0.4, 0, 0.02], min_items=3, max_items=3, description="Board position [x, y, z]")
    board_orientation: List[float] = Field(default=[0, 0, 0, 1], min_items=4, max_items=4, description="Board orientation [x, y, z, w]")
    board_square_size: float = Field(default=0.055, gt=0.0, description="Size of each square on the chess board (meters)")
    
    
    
class ObjectConfig(BaseModel):
    pawn_white_urdf: str = Field(default="assets/urdfs/pawn.urdf", description="URDF file path for the pawn")
    pawn_black_urdf: str = Field(default="assets/urdfs/pawn_b.urdf", description="URDF file path for the pawn")
    
    rook_white_urdf: str = Field(default="assets/urdfs/rook.urdf", description="URDF file path for the rook")
    rook_black_urdf: str = Field(default="assets/urdfs/rook_b.urdf", description="URDF file path for the rook")
    
    knight_white_urdf: str = Field(default="assets/urdfs/knight.urdf", description="URDF file path for the knight")
    knight_black_urdf: str = Field(default="assets/urdfs/knight_b.urdf", description="URDF file path for the knight")
    
    bishop_white_urdf: str = Field(default="assets/urdfs/bishop.urdf", description="URDF file path for the bishop")
    bishop_black_urdf: str = Field(default="assets/urdfs/bishop_b.urdf", description="URDF file path for the bishop")
    
    queen_white_urdf: str = Field(default="assets/urdfs/queen.urdf", description="URDF file path for the queen")
    queen_black_urdf: str = Field(default="assets/urdfs/queen_b.urdf", description="URDF file path for the queen")
    
    king_white_urdf: str = Field(default="assets/urdfs/king.urdf", description="URDF file path for the king")
    king_black_urdf: str = Field(default="assets/urdfs/king_b.urdf", description="URDF file path for the king")
    
    
    pawn_start_pos: List[float] = Field(default=[0.265, -0.2, 0.02], min_items=3, max_items=3, description="Pawn starting position [x, y, z]")
    pawn_target_pos: List[float] = Field(default=[0.7, 0, 0.02], min_items=3, max_items=3, description="Pawn target position [x, y, z]")

    white_pawns: List = Field(default_factory=list)
    black_pawns: List = Field(default_factory=list)
    
    piece_mass: float = Field(default=0.08, gt=0.0, description="Mass of a single chess piece")
    piece_height: float = Field(default=0.05, gt=0.0, description="Height of the chess piece for grasping calculations")
    piece_radius: float = Field(default=0.02, gt=0.0, description="Radius of the chess piece for grasping calculations")
    piece_lateral_friction: float = Field(default=1.0, gt=0.0, description="Lateral friction for the chess pieces")
    piece_spinning_friction: float = Field(default=0.001, gt=0.0, description="Spinning friction for the chess pieces")
    piece_rolling_friction: float = Field(default=0.001, gt=0.0, description="Rolling friction for the chess pieces")
    piece_restitution: float = Field(default=0.0, ge=0.0, description="Restitution (bounciness) for the chess pieces")
    piece_contact_damping: float = Field(default=0.1, gt=0.0, description="Contact damping for the chess pieces")
    piece_contact_stiffness: float = Field(default=1000.0, gt=0.0, description="Contact stiffness for the chess pieces")
    piece_color: List[float] = Field(default=[1, 1, 1, 1], min_items=4, max_items=4, description="Default RGBA color for the chess pieces")



class PickPlaceConfig(BaseModel):
    clearance_z: float = Field(default=0.12, gt=0.0, description="Clearance height above surface for safe moves")
    pick_z_offset: float = Field(default=0.015, gt=0.0, description="Z offset above object for grasping")
    place_z_offset: float = Field(default=0.015, gt=0.0, description="Z offset above target for placing")
      # Define tolerances - consider making these config parameters
    xy_tolerance: float = Field(default=0.015, gt=0.0, description="1cm tolerance for XY")
    z_tolerance: float = Field(default=0.02, gt=0.0, description="2cm tolerance for Z")

    # Store Euler angles, convert to quaternion after loading
    ee_down_orientation_euler: List[float] = Field(default=[3.141592653589793, 0, 0], min_items=3, max_items=3, description="EE orientation (Euler angles [roll, pitch, yaw])")
    ee_down_orientation: List[float] = Field(default=None, description="EE orientation (Quaternion [x, y, z, w])") # Will be set after loading

    def model_post_init(self, __context):
        """Convert Euler angles to quaternion after model initialization."""
        self.ee_down_orientation = p.getQuaternionFromEuler(self.ee_down_orientation_euler)



class TaskConfig(BaseModel):
    max_retries: int = Field(default=2, ge=0, description="Maximum number of retries for pick/place")
    




class PlanningConfig(BaseModel):
    max_retries: int = Field(default=3, ge=0, description="Maximum number of retries for planning")
    safety_distance_threshold: float = Field(default=0.05, gt=0.0, description="Safety distance threshold for direct path check")
    ompl_validity_checking_resolution: float = Field(default=0.05, gt=0.0, description="OMPL validity checking resolution")
    ompl_path_interpolation_steps: int = Field(default=15, ge=0, description="OMPL path interpolation steps")
    
    
    
    
    
class Config(BaseModel):
    simulation: SimulationConfig
    robot: RobotConfig
    environment: EnvironmentConfig
    object: ObjectConfig
    pick_place: PickPlaceConfig
    task: TaskConfig
    planning: PlanningConfig

    def update_ee_orientation(self):
        """Update the end-effector orientation quaternion after loading Euler angles."""
        # This is now handled in PickPlaceConfig.model_post_init
        pass





config = Config(
    simulation=SimulationConfig(**CONFIG.get("simulation", {})),
    robot=RobotConfig(**CONFIG.get("robot", {})),
    environment=EnvironmentConfig(**CONFIG.get("environment", {})),
    object=ObjectConfig(**CONFIG.get("object", {})),
    pick_place=PickPlaceConfig(**CONFIG.get("pick_place", {})),
    task=TaskConfig(**CONFIG.get("task", {})),
    planning=PlanningConfig(**CONFIG.get("planning", {})),
)