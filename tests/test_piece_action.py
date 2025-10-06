# test_single_piece_action.py (or integrated into your existing test file)
import time
import logging
import pybullet as p
from simulation.builder import EnvironmentBuilder
from simulation.robot_controller import RobotController # Ensure this points to your actual controller
from configs.config import config
from configs.logger_config import MetricsLoggerSQLModel # Ensure this points to your actual logger
from ui.schemas import MoveData # Ensure this points to your actual schema
import numpy as np # Import for norm calculation

logger = logging.getLogger(__name__)
# logging.basicConfig(level=logging.INFO) # Consider configuring this in your main entry point

def test_single_piece_action(piece_name: str, target_square: str = "e4", start_square: str = None):
    """
    Test function for manipulating a single chess piece without game logic.
    Args:
        piece_name (str): Name of piece to test, e.g., 'pawn_w', 'rook_b', 'bishop_w'.
        target_square (str): Target square for placement (e.g., 'e4').
        start_square (str, optional): Starting square name. If None, attempts to find it.
    Returns:
        bool: True if the test completed (success or logged failure), False on setup error.
    """
    physics_client = None
    try:
        # --- 1. Environment setup ---
        builder = EnvironmentBuilder()
        env = (builder
               .connect()
               .load_ground_plane()
               .load_robots()
               .load_chess_board()
               .load_pieces()
               .build())
        physics_client = builder.physics_client # Get client ID for cleanup
        logger.info("Environment ready for isolated test.")

        robot_id = env['robot2']['id'] # Assuming testing with robot2
        arm_joints = env['robot2']['arm_joints']
        gripper_joints = env['robot2']['gripper_joints']
        square_to_world = env['square_to_world_coords']
        pid_to_type = env['piece_id_to_piece_type'] # Assuming this mapping exists
        all_piece_ids = env['piece_ids']

        # Validate target square
        if target_square not in square_to_world:
             logger.error(f"Target square '{target_square}' not found in mapping.")
             return False

        robot = RobotController(
            robot_id, arm_joints, gripper_joints,
            config.robot.first.end_effector_index,
            "Robot2-Test", all_obstacle_ids=all_piece_ids
        )

        # Move to home to ensure a known starting state
        logger.info("Moving robot to home position...")
        robot.move_to_home_position()
        logger.info("Robot at home.")

        metrics_logger = MetricsLoggerSQLModel()
        # Ensure experiment is started if your logger requires it
        # experiment_id = metrics_logger.start_experiment(name="Single_Piece_Test", notes=f"Testing {piece_name}")
        move_log = MoveData(
            robot_name="Robot2-Test",
            robot_color="neutral", # Or derive from piece_name if needed
            source_square=start_square or "unknown_start", # Will update if found
            target_square=target_square,
            piece_type=piece_name,
            timestamp=time.time(), # Or datetime.now()
            success=False,
            total_time_seconds=None,
            # ... initialize other fields as needed or let them default ...
        )

        # --- 2. Find a piece by name ---
        # Search for the piece ID matching the *exact* name provided
        piece_id = next((pid for pid, ptype in pid_to_type.items() if ptype.lower() == piece_name.lower()), None)
        if piece_id is None:
            logger.error(f"No piece found matching exact name '{piece_name}'. Available: {set(pid_to_type.values())}")
            move_log.success = False
            move_log.failure_type = "Setup_Error" # If you have such a field
            metrics_logger.log_move(move_log)
            return False # Indicate setup failure

        # --- 3. Determine Start Square (if not provided) ---
        if start_square is None:
            logger.info("Start square not provided, attempting to find it...")
            # Find the square closest to the piece's current position
            piece_pos, _ = p.getBasePositionAndOrientation(piece_id)
            piece_pos_np = np.array(piece_pos)
            min_distance = float('inf')
            found_start_square = None
            for sq_name, sq_pos in square_to_world.items():
                sq_pos_np = np.array(sq_pos)
                distance = np.linalg.norm(piece_pos_np[:2] - sq_pos_np[:2]) # Compare X,Y only
                if distance < min_distance:
                    min_distance = distance
                    found_start_square = sq_name
            start_square = found_start_square
            logger.info(f"Determined start square for {piece_name} (ID {piece_id}) as '{start_square}' (distance: {min_distance:.4f}m)")

        if start_square not in square_to_world:
             logger.error(f"Determined or provided start square '{start_square}' not found in mapping.")
             move_log.success = False
             move_log.failure_type = "Setup_Error"
             move_log.source_square = start_square # Log the problematic square
             metrics_logger.log_move(move_log)
             return False

        # Update move log with found/corrected start square
        move_log.source_square = start_square

        # Get world coordinates
        start_pos_world = square_to_world[start_square]
        target_pos_world = square_to_world[target_square]

        # Update piece position to its actual current location for grasp
        actual_start_pos, _ = p.getBasePositionAndOrientation(piece_id)
        logger.info(f"Testing piece: {piece_name} (ID: {piece_id}) | Start: {start_square} @ {actual_start_pos} | Target: {target_square} @ {target_pos_world}")
        move_log.piece_id = piece_id # Log the actual ID found

        # --- 4. Execute the Action ---
        start_time = time.time()
        # Use the actual piece position for grasping
        success = robot.pick_and_place_with_retry(
            object_id=piece_id,
            start_pos=actual_start_pos, # Use actual position
            target_pos=target_pos_world,
            move_log_data=move_log # Pass the log data object
        )
        end_time = time.time()
        if not success:
            success = False
        move_log.success = success
        move_log.total_time_seconds = end_time - start_time

        # --- 5. Log Results ---
        logged_move_id = metrics_logger.log_move(move_log)
        if logged_move_id:
             logger.debug(f"Test move metrics logged successfully with DB ID {logged_move_id}.")
        else:
             logger.warning("Failed to log test move metrics.")

        if success:
            logger.info(f"✅ Successfully moved {piece_name} from {start_square} to {target_square}.")
        else:
            logger.error(f"❌ Failed to move {piece_name} from {start_square} to {target_square}.")

        return True # Indicate test *ran* (success status is in move_log)

    except Exception as e:
        logger.exception(f"Unexpected error during single piece test: {e}")
        if 'move_log' in locals() and metrics_logger:
            # Attempt to log the error
            move_log.success = False
            move_log.failure_type = "Execution"
            # Add error message if you have a field for it, or log it separately
            metrics_logger.log_move(move_log)
        return False # Indicate test failed to run properly
    finally:
        # --- 6. Cleanup ---
        if physics_client is not None:
            try:
                p.disconnect(physics_client)
                logger.info("Disconnected from PyBullet.")
            except Exception as e:
                logger.warning(f"Error disconnecting from PyBullet: {e}")


# --- Example Usage ---
if __name__ == "__main__":
    # Configure logging for the test script itself
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    # Test moving a white Bishop
    # test_single_piece_action("bishop_w", target_square="e5", start_square="c1") # Specify start if known
    test_single_piece_action("bishop_w", target_square="e5",start_square="f1") # Let it find start

    # Test moving a white King
    # test_single_piece_action("king_w", target_square="g1", start_square="e1")
    test_single_piece_action("king_w", target_square="g1")

    # Test moving a white Pawn (ensure it's the right one, e.g., the one on e2)
    # test_single_piece_action("pawn_w", target_square="e4", start_square="e2")
    test_single_piece_action("pawn_w", target_square="e4") # Let it find start (likely e2)
