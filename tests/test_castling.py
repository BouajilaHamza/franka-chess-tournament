# tests/test_castling.py
import time
import logging
import pybullet as p
import chess # Import python-chess
from simulation.builder import EnvironmentBuilder
from simulation.robot_controller import RobotController # Ensure this points to your actual controller
from simulation.chess_engine import ChessEngine # Adjust import path if needed
from simulation.game_logic import handle_kingside_castling # Adjust import path if needed

from configs.config import config # Assuming config is accessible
from configs.logger_config import MetricsLoggerSQLModel # Adjust import path if needed

# Import your Pydantic/SQLModel schemas
# --- CHANGED: Use ui.schemas for Pydantic models ---
from ui.schemas import MoveData, ExperimentData # Adjust import path if needed

import numpy as np

logger = logging.getLogger(__name__)
# logging.basicConfig(level=logging.INFO) # Consider configuring in main entry point

def test_kingside_castling(color: str = "white"):
    """
    Test function for executing a kingside castling move without full game logic.
    Sets up the environment to allow castling by clearing relevant squares.
    Assumes pieces are loaded in starting positions, minus those blocking castling.
    """
    physics_client = None
    engine = None
    try:
        # --- 1. Environment setup (MODIFIED to clear castling squares) ---
        logger.info("Setting up environment for castling test...")
        builder = EnvironmentBuilder()
        
        # --- DETERMINE SQUARES TO SKIP FOR CASTLING ---
        # Define the squares that need to be empty for kingside castling
        castling_squares_to_clear = []
        if color.lower() == "white":
            castling_squares_to_clear = ['f1', 'g1'] # Clear f1 and g1 for White kingside O-O
            logger.info("Clearing squares f1 and g1 for White kingside castling setup.")
        elif color.lower() == "black":
            castling_squares_to_clear = ['f8', 'g8'] # Clear f8 and g8 for Black kingside O-O
            logger.info("Clearing squares f8 and g8 for Black kingside castling setup.")
        else:
            logger.error(f"Invalid color '{color}' for castling test. Use 'white' or 'black'.")
            return False
        # --- END DETERMINING SQUARES ---

        env_components = (builder
                         .connect()
                         .load_ground_plane()
                         .load_robots()
                         .load_chess_board()
                         # --- MODIFIED CALL: Pass skip_squares ---
                         .load_pieces(skip_squares=castling_squares_to_clear) # Clear relevant squares
                         # --- END MODIFIED CALL ---
                         .build()
                         )
        physics_client = builder.physics_client
        logger.info("Environment built successfully for castling test (castling squares cleared).")

        # --- 2. Setup Controllers ---
        # Extract necessary components from env_components
        robot1_id = env_components['robot1']['id']
        robot1_arm_joints = env_components['robot1']['arm_joints']
        robot1_gripper_joints = env_components['robot1']['gripper_joints']
        robot2_id = env_components['robot2']['id']
        robot2_arm_joints = env_components['robot2']['arm_joints']
        robot2_gripper_joints = env_components['robot2']['gripper_joints']
        pid_to_piece_type = env_components['piece_id_to_piece_type']
        all_piece_ids = env_components['piece_ids']
        square_to_world = env_components['square_to_world_coords']

        # Determine which robot plays which color
        # --- ADJUST THIS LOGIC BASED ON YOUR ACTUAL ROBOT-COLOR ASSIGNMENT ---
        # Assuming Robot1 is Black, Robot2 is White (common, but verify in main.py)
        if color.lower() == "white":
            controller_to_test = RobotController(
                robot2_id, robot2_arm_joints, robot2_gripper_joints,
                config.robot.second.end_effector_index, "Robot2 (White)", all_obstacle_ids=all_piece_ids
            )
            robot_name = "Robot2 (White)"
            # Optional: Get opponent controller if needed for anything
            # opponent_controller = RobotController(
            #     robot1_id, robot1_arm_joints, robot1_gripper_joints,
            #     config.robot.first.end_effector_index, "Robot1 (Black)", all_obstacle_ids=all_piece_ids
            # )
        elif color.lower() == "black":
            controller_to_test = RobotController(
                robot1_id, robot1_arm_joints, robot1_gripper_joints,
                config.robot.first.end_effector_index, "Robot1 (Black)", all_obstacle_ids=all_piece_ids
            )
            robot_name = "Robot1 (Black)"
            # Optional: Get opponent controller if needed for anything
            # opponent_controller = RobotController(
            #     robot2_id, robot2_arm_joints, robot2_gripper_joints,
            #     config.robot.second.end_effector_index, "Robot2 (White)", all_obstacle_ids=all_piece_ids
            # )
        else:
            logger.error(f"Invalid color '{color}' for castling test. Use 'white' or 'black'.")
            return False

        logger.info(f"Using {robot_name} for {color.capitalize()} kingside castling test.")

        # Move robot to home position
        logger.info(f"Moving {robot_name} to home position...")
        controller_to_test.move_to_home_position()
        logger.info(f"{robot_name} at home.")

        # --- 3. Setup Chess Engine ---
        logger.info("Initializing Chess Engine for castling test...")
        engine = ChessEngine(stockfish_path="stockfish") # Adjust path if needed
        engine.initialize_engine()
        engine.reset_game() # Ensure the engine starts from the standard position
        logger.info("Chess Engine initialized and reset to starting position.")

        # --- 4. Prepare for Castling ---
        board_state = engine.get_internal_board()
        logger.info(f"Engine board state for castling test: {board_state.fen()}")

        # --- CRITICAL FIX: Check Castling Rights Correctly ---
        # Use python-chess's built-in methods to check castling rights
        # The board should now reflect the cleared squares, allowing castling.
        if color.lower() == "white":
            can_castle_kingside = board_state.has_castling_rights(chess.WHITE) and \
                                  board_state.has_kingside_castling_rights(chess.WHITE)
            logger.info(f"White kingside castling available in starting position: {can_castle_kingside}")
        else: # Black
            can_castle_kingside = board_state.has_castling_rights(chess.BLACK) and \
                                  board_state.has_kingside_castling_rights(chess.BLACK)
            logger.info(f"Black kingside castling available in starting position: {can_castle_kingside}")

        if not can_castle_kingside:
            logger.error(f"Kingside castling not available for {color} in the starting position (after clearing squares).")
            return False # Exit early if castling is still not possible

        # Create the UCI move object for kingside castling
        # The engine will generate this move if queried when it's the correct turn.
        # However, for a direct test, we can create it.
        if color.lower() == "white":
            uci_castle_move_str = "e1g1" # White kingside castling UCI
        else: # Black
            uci_castle_move_str = "e8g8" # Black kingside castling UCI

        try:
            castle_move_uci = chess.Move.from_uci(uci_castle_move_str)
            san_move_str = board_state.san(castle_move_uci)
            logger.info(f"Generated castling move UCI: {uci_castle_move_str}, SAN: {san_move_str}")
        except ValueError as e:
            logger.error(f"Error creating castling move '{uci_castle_move_str}': {e}")
            return False

        # --- 5. Setup Logging for the Test Move ---
        metrics_logger = MetricsLoggerSQLModel()
        # Ensure experiment is started if your logger requires it
        # experiment_id = metrics_logger.start_experiment(name="Castling_Test", notes=f"Testing {color} kingside castling")
        
        # Create ExperimentData instance
        experiment_data = ExperimentData(
            name=f"Castling_Test_{color}_{int(time.time())}",
            start_time=time.strftime('%Y-%m-%d %H:%M:%S'),
            status="running",
            notes=f"Automated test for {color} kingside castling with cleared squares."
        )
        # You might need to save this experiment to DB first if MoveData requires an experiment_id
        # experiment_id = metrics_logger.log_experiment(experiment_data) # Hypothetical method

        test_move_log_data = MoveData()

        # --- 6. Execute Castling using Handler ---
        logger.info(f"--- Starting {color.capitalize()} Kingside Castling Test ---")
        test_start_time = time.time()

        # Call the castling handler function you implemented
        # It needs:
        # - robot_controller: The controller for the robot performing the move
        # - board_before_move: The python-chess.Board state *before* the move (our current state)
        # - king_move_uci: The UCI move object for the king's part of the castle (e1g1 or e8g8)
        # - env_components: To access square_to_world_coords, piece_id_to_piece_type, etc.
        # - move_log_data: The MoveData object to log results to
        success = handle_kingside_castling(
            robot_controller=controller_to_test,
            board_before_move=board_state, # Pass the current board state
            king_move_uci=castle_move_uci, # Pass the UCI move object for the king's move
            env_components=env_components, # Pass environment details
            move_log_data=test_move_log_data # Pass the log object
        )

        test_end_time = time.time()
        test_duration = test_end_time - test_start_time
        test_move_log_data.total_time_seconds = test_duration
        test_move_log_data.success = success # Record the outcome from the handler

        # --- 7. Log Results ---
        logger.info(f"Kingside Castling Test Result for {color}: {'SUCCESS' if success else 'FAILURE'}")
        # Update experiment status before logging
        experiment_data.end_time = time.strftime('%Y-%m-%d %H:%M:%S')
        experiment_data.status = "completed" if success else "failed"
        # Log the move (this will likely also log the experiment if linked correctly)
        logged_test_id = metrics_logger.log_move(test_move_log_data)
        if logged_test_id:
            logger.info(f"Castling test logged to DB with ID {logged_test_id}")
        else:
            logger.warning("Failed to log castling test to DB.")

        # --- 8. Final Observation Pause ---
        logger.info("Castling test completed. Pausing to observe final state in simulation.")
        try:
            for _ in range(int(2.0 / config.simulation.step_delay)): # Pause for ~2 seconds
                p.stepSimulation()
                time.sleep(config.simulation.step_delay)
        except KeyboardInterrupt:
            logger.info("Observation paused by user.")

        return success # Indicate if the test handler reported success

    except Exception as e:
        logger.exception(f"Unexpected error during kingside castling test: {e}")
        # Log the error to the database if possible
        if 'test_move_log_data' in locals() and 'metrics_logger' in locals():
             test_move_log_data.success = False
             test_move_log_data.failure_type = "Exception"
             # Add error message if you have a field for it
             metrics_logger.log_move(test_move_log_data)
        return False # Indicate test failed to run properly
    finally:
        # --- 9. Cleanup ---
        logger.info("Cleaning up castling test...")
        if engine:
            try:
                engine.close_engine()
                logger.info("Chess engine closed.")
            except Exception as e:
                logger.warning(f"Error closing chess engine: {e}")
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

    # Test White Kingside Castling
    logger.info("===== TESTING WHITE KINGSIDE CASTLING =====")
    white_success = test_kingside_castling(color="white")
    logger.info(f"White Kingside Castling Test: {'PASSED' if white_success else 'FAILED'}")

    # Test Black Kingside Castling (optional, run separately or add logic)
    # logger.info("===== TESTING BLACK KINGSIDE CASTLING =====")
    # black_success = test_kingside_castling(color="black")
    # logger.info(f"Black Kingside Castling Test: {'PASSED' if black_success else 'FAILED'}")
