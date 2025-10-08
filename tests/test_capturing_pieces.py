# tests/test_capture_handling.py
import time
import logging
import pybullet as p
import numpy as np
from simulation.builder import EnvironmentBuilder
from simulation.robot_controller import RobotController
from simulation.game_logic import handle_captured_piece
from configs.config import config# DEAD_PIECES_AREA_CENTER, DEAD_PIECES_AREA_SIZE
from configs.logger_config import MetricsLoggerSQLModel
from ui.schemas import MoveData, ExperimentData

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO) # Configure logging for this script



def test_capture_handling(target_square_name="e4"):
    """
    Test function for handling captured pieces.
    Simulates a scenario where a piece is on a square and needs to be moved out of the way.
    """
    physics_client = None
    engine = None
    try:
        # --- 1. Environment setup ---
        logger.info("Setting up environment for capture handling test...")
        builder = EnvironmentBuilder()
        # Load the full environment, including all pieces
        env_components = (builder
                         .connect()
                         .load_ground_plane()
                         .load_robots()
                         .load_chess_board()
                         .load_pieces() # Load ALL pieces initially
                         .build()
                         )
        physics_client = builder.physics_client
        logger.info("Environment built successfully for capture handling test.")

        # --- 2. Setup Controllers ---
        robot1_id = env_components['robot1']['id']
        robot1_arm_joints = env_components['robot1']['arm_joints']
        robot1_gripper_joints = env_components['robot1']['gripper_joints']
        robot2_id = env_components['robot2']['id']
        robot2_arm_joints = env_components['robot2']['arm_joints']
        robot2_gripper_joints = env_components['robot2']['gripper_joints']
        pid_to_piece_type = env_components['piece_id_to_piece_type']
        all_piece_ids = env_components['piece_ids']
        square_to_world = env_components['square_to_world_coords']

        # Use Robot2 (White) for this test
        controller_to_test = RobotController(
            robot2_id, robot2_arm_joints, robot2_gripper_joints,
            config.robot.second.end_effector_index, "Robot2 (White)", all_obstacle_ids=all_piece_ids
        )
        robot_name = "Robot2 (White)"
        logger.info(f"Using {robot_name} for capture handling test.")

        # Move robot to home position
        logger.info(f"Moving {robot_name} to home position...")
        controller_to_test.move_to_home_position()
        logger.info(f"{robot_name} at home.")

        # --- 3. Identify Pieces for Test ---
        # Find a piece that is *currently* on the target square
        # We'll use the logic from handle_captured_piece to find it
        target_pos = square_to_world.get(target_square_name)
        if target_pos is None:
            logger.error(f"Target square {target_square_name} not found in mapping.")
            return False

        # --- Find the piece ID on the target square ---
        TOLERANCE = 0.02 # 2cm tolerance, adjust based on piece/bucket size
        piece_id_on_square = None
        for piece_id in all_piece_ids.keys():
            try:
                piece_pos, _ = p.getBasePositionAndOrientation(piece_id)
                distance = np.linalg.norm(np.array(piece_pos[:2]) - np.array(target_pos[:2])) # Compare X,Y only
                if distance <= TOLERANCE:
                    piece_id_on_square = piece_id
                    logger.info(f"Found piece ID {piece_id_on_square} ({pid_to_piece_type.get(piece_id_on_square, 'Unknown')}) on square {target_square_name} (distance: {distance:.4f}m)")
                    break
            except p.error:
                # Piece might have been removed already
                continue

        if piece_id_on_square is None:
            logger.error(f"No piece found on square {target_square_name} to simulate capture.")
            return False

        # --- 4. Define Attacker Piece (Just for logging, not physically moved in this test) ---
        attacker_piece_id = next((pid for pid in all_piece_ids.keys() if pid != piece_id_on_square), None)
        if attacker_piece_id is None:
            logger.error("Could not find another piece to act as the 'attacker' for logging.")
            return False
        logger.info(f"Simulating attacker piece ID: {attacker_piece_id}")

        # --- 5. Setup Logging for the Test ---
        metrics_logger = MetricsLoggerSQLModel()
        # Ensure experiment is started if your logger requires it
        # experiment_id = metrics_logger.start_experiment(name="Capture_Handling_Test", notes=f"Testing capture handling on square {target_square_name}")
        
        experiment_data = ExperimentData(
            name=f"Capture_Handling_Test_{target_square_name}_{int(time.time())}",
            start_time=time.strftime('%Y-%m-%d %H:%M:%S'),
            status="running",
            notes=f"Automated test for handling capture on square {target_square_name}."
        )

        test_move_log_data = MoveData()


        logger.info(f"--- Starting Capture Handling Test on Square {target_square_name} ---")
        test_start_time = time.time()

        success = handle_captured_piece(
            target_square_name=target_square_name,
            attacker_piece_id=attacker_piece_id, # The piece that "did" the capturing (for logging/context)
            env_components=env_components, # Pass environment details
            robot_controller=controller_to_test # Pass the controller to move the captured piece
        )

        test_end_time = time.time()
        test_duration = test_end_time - test_start_time
        test_move_log_data.total_time_seconds = test_duration
        test_move_log_data.success = success # Record the outcome from the handler

        # --- 7. Log Results ---
        logger.info(f"Capture Handling Test Result for square {target_square_name}: {'SUCCESS' if success else 'FAILURE'}")
        # Update experiment status before logging
        experiment_data.end_time = time.strftime('%Y-%m-%d %H:%M:%S')
        experiment_data.status = "completed" if success else "failed"
        # Log the move (this will likely also log the experiment if linked correctly)
        logged_test_id = metrics_logger.log_move(test_move_log_data)
        if logged_test_id:
            logger.info(f"Capture handling test logged to DB with ID {logged_test_id}")
        else:
            logger.warning("Failed to log capture handling test to DB.")

        # --- 8. Final Observation Pause ---
        logger.info("Capture handling test completed. Pausing to observe final state in simulation.")
        try:
            for _ in range(int(2.0 / config.simulation.step_delay)): # Pause for ~2 seconds
                p.stepSimulation()
                time.sleep(config.simulation.step_delay)
        except KeyboardInterrupt:
            logger.info("Observation paused by user.")

        return success # Indicate if the test handler reported success

    except Exception as e:
        logger.exception(f"Unexpected error during capture handling test: {e}")
        # Log the error to the database if possible
        if 'test_move_log_data' in locals() and 'metrics_logger' in locals():
             test_move_log_data.success = False
             test_move_log_data.failure_type = "Execution"
             # Add error message if you have a field for it
             metrics_logger.log_move(test_move_log_data)
        return False # Indicate test failed to run properly
    finally:
        # --- 9. Cleanup ---
        logger.info("Cleaning up capture handling test...")
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

    # Test Capture Handling on a specific square (e.g., e4)
    logger.info("===== TESTING CAPTURE HANDLING ON SQUARE e4 =====")
    capture_success = test_capture_handling(target_square_name="e7")
    logger.info(f"Capture Handling Test on e4: {'PASSED' if capture_success else 'FAILED'}")

    # Test Capture Handling on another square (optional, run separately or add logic)
    # logger.info("===== TESTING CAPTURE HANDLING ON SQUARE d5 =====")
    # capture_success_d5 = test_capture_handling(target_square_name="d5")
    # logger.info(f"Capture Handling Test on d5: {'PASSED' if capture_success_d5 else 'FAILED'}")
