import pybullet as p
import time
import logging

from simulation import game_logic
from simulation.chess_engine import ChessEngine
from simulation.builder import EnvironmentBuilder
from simulation.robot_controller import RobotController

from configs.config import config
from configs.logger_config import MetricsLoggerSQLModel

from ui.database_setup import create_db_and_tables
from ui.schemas import MoveData


logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def main():
    """Main function to run the chess-playing simulation loop."""
    logger.info("Starting Metrics Logger and Setting Up Database...")
    create_db_and_tables()
    # experimend_log_data = ExperimentData()
    move_log_data = MoveData()
    # failure_detail_log_data = FailureDetail()

    logger.info("Starting Chess Robot Simulation...")

    # --- 1. Setup Environment ---
    builder = EnvironmentBuilder()
    env_components = (builder
                      .connect()
                      .load_ground_plane()
                      .load_robots()
                      .load_chess_board()
                      .load_pieces() # This calls _define_square_mapping internally
                      .build()
                      )
    logger.info("Environment built successfully.")

    robot1_id = env_components['robot1']['id']
    robot1_arm_joints = env_components['robot1']['arm_joints']
    robot1_gripper_joints = env_components['robot1']['gripper_joints']
    robot2_id = env_components['robot2']['id']
    robot2_arm_joints = env_components['robot2']['arm_joints']
    robot2_gripper_joints = env_components['robot2']['gripper_joints']


    robot1_controller = RobotController(
        robot1_id, robot1_arm_joints, robot1_gripper_joints,
        config.robot.first.end_effector_index, "Robot1 (Black)",all_obstacle_ids=env_components['piece_ids']
    )
    robot2_controller = RobotController(
        robot2_id, robot2_arm_joints, robot2_gripper_joints,
        config.robot.second.end_effector_index, "Robot2 (White)",all_obstacle_ids=env_components['piece_ids']
    )

    robot1_controller.move_to_home_position()
    robot2_controller.move_to_home_position()


    metrics_logger = MetricsLoggerSQLModel()

    import datetime
    experiment_name = f"Chess_Sim_Run_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    experiment_notes = "Initial run with SQLModel direct logging."

    # --- NEW: Start the Experiment Logging ---
    experiment_id = metrics_logger.start_experiment(name=experiment_name, notes=experiment_notes)
    if experiment_id is None:
        logger.error("Failed to start experiment logging. Aborting.")
        return # Or handle the error appropriately

    logger.info(f"Started logging experiment with ID: {experiment_id}")


    # --- 3. Setup Chess Engine ---
    try:
        engine = ChessEngine(stockfish_path="stockfish") # Adjust path if needed
        engine.initialize_engine()
        engine.reset_game() # Ensure the engine starts from the standard position
    except FileNotFoundError as e:
        logger.error(f"Stockfish not found: {e}")
        logger.error("Install stockfish using 'sudo apt install stockfish' or set correct path.")
        # --- NEW: Log experiment end on critical failure ---
        metrics_logger.end_experiment()
        return
    except Exception as e:
        logger.error(f"Error initializing ChessEngine: {e}")
        # --- NEW: Log experiment end on critical failure ---
        metrics_logger.end_experiment()
        return

    # --- 4. Start the Game Loop ---
    try:
        game_logic.run_game_loop(
            env_components=env_components,
            robot_controllers={'black': robot1_controller, 'white': robot2_controller},
            chess_engine=engine,
            metrics_logger=metrics_logger,
            move_log_data=move_log_data,
        )
    except Exception as e:
        logger.error(f"Unexpected error in game loop: {e}")
        metrics_logger.end_experiment()
        raise # Re-raise to propagate the error

    logger.info("Game loop ended.")


    metrics_logger.end_experiment()
    logger.info("Experiment logging ended.")

    engine.close_engine()
    try:
        while True:
            p.stepSimulation()
            time.sleep(config.simulation.step_delay)
    except KeyboardInterrupt:
        logger.info("Simulation stopped by user.")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()