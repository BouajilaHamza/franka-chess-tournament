import pybullet as p
import time
import logging
from simulation.builder import EnvironmentBuilder
from simulation.robot_controller import RobotController
from simulation.chess_engine import ChessEngine
from simulation import game_logic
from configs.config import config

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
# Uncomment the line below if you find OMPL or other PyBullet logs too noisy
# logging.getLogger("pybullet").setLevel(logging.WARNING)




def main():
    """Main function to run the chess-playing simulation loop."""
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
        config.robot.first.end_effector_index, "Robot1 (Black)"
    )
    robot2_controller = RobotController(
        robot2_id, robot2_arm_joints, robot2_gripper_joints,
        config.robot.second.end_effector_index, "Robot2 (White)"
    )

    robot1_controller.move_to_home_position()
    robot2_controller.move_to_home_position()

    # --- 3. Setup Chess Engine ---
    try:
        engine = ChessEngine(stockfish_path="stockfish") # Adjust path if needed
        engine.initialize_engine()
        engine.reset_game() # Ensure the engine starts from the standard position
    except FileNotFoundError as e:
        logger.error(f"Stockfish not found: {e}")
        logger.error("Install stockfish using 'sudo apt install stockfish' or set correct path.")
        return
    except Exception as e:
        logger.error(f"Error initializing ChessEngine: {e}")
        return

    # --- 4. Start the Game Loop ---
    # Pass necessary components to the game logic function
    game_logic.run_game_loop(
        env_components=env_components,
        robot_controllers={'black': robot1_controller, 'white': robot2_controller},
        chess_engine=engine
    )


    logger.info("Game loop ended.")
    engine.close_engine() # Close the Stockfish process
    try:
        while True: # Keep the simulation window open to view the final state
            p.stepSimulation()
            time.sleep(config.simulation.step_delay)
    except KeyboardInterrupt:
        logger.info("Simulation stopped by user.")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()