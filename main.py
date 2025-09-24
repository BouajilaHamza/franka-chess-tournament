import pybullet as p
import time
import logging
from simulation.builder import EnvironmentBuilder
from simulation.robot_controller import RobotController

from configs.config import config

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
# Uncomment the line below if you find OMPL or other PyBullet logs too noisy
# logging.getLogger("pybullet").setLevel(logging.WARNING)



def main():
    """Main function to run the simulation."""

    builder = EnvironmentBuilder()
    env_components = (builder
                      .connect()
                      .load_ground_plane()
                      .load_robots()
                      .load_chess_board()
                      .load_pieces() # This calls _define_square_mapping internally
                      .build()
                      )
    logger.info(env_components)

    robot1_id = env_components['robot1']['id']
    robot1_arm_joints = env_components['robot1']['arm_joints']
    robot1_gripper_joints = env_components['robot1']['gripper_joints']
    robot2_id = env_components['robot2']['id']
    pawn_id  = list(env_components['piece_ids'].items())[0][0]
    start_pos = env_components['square_to_world_coords']['e7']
    target_pos = env_components['square_to_world_coords']['e5']

    start_pos_r2 = env_components['square_to_world_coords']['e2']
    target_pos_r2 = env_components['square_to_world_coords']['e4']


    robot1_controller = RobotController(robot1_id, robot1_arm_joints, robot1_gripper_joints, config.robot.first.end_effector_index, "Robot1")
    robot1_controller.move_to_home_position()
    robot1_controller.pick_and_place_with_retry(pawn_id, start_pos, target_pos)
    robot1_controller.move_to_home_position()

    robot2_controller = RobotController(robot2_id, env_components['robot2']['arm_joints'], env_components['robot2']['gripper_joints'], config.robot.second.end_effector_index, "Robot2")
    robot2_controller.move_to_home_position()
    logger.info(f"Start position for Robot2: {start_pos_r2}, Target position for Robot2: {target_pos_r2}")
    robot2_controller.pick_and_place_with_retry(pawn_id, start_pos_r2, target_pos_r2)
    robot2_controller.move_to_home_position()
    
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