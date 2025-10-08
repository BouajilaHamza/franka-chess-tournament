import time
import chess
import logging
from typing import Dict, Any
from datetime import datetime


from configs.config import config
from configs.logger_config import MetricsLoggerSQLModel
from ui.schemas import MoveData

from simulation.chess_engine import ChessEngine
from simulation.robot_controller import RobotController
from simulation.perception import find_piece_id_at_square
from simulation.special_moves import handle_kingside_castling, handle_queenside_castling,handle_captured_piece


logger = logging.getLogger(__name__)

def run_game_loop(
    env_components: Dict[str, Any],
    robot_controllers: Dict[str, RobotController],
    chess_engine: ChessEngine,
    metrics_logger: MetricsLoggerSQLModel,
    move_log_data: MoveData,
    pid_to_piece_type: Dict[int, str] = None
):
    """
    Main game loop integrating perception, decision (engine), and control (robot).
    Orchestrates the flow, delegating specific tasks to helper modules.
    """
    logger.info("Starting main game loop...")

    square_to_world = env_components['square_to_world_coords']
    piece_info = env_components['piece_ids']
    all_piece_ids_set = set(piece_info.keys()) if isinstance(piece_info, dict) else set(piece_info)

    game_over = False
    move_count = 0
    max_moves = config.task.max_moves

    while not game_over and move_count < max_moves:
        move_count += 1
        logger.info(f"--- Game Turn {move_count} ---")

        # 1. Determine turn and controller
        current_turn_color = chess_engine.get_internal_board().turn
        controller_to_move = robot_controllers['white'] if current_turn_color == chess.WHITE else robot_controllers['black']
        robot_name = controller_to_move.name
        robot_color = "white" if current_turn_color == chess.WHITE else "black"
        logger.info(f"It's {robot_name}'s turn (Color: {robot_color}).")


        robot_move_uci = chess_engine.get_robot_move()
        if robot_move_uci is None:
            logger.info("Engine returned no move. Game might be over.")

            game_over = True
            break


        board_before_move = chess_engine.get_internal_board()
        san_move_str = board_before_move.san(robot_move_uci)
        start_square_name = chess.square_name(robot_move_uci.from_square)
        target_square_name = chess.square_name(robot_move_uci.to_square)
        logger.info(f"{robot_name} is commanded to move: {start_square_name} -> {target_square_name} (SAN: {san_move_str})")


        is_kingside_castle = board_before_move.is_kingside_castling(robot_move_uci)
        is_queenside_castle = board_before_move.is_queenside_castling(robot_move_uci)

        if is_kingside_castle or is_queenside_castle:
            logger.info(f"{robot_name} detected castling move: {san_move_str}")
            

            move_log_data.move_number=move_count
            move_log_data.robot_name=robot_name
            move_log_data.robot_color=robot_color
            move_log_data.source_square=start_square_name
            move_log_data.target_square=target_square_name
            move_log_data.timestamp=datetime.now()


            # --- DELEGATE TO SPECIAL MOVES MODULE ---
            if is_kingside_castle:
                castling_success = handle_kingside_castling(
                    robot_controller=controller_to_move,
                    board_before_move=board_before_move,
                    king_move_uci=robot_move_uci,
                    env_components=env_components,
                    move_log_data=move_log_data
                )
            else: # is_queenside_castle
                castling_success = handle_queenside_castling(
                    robot_controller=controller_to_move,
                    board_before_move=board_before_move,
                    king_move_uci=robot_move_uci,
                    env_components=env_components,
                    move_log_data=move_log_data
                )

            # Update engine and log result
            if castling_success:
                chess_engine.update_internal_board(robot_move_uci)
                move_log_data.success = True
                logger.info(f"{robot_name} successfully executed castling.")
            else:
                move_log_data.success = False
                logger.error(f"{robot_name} failed to execute castling.")
                game_over = True # Or implement retry logic

            metrics_logger.log_move(move_log_data)
            controller_to_move.move_to_home_position()
            continue # Skip normal move logic

        # 5. --- Normal Move Handling ---
        logger.info(f"{robot_name} executing normal move: {start_square_name} -> {target_square_name}")

        # Get world coordinates
        try:
            start_pos_world = square_to_world[start_square_name]
            target_pos_world = square_to_world[target_square_name]
        except KeyError as e:
            logger.error(f"Square coordinate error: {e}")
            game_over = True
            break

        # --- DELEGATE TO PERCEPTION MODULE ---
        piece_id_to_move = find_piece_id_at_square(start_square_name, all_piece_ids_set, square_to_world)
        if piece_id_to_move is None:
            logger.error(f"Could not find piece at {start_square_name}.")
            game_over = True
            break

        # Prepare MoveData log object for the normal move
        normal_move_log_data = MoveData(
            move_number=move_count,
            robot_name=robot_name,
            robot_color=robot_color,
            source_square=start_square_name,
            target_square=target_square_name,
            piece_id=piece_id_to_move,
            piece_type=pid_to_piece_type.get(piece_id_to_move, "unknown"),
            timestamp=datetime.now(),

        )

        # Execute normal move using the controller
        success = controller_to_move.pick_and_place_with_retry(
            object_id=piece_id_to_move,
            start_pos=start_pos_world,
            target_pos=target_pos_world,
            move_log_data=normal_move_log_data # Pass log data
        )


        if success:
            # Call the function right after a successful placement
            capture_handled = handle_captured_piece(
                target_square_name=target_square_name,
                attacker_piece_id=piece_id_to_move, # The piece that just moved and captured
                env_components=env_components,
                robot_controller=controller_to_move # Use the controller that just moved, or decide on a specific one
            )
            if not capture_handled:
                logger.warning(f"Failed to handle potential capture on {target_square_name}.")
                # Decide if this is critical enough to stop the game or just log the issue
                # game_over = True # Uncomment if capture failure is critical
            # --- END NEW ---
            chess_engine.update_internal_board(robot_move_uci)
            normal_move_log_data.success = True
            logger.info(f"{robot_name} successfully executed normal move.")
        else:
            normal_move_log_data.success = False
            logger.error(f"{robot_name} failed to execute normal move.")
            game_over = True # Or implement retry logic

        metrics_logger.log_move(normal_move_log_data)
        controller_to_move.move_to_home_position()

        time.sleep(config.simulation.step_delay) # Use config

    logger.info("Game loop finished.")
