# simulation/game_logic.py
import logging
import numpy as np
import chess
import pybullet as p
import time
from typing import Dict, Any
from datetime import datetime
from configs.logger_config import MetricsLoggerSQLModel
from simulation.robot_controller import RobotController
from simulation.chess_engine import ChessEngine

from ui.schemas import MoveData
logger = logging.getLogger(__name__)

# --- CHANGED: Update function signature to accept metrics_logger ---
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
    Assumes RobotController has methods like pick_and_place_with_retry and move_to_home_position.
    Now accepts a metrics_logger instance for performance tracking.
    """
    square_to_world = env_components['square_to_world_coords']
    piece_info = env_components['piece_ids'] # Could be dict {id: square} or list [id1, id2, ...]

    # Ensure piece_ids is a set of object IDs for obstacle checking
    if isinstance(piece_info, dict):
        all_piece_ids_set = set(piece_info.keys())
    elif isinstance(piece_info, (list, set)):
        all_piece_ids_set = set(piece_info)
    else:
        logger.error("env_components['piece_ids'] format not recognized.")
        return

    game_over = False
    move_count = 0
    max_moves = 100 # Safety limit

    # --- Main game loop ---
    while not game_over and move_count < max_moves:
        move_count += 1
        logger.info(f"--- Game Turn {move_count} ---")

        # 1. Check whose turn it is
        current_turn_color = chess_engine.get_internal_board().turn
        logger.info(f"Current turn according to engine: {'White' if current_turn_color == chess.WHITE else 'Black'}")

        # 2. Determine which controller to use
        if current_turn_color == chess.WHITE:
            controller_to_move: RobotController = robot_controllers['white']
            robot_name = "Robot1 (White)"
            robot_color = "white"
        else: # current_turn_color == chess.BLACK
            controller_to_move: RobotController = robot_controllers['black']
            robot_name = "Robot2 (Black)"
            robot_color = "black"

        logger.info(f"It's {robot_name}'s turn.")

        # 3. Get the move from the engine
        robot_move_uci = chess_engine.get_robot_move()

        if robot_move_uci is None:
            logger.info("Engine returned no move. Game might be over.")
            _board_state = chess_engine.get_internal_board()
            # ... (game over checks - unchanged) ...
            game_over = True
            break

        # 4. Convert UCI move to physical world coordinates
        start_square_uci = robot_move_uci.from_square
        target_square_uci = robot_move_uci.to_square
        start_square_name = chess.square_name(start_square_uci)
        target_square_name = chess.square_name(target_square_uci)

        logger.info(f"{robot_name} is commanded to move: {start_square_name} -> {target_square_name} (SAN: {chess_engine.get_internal_board().san(robot_move_uci)})")

        try:
            start_pos_world = square_to_world[start_square_name]
            target_pos_world = square_to_world[target_square_name]
        except KeyError as e:
            logger.error(f"Square {e} not found in mapping. Cannot execute move.")
            game_over = True
            break

        # 5. Find the piece ID to move
        piece_id_to_move = find_piece_id_at_square(start_square_name, all_piece_ids_set, square_to_world)
        if piece_id_to_move is None:
            logger.error(f"Could not find a piece at square {start_square_name} to move.")
            game_over = True
            break
        
        logger.info(f"Target Square World pose : {start_pos_world}")
        start_pos_world = p.getBasePositionAndOrientation(piece_id_to_move)[0]

        logger.info(f"{robot_name} executing move: Pick {piece_id_to_move} from {start_square_name}, place at {target_square_name}")
        logger.info(f"  -> Start Pos: {start_pos_world}, Target Piece Pos: {target_pos_world}")
        move_log_data.move_number=move_count
        move_log_data.robot_name=robot_name
        move_log_data.robot_color=robot_color
        move_log_data.source_square=start_square_name
        move_log_data.target_square=target_square_name
        move_log_data.piece_id=piece_id_to_move
        move_log_data.piece_type=pid_to_piece_type.get(piece_id_to_move, "unknown")
        move_log_data.timestamp=datetime.now()

   
        move_start_time = time.time()


        
        success = controller_to_move.pick_and_place_with_retry(
            object_id=piece_id_to_move,
            start_pos=start_pos_world,
            target_pos=target_pos_world,
            max_retries=getattr(controller_to_move, 'max_retries', 2),
            move_log_data=move_log_data
        )

        # --- NEW: Record end time and calculate total move time ---
        move_end_time = time.time()
        move_log_data.total_time_seconds = move_end_time - move_start_time


        
        if move_log_data.success is None:
             move_log_data.success = success

        logger.info(f"Logging metrics for move {move_count}: Success={move_log_data.success}")
        # The metrics_logger handles associating this with the current experiment
        logged_move_id = metrics_logger.log_move(move_log_data)
        if logged_move_id:
             logger.debug(f"Move {move_count} metrics logged successfully with DB ID {logged_move_id}.")
        else:
             logger.warning(f"Failed to log metrics for move {move_count}.")


        if not success:
            logger.error(f"{robot_name} failed to execute move {start_square_name} -> {target_square_name}. Stopping simulation.")
            # Optional: Add specific error handling/recovery attempts here
            game_over = True
            break
        else:
            # Controlled by pick_and_place_with_retry or separate logic
            controller_to_move.move_to_home_position()
            logger.info(f"{robot_name} successfully executed move {start_square_name} -> {target_square_name}")

        # 7. Update the chess engine's internal state
        chess_engine.update_internal_board(robot_move_uci)
        logger.info(f"Engine state updated after move: {chess_engine.get_internal_board().fen()}")

        # 8. Wait or handle human move if applicable
        time.sleep(0.5)

    logger.info("Game loop finished.")


# find_piece_id_at_square remains the same
def find_piece_id_at_square(square_name: str, all_piece_ids: set, square_to_world: dict, tolerance: float = 0.015) -> int | None:
    """
    Find the PyBullet object ID of the piece located at a given square name.
    Uses PyBullet's getBasePositionAndOrientation to check piece locations.

    Args:
        square_name (str): The algebraic name of the square (e.g., 'e4').
        all_piece_ids (set): A set of PyBullet object IDs representing all pieces.
        square_to_world (dict): Mapping from square names to world coordinates.
        tolerance (float): Tolerance for matching the square's world coordinates.

    Returns:
        int or None: The PyBullet object ID of the piece at the square, or None if not found.
    """
    if square_name not in square_to_world:
        logger.error(f"Square {square_name} not found in mapping.")
        return None
    target_pos = np.array(square_to_world[square_name])

    # Iterate through known piece IDs to find the one at the target square
    # Sort by distance to make it slightly more robust if pieces are slightly off-center
    pieces_with_distances = []
    for piece_id in all_piece_ids:
        try:
            piece_pos, _ = p.getBasePositionAndOrientation(piece_id)
            piece_pos_np = np.array(piece_pos)
            distance = np.linalg.norm(target_pos - piece_pos_np)
            pieces_with_distances.append((piece_id, distance))
        except p.error:
            # Object might have been removed (e.g., captured piece)
            logger.debug(f"Object ID {piece_id} not found in PyBullet, skipping (likely captured).")
            continue

    # Sort by distance and check the closest ones within tolerance
    pieces_with_distances.sort(key=lambda x: x[1])
    for piece_id, distance in pieces_with_distances:
        if distance <= tolerance:
            logger.debug(f"Found piece {piece_id} at square {square_name} (distance: {distance:.4f})")
            return piece_id

    logger.debug(f"No piece found at square {square_name} within tolerance {tolerance}. Closest candidate was {distance:.4f}m away.")
    return None
