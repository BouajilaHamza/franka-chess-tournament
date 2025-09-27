import logging
import numpy as np
import chess
import pybullet as p
import time
from typing import Dict, Any

from simulation.robot_controller import RobotController
from simulation.chess_engine import ChessEngine

logger = logging.getLogger(__name__)

def run_game_loop(env_components: Dict[str, Any], robot_controllers: Dict[str, RobotController], chess_engine: ChessEngine):
    """
    Main game loop integrating perception, decision (engine), and control (robot).
    Assumes RobotController has methods like pick_and_place_with_retry and move_to_home_position.
    """
    square_to_world = env_components['square_to_world_coords']
    # piece_ids is assumed to be a dict mapping PyBullet object IDs to their square names or just a list of IDs
    # We primarily need the list of IDs for obstacle checking and finding pieces
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

    while not game_over and move_count < max_moves:
        move_count += 1
        logger.info(f"--- Game Turn {move_count} ---")

        # 1. Check whose turn it is based on the chess engine's internal state
        current_turn_color = chess_engine.get_internal_board().turn # chess.WHITE or chess.BLACK
        logger.info(f"Current turn according to engine: {'White' if current_turn_color == chess.WHITE else 'Black'}")

        # 2. Determine which controller to use
        if current_turn_color == chess.WHITE:
            controller_to_move: RobotController = robot_controllers['white']
            robot_name = "Robot1 (White)"
        else: # current_turn_color == chess.BLACK
            controller_to_move: RobotController = robot_controllers['black']
            robot_name = "Robot2 (Black)"

        logger.info(f"It's {robot_name}'s turn.")

        # 3. Get the move from the engine (this works for the player whose turn it is)
        robot_move_uci = chess_engine.get_robot_move()

        if robot_move_uci is None:
            logger.info("Engine returned no move. Game might be over.")
            # Check game over conditions using python-chess
            board_state = chess_engine.get_internal_board()
            if board_state.is_checkmate():
                 winning_color = "White" if board_state.result() == "1-0" else "Black"
                 logger.info(f"Checkmate! {winning_color} wins.")
            elif board_state.is_stalemate():
                 logger.info("Stalemate! Game is a draw.")
            elif board_state.is_insufficient_material():
                 logger.info("Draw due to insufficient material.")
            elif board_state.is_seventyfive_moves():
                 logger.info("Draw due to 75-move rule.")
            elif board_state.is_fivefold_repetition():
                 logger.info("Draw due to fivefold repetition.")
            else:
                 logger.warning("Engine returned None, but game state unclear.")
            game_over = True
            break # Exit the game loop

        # 4. Convert UCI move to physical world coordinates
        start_square_uci = robot_move_uci.from_square # e.g., chess.E2 (integer)
        target_square_uci = robot_move_uci.to_square # e.g., chess.E4 (integer)
        start_square_name = chess.square_name(start_square_uci) # e.g., 'e2'
        target_square_name = chess.square_name(target_square_uci) # e.g., 'e4'

        logger.info(f"{robot_name} is commanded to move: {start_square_name} -> {target_square_name} (SAN: {chess_engine.get_internal_board().san(robot_move_uci)})")

        try:
            start_pos_world = square_to_world[start_square_name]
            target_pos_world = square_to_world[target_square_name]
        except KeyError as e:
            logger.error(f"Square {e} not found in mapping. Cannot execute move.")
            game_over = True
            break

        # 5. Find the piece ID to move (using PyBullet state diff or a maintained mapping)
        # Pass the set of all piece IDs for accurate checking
        piece_id_to_move = find_piece_id_at_square(start_square_name, all_piece_ids_set, square_to_world)
        if piece_id_to_move is None:
            logger.error(f"Could not find a piece at square {start_square_name} to move.")
            game_over = True
            break

        # 6. --- CRITICAL UPDATE: Execute the move using the appropriate robot controller ---
        # The controller now handles IK/OMPL selection internally via its methods.
        logger.info(f"{robot_name} executing move: Pick {piece_id_to_move} from {start_square_name}, place at {target_square_name}")
        # --- Use the high-level pick-and-place method ---
        # This method internally calls _pick_place_single_attempt which should use move_smartly_to_position
        success = controller_to_move.pick_and_place_with_retry(
            object_id=piece_id_to_move,
            start_pos=start_pos_world,
            target_pos=target_pos_world,
            max_retries=getattr(controller_to_move, 'max_retries', 2) # Use controller's default or config
        )

        if not success:
            logger.error(f"{robot_name} failed to execute move {start_square_name} -> {target_square_name}. Stopping simulation.")
            # Optional: Add specific error handling/recovery attempts here
            game_over = True
            break
        else:
            controller_to_move.move_to_home_position() # Controlled by pick_and_place_with_retry or separate logic
            logger.info(f"{robot_name} successfully executed move {start_square_name} -> {target_square_name}")

        # 7. Update the chess engine's internal state after the physical move
        chess_engine.update_internal_board(robot_move_uci)
        logger.info(f"Engine state updated after move: {chess_engine.get_internal_board().fen()}")

        # 8. Wait or handle human move if applicable (simplified here)
        # If robots are playing each other, loop continues.
        # If human vs robot, implement logic to wait for human move detection.
        time.sleep(0.5) # Optional: delay between moves

    logger.info("Game loop finished.")

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
