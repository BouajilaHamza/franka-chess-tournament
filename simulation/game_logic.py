import logging
import numpy as np
import chess
import pybullet as p
import time
from typing import Dict
from simulation.robot_controller import RobotController
from simulation.chess_engine import ChessEngine



logger = logging.getLogger(__name__)

def run_game_loop(env_components, robot_controllers:Dict[str, RobotController], chess_engine: ChessEngine):
    """
    Main game loop integrating perception, decision (engine), and control (robot).
    """
    square_to_world = env_components['square_to_world_coords']
    piece_ids = env_components['piece_ids'] # Assuming this maps initial piece positions or IDs

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
            controller_to_move = robot_controllers['white']
            robot_name = "Robot1 (White)"
        else: # current_turn_color == chess.BLACK
            controller_to_move = robot_controllers['black']
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

        # 4. Convert UCI move to physical world coordinates and find the piece ID
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

        # Find the piece ID to move (using PyBullet state diff or a maintained mapping)
        piece_id_to_move = find_piece_id_at_square(start_square_name, piece_ids, square_to_world)
        if piece_id_to_move is None:
            logger.error(f"Could not find a piece at square {start_square_name} to move.")
            game_over = True
            break

        # 5. Execute the move using the appropriate robot controller
        logger.info(f"{robot_name} executing move: Pick {piece_id_to_move} from {start_square_name}, place at {target_square_name}")
        success = controller_to_move.pick_and_place_with_retry(
            piece_id_to_move, start_pos_world, target_pos_world
        )

        if not success:
            logger.error(f"{robot_name} failed to execute move {start_square_name} -> {target_square_name}. Stopping simulation.")
            game_over = True
            break
        else:
            controller_to_move.move_to_home_position()
            logger.info(f"{robot_name} successfully executed move {start_square_name} -> {target_square_name}")

        # 6. Update the chess engine's internal state after the physical move
        chess_engine.update_internal_board(robot_move_uci)
        logger.info(f"Engine state updated after move: {chess_engine.get_internal_board().fen()}")

        # 7. Wait or handle human move if applicable (simplified here)
        # If robots are playing each other, loop continues.
        # If human vs robot, implement logic to wait for human move detection.
        time.sleep(0.5) # Optional: delay between moves

    logger.info("Game loop finished.")

def find_piece_id_at_square(square_name, piece_ids, square_to_world, tolerance=0.01):
    """
    Find the PyBullet object ID of the piece located at a given square name.
    Uses PyBullet's getBasePositionAndOrientation to check piece locations.

    Args:
        square_name (str): The algebraic name of the square (e.g., 'e4').
        piece_ids (dict): A dictionary mapping PyBullet object IDs to their initial square names
                          or just a list/dict of IDs to check.
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
    for piece_id in piece_ids.keys(): # Iterate over the keys (object IDs) of the loaded pieces
        try:
            piece_pos, _ = p.getBasePositionAndOrientation(piece_id)
            piece_pos_np = np.array(piece_pos)
            distance = np.linalg.norm(target_pos - piece_pos_np)
            if distance <= tolerance:
                logger.debug(f"Found piece {piece_id} at square {square_name} (distance: {distance:.4f})")
                return piece_id
        except p.error:
            # Object might have been removed (e.g., captured piece)
            logger.warning(f"Object ID {piece_id} not found in PyBullet, skipping.")
            continue

    logger.debug(f"No piece found at square {square_name} within tolerance {tolerance}")
    return None

# Add other game-specific functions here if needed, like handling human moves,
# checking game status, managing captured pieces, etc.