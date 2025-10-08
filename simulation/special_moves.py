import logging
import chess
from simulation.robot_controller import RobotController
from ui.schemas import MoveData

try:
    from simulation.perception import find_piece_id_at_square
    PERCEPTION_AVAILABLE = True
except ImportError as e:
    logger = logging.getLogger(__name__)
    logger.error(f"Error importing perception module: {e}. Castling logic will fail.")
    PERCEPTION_AVAILABLE = False
    find_piece_id_at_square = None

logger = logging.getLogger(__name__)








def _get_piece_id_at_square(square_name: str, env_components: dict) -> int | None:
    """
    Helper to find the PyBullet object ID of the piece at a given square.
    Uses the perception module's `find_piece_id_at_square` function.
    """
    if not PERCEPTION_AVAILABLE or find_piece_id_at_square is None:
        logger.error("_get_piece_id_at_square: Perception module not available.")
        return None

    square_to_world = env_components.get('square_to_world_coords', {})
    all_piece_ids_set = env_components.get('piece_ids', set())

    if not square_to_world:
        logger.error("_get_piece_id_at_square: square_to_world_coords mapping not found in env_components.")
        return None
    if not all_piece_ids_set:
        logger.error("_get_piece_id_at_square: piece_ids set not found in env_components.")
        return None

    try:
        piece_id = find_piece_id_at_square(square_name, all_piece_ids_set, square_to_world, tolerance=0.015)
        if piece_id is not None:
            logger.debug(f"_get_piece_id_at_square: Found piece ID {piece_id} at square {square_name}.")
        else:
            logger.debug(f"_get_piece_id_at_square: No piece found at square {square_name}.")
        return piece_id
    except Exception as e:
        logger.error(f"_get_piece_id_at_square: Error finding piece at {square_name}: {e}")
        return None







def handle_kingside_castling(
    robot_controller: RobotController,
    board_before_move: chess.Board,
    king_move_uci: chess.Move,
    env_components: dict,
    move_log_data: MoveData
) -> bool:
    """
    Handles the physical execution of a kingside castling move.
    King moves e1->g1 (or e8->g8), Rook moves h1->f1 (or h8->f8).
    """
    if not PERCEPTION_AVAILABLE:
        logger.error(f"{robot_controller.name}: Cannot handle castling, perception module unavailable.")
        move_log_data.success = False
        move_log_data.failure_type = "Castling_Error_Perception_Unavailable"
        return False

    logger.info(f"{robot_controller.name}: Handling Kingside Castling (O-O)...")

    # 1. Determine color and squares based on the board state before the move
    turn_color = board_before_move.turn # chess.WHITE or chess.BLACK

    if turn_color == chess.WHITE:
        king_start_square = chess.E1 # e1
        king_end_square = chess.G1   # g1
        rook_start_square = chess.H1 # h1
        rook_end_square = chess.F1   # f1
        _color_prefix = "w"
    else: # Black's turn
        king_start_square = chess.E8 # e8
        king_end_square = chess.G8   # g8
        rook_start_square = chess.H8 # h8
        rook_end_square = chess.F8   # f8
        _color_prefix = "b"

    king_start_name = chess.square_name(king_start_square)
    king_end_name = chess.square_name(king_end_square)
    rook_start_name = chess.square_name(rook_start_square)
    rook_end_name = chess.square_name(rook_end_square)

    logger.info(f"{robot_controller.name}: Castling details:")
    logger.info(f"  -> King: {king_start_name} -> {king_end_name}")
    logger.info(f"  -> Rook: {rook_start_name} -> {rook_end_name}")

    # 2. Get world coordinates for the squares
    square_to_world = env_components.get('square_to_world_coords', {})
    if not square_to_world:
        logger.error(f"{robot_controller.name}: square_to_world_coords mapping not found in env_components for castling.")
        move_log_data.success = False
        move_log_data.failure_type = "Castling_Error_Mapping_Unavailable"
        return False

    try:
        king_start_pos_world = square_to_world[king_start_name]
        king_end_pos_world = square_to_world[king_end_name]
        rook_start_pos_world = square_to_world[rook_start_name]
        rook_end_pos_world = square_to_world[rook_end_name]
    except KeyError as e:
        logger.error(f"{robot_controller.name}: Square coordinate {e} not found for castling.")
        move_log_data.success = False
        move_log_data.failure_type = "Castling_Error_Coord_Not_Found"
        return False

    # 3. Find Piece IDs using perception
    king_piece_id = _get_piece_id_at_square(king_start_name, env_components)
    rook_piece_id = _get_piece_id_at_square(rook_start_name, env_components)

    if king_piece_id is None:
        logger.error(f"{robot_controller.name}: Could not find King piece at {king_start_name} for castling.")
        move_log_data.success = False
        move_log_data.failure_type = "Castling_Error_King_Not_Found"
        return False
    if rook_piece_id is None:
        logger.error(f"{robot_controller.name}: Could not find Rook piece at {rook_start_name} for castling.")
        move_log_data.success = False
        move_log_data.failure_type = "Execution"
        return False

    # 4. --- Execute Castling Moves ---
    castling_success = True
    failure_reason = None

    # --- 4a. Move the King first ---
    logger.info(f"{robot_controller.name}: Moving King ({king_piece_id}) from {king_start_name} to {king_end_name}...")
    # Use the robot controller's pick_and_place_with_retry for the King's move
    king_success = robot_controller.pick_and_place_with_retry(
        object_id=king_piece_id,
        start_pos=king_start_pos_world,
        target_pos=king_end_pos_world,
        max_retries=getattr(robot_controller, 'max_retries', 2), # Use controller's config
        move_log_data=move_log_data # You might pass a specific log if needed
    )
    if not king_success:
        logger.error(f"{robot_controller.name}: King move for castling failed.")
        castling_success = False
        failure_reason = "Castling_Error_King_Move_Failed"
        # Note: In a real scenario, you might want to attempt to move the King back here.
        # For simplicity in simulation, we'll just mark it as failed.

    # --- 4b. Move the Rook (only if King move succeeded) ---
    if castling_success:
        logger.info(f"{robot_controller.name}: Moving Rook ({rook_piece_id}) from {rook_start_name} to {rook_end_name}...")
        # Use the robot controller's pick_and_place_with_retry for the Rook's move
        rook_success = robot_controller.pick_and_place_with_retry(
            object_id=rook_piece_id,
            start_pos=rook_start_pos_world,
            target_pos=rook_end_pos_world,
            max_retries=getattr(robot_controller, 'max_retries', 2),
            move_log_data=move_log_data
        )
        if not rook_success:
            logger.error(f"{robot_controller.name}: Rook move for castling failed.")
            castling_success = False
            failure_reason = "Castling_Error_Rook_Move_Failed"
            # Note: If Rook fails, the King is already moved. You might want to move it back.
            # For simplicity, we'll just mark it as failed.

    # 5. --- Finalize and Log ---
    if castling_success:
        logger.info(f"{robot_controller.name}: Kingside Castling executed successfully.")
        move_log_data.success = True
        move_log_data.failure_type = None
    else:
        logger.error(f"{robot_controller.name}: Kingside Castling failed: {failure_reason}")
        move_log_data.success = False
        move_log_data.failure_type = failure_reason
        # Optional: Move robot to home on failure?
        robot_controller.move_to_home_position()

    # Update other relevant fields in move_log_data if needed
    # move_log_data.total_time_seconds = ... (calculate if tracked)
    # move_log_data.algorithm_used = "Castling"

    return castling_success

def handle_queenside_castling(
    robot_controller: RobotController,
    board_before_move: chess.Board,
    king_move_uci: chess.Move, # The UCI move object for the king's part of castling (e.g., e1c1)
    env_components: dict,
    move_log_data: MoveData
) -> bool:
    """
    Handles the physical execution of a queenside castling move.
    King moves e1->c1 (or e8->c8), Rook moves a1->d1 (or a8->d8).
    """
    if not PERCEPTION_AVAILABLE:
        logger.error(f"{robot_controller.name}: Cannot handle castling, perception module unavailable.")
        move_log_data.success = False
        move_log_data.failure_type = "Castling_Error_Perception_Unavailable"
        return False

    logger.info(f"{robot_controller.name}: Handling Queenside Castling (O-O-O)...")

    # 1. Determine color and squares based on the board state before the move
    turn_color = board_before_move.turn # chess.WHITE or chess.BLACK

    if turn_color == chess.WHITE:
        king_start_square = chess.E1 # e1
        king_end_square = chess.C1   # c1
        rook_start_square = chess.A1 # a1
        rook_end_square = chess.D1   # d1
        _color_prefix = "w"
    else: # Black's turn
        king_start_square = chess.E8 # e8
        king_end_square = chess.C8   # c8
        rook_start_square = chess.A8 # a8
        rook_end_square = chess.D8   # d8
        _color_prefix = "b"

    king_start_name = chess.square_name(king_start_square)
    king_end_name = chess.square_name(king_end_square)
    rook_start_name = chess.square_name(rook_start_square)
    rook_end_name = chess.square_name(rook_end_square)

    logger.info(f"{robot_controller.name}: Castling details:")
    logger.info(f"  -> King: {king_start_name} -> {king_end_name}")
    logger.info(f"  -> Rook: {rook_start_name} -> {rook_end_name}")

    # 2. Get world coordinates for the squares
    square_to_world = env_components.get('square_to_world_coords', {})
    if not square_to_world:
        logger.error(f"{robot_controller.name}: square_to_world_coords mapping not found in env_components for castling.")
        move_log_data.success = False
        move_log_data.failure_type = "Castling_Error_Mapping_Unavailable"
        return False

    try:
        king_start_pos_world = square_to_world[king_start_name]
        king_end_pos_world = square_to_world[king_end_name]
        rook_start_pos_world = square_to_world[rook_start_name]
        rook_end_pos_world = square_to_world[rook_end_name]
    except KeyError as e:
        logger.error(f"{robot_controller.name}: Square coordinate {e} not found for castling.")
        move_log_data.success = False
        move_log_data.failure_type = "Castling_Error_Coord_Not_Found"
        return False

    # 3. Find Piece IDs using perception
    king_piece_id = _get_piece_id_at_square(king_start_name, env_components)
    rook_piece_id = _get_piece_id_at_square(rook_start_name, env_components)

    if king_piece_id is None:
        logger.error(f"{robot_controller.name}: Could not find King piece at {king_start_name} for queenside castling.")
        move_log_data.success = False
        move_log_data.failure_type = "Castling_Error_Queen_King_Not_Found"
        return False
    if rook_piece_id is None:
        logger.error(f"{robot_controller.name}: Could not find Rook piece at {rook_start_name} for queenside castling.")
        move_log_data.success = False
        move_log_data.failure_type = "Castling_Error_Queen_Rook_Not_Found"
        return False

    # 4. --- Execute Queenside Castling Moves ---
    castling_success = True
    failure_reason = None

    # --- 4a. Move the King first ---
    logger.info(f"{robot_controller.name}: Moving King ({king_piece_id}) from {king_start_name} to {king_end_name}...")
    king_success = robot_controller.pick_and_place_with_retry(
        object_id=king_piece_id,
        start_pos=king_start_pos_world,
        target_pos=king_end_pos_world,
        # max_retries=getattr(robot_controller, 'max_retries', 2),
        move_log_data=move_log_data
    )
    if not king_success:
        logger.error(f"{robot_controller.name}: King move for queenside castling failed.")
        castling_success = False
        failure_reason = "Castling_Error_Queen_King_Move_Failed"

    # --- 4b. Move the Rook (only if King move succeeded) ---
    if castling_success:
        logger.info(f"{robot_controller.name}: Moving Rook ({rook_piece_id}) from {rook_start_name} to {rook_end_name}...")
        rook_success = robot_controller.pick_and_place_with_retry(
            object_id=rook_piece_id,
            start_pos=rook_start_pos_world,
            target_pos=rook_end_pos_world,
            # max_retries=getattr(robot_controller, 'max_retries', 2),
            move_log_data=move_log_data
        )
        if not rook_success:
            logger.error(f"{robot_controller.name}: Rook move for queenside castling failed.")
            castling_success = False
            failure_reason = "Castling_Error_Queen_Rook_Move_Failed"

    # 5. --- Finalize and Log ---
    if castling_success:
        logger.info(f"{robot_controller.name}: Queenside Castling executed successfully.")
        move_log_data.success = True
        move_log_data.failure_type = None
    else:
        logger.error(f"{robot_controller.name}: Queenside Castling failed: {failure_reason}")
        move_log_data.success = False
        move_log_data.failure_type = failure_reason
        # Optional: Move robot to home on failure?
        robot_controller.move_to_home_position()

    # Update other relevant fields in move_log_data if needed
    # move_log_data.total_time_seconds = ... (calculate if tracked)
    # move_log_data.algorithm_used = "Queenside_Castling"

    return castling_success

# Add other special moves (en passant, promotion) here if needed
