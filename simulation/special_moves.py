import chess
import random
import logging
import numpy as np
import pybullet as p

from simulation.robot_controller import RobotController
from ui.schemas import MoveData
from configs.config import config, DEAD_PIECES_AREA_CENTER, DEAD_PIECES_AREA_SIZE

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





def handle_captured_piece(target_square_name:str, attacker_piece_id:int, env_components:dict, robot_controller:RobotController):
    """
    Checks if a piece was captured on the target square and moves it aside.

    Args:
        target_square_name (str): The name of the square where the attacker was placed (e.g., 'e4').
        attacker_piece_id (int): The PyBullet ID of the piece that just moved (the attacker).
        env_components (dict): Environment components including square_to_world_coords and piece_ids.
        robot_controller (RobotController): The controller to use for moving the captured piece.

    Returns:
        bool: True if handling was successful (no capture or capture moved), False otherwise.
    """
    logger.info(f"Checking for captured piece on square {target_square_name} after placing attacker ID {attacker_piece_id}...")
    square_to_world = env_components.get('square_to_world_coords', {})
    all_piece_ids = env_components.get('piece_ids', {}).keys()
    target_pos = square_to_world.get(target_square_name)

    if target_pos is None:
        logger.error(f"Target square {target_square_name} not found in mapping.")
        return False

    # --- 1. Find all piece IDs currently near the target square ---
    # Use a small tolerance to find pieces considered "on" the square
    TOLERANCE = 0.02 # 2cm tolerance, adjust based on piece/bucket size
    pieces_on_square = []
    for piece_id in all_piece_ids:
        try:
            piece_pos, _ = p.getBasePositionAndOrientation(piece_id)
            distance = np.linalg.norm(np.array(piece_pos[:2]) - np.array(target_pos[:2])) # Compare X,Y only
            if distance <= TOLERANCE:
                pieces_on_square.append(piece_id)
        except p.error:
            # Piece might have been removed already
            continue

    logger.debug(f"Pieces found near {target_square_name}: {pieces_on_square}")

    # --- 2. Identify the captured piece ---
    if len(pieces_on_square) < 1:
        logger.warning(f"No pieces found near {target_square_name} after placing attacker. This is unexpected.")
        return True # Technically no capture to handle
    elif len(pieces_on_square) == 1:
        piece_on_square_id = pieces_on_square[0]
        if piece_on_square_id == attacker_piece_id:
            logger.info(f"Only the attacker piece {attacker_piece_id} found on {target_square_name}. No capture occurred.")
            return True # No capture
        else:
            # This shouldn't happen if the attacker was just placed correctly
            logger.warning(f"Only piece {piece_on_square_id} found on {target_square_name}, but it's not the attacker {attacker_piece_id}. Possible placement error?")
            # Assume the one present is the captured one? Or treat as error?
            captured_piece_id = piece_on_square_id
    else: # len(pieces_on_square) > 1
        logger.info(f"Multiple pieces found on {target_square_name}: {pieces_on_square}. Capture detected.")
        # Assume the one that is NOT the attacker is the captured piece
        # (This handles the case where pieces might overlap slightly in simulation)
        captured_candidates = [pid for pid in pieces_on_square if pid != attacker_piece_id]
        if len(captured_candidates) == 1:
            captured_piece_id = captured_candidates[0]
            logger.info(f"Identified captured piece ID: {captured_piece_id}")
        elif len(captured_candidates) > 1:
            logger.warning(f"Multiple captured candidates found: {captured_candidates}. Arbitrarily choosing the first one.")
            captured_piece_id = captured_candidates[0] # Arbitrary choice, might need better logic
        else:
            # This means the attacker isn't in the list, which is strange
            logger.error(f"Attacker {attacker_piece_id} not found in pieces_on_square {pieces_on_square} but multiple pieces are present. Logic error?")
            return False

    # --- 3. Move the captured piece outside the board ---
    if captured_piece_id == attacker_piece_id:
        logger.info("Piece on square is the attacker itself. No capture to handle.")
        return True

    logger.info(f"Moving captured piece {captured_piece_id} out of play...")

    # --- 4. Calculate a random position within the dead pieces area ---
    # Define the bounds of the area
    center = np.array(DEAD_PIECES_AREA_CENTER)
    size = np.array(DEAD_PIECES_AREA_SIZE)
    half_size = size / 2.0

    # Generate a random position within the area (X, Y) and keep Z constant or slightly random
    random_x = random.uniform(center[0] - half_size[0], center[0] + half_size[0])
    random_y = random.uniform(center[1] - half_size[1], center[1] + half_size[1])
    # Keep Z constant or add a small random height variation
    random_z = center[2] # + random.uniform(-0.01, 0.01) # Optional slight Z variation

    target_drop_pos = [random_x, random_y, random_z]
    _target_drop_orient = p.getQuaternionFromEuler([0, 0, 0]) # Upright orientation, or match board orientation

    logger.info(f"Dropping captured piece {captured_piece_id} at random position {target_drop_pos}...")


    try:
        current_piece_pos, _ = p.getBasePositionAndOrientation(captured_piece_id)
    except p.error:
        logger.warning(f"Captured piece {captured_piece_id} no longer exists. Assuming it was already handled or removed.")
        return True # Piece is gone, nothing to do
    success = robot_controller.pick_and_place_with_retry(
        object_id=captured_piece_id,
        start_pos=current_piece_pos, # Pick from its current location on the board
        target_pos=target_drop_pos,   # Place in the dead pieces area
        max_retries=config.task.max_retries, # Use config value
        move_log_data=MoveData()
    )

    if success:
        logger.info(f"Successfully moved captured piece {captured_piece_id} to dead pieces area at {target_drop_pos}.")
        # Optional: Update internal game state tracking if you maintain a list of active/inactive pieces
        # env_components['inactive_piece_ids'].add(captured_piece_id) # If you have such a set
        # env_components['piece_ids'].pop(captured_piece_id, None) # Remove from active list if needed
        return True
    else:
        logger.error(f"Failed to move captured piece {captured_piece_id} to dead pieces area.")
        return False # Indicate failure in handling the capture



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
        move_log_data.failure_type = "Execution"
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

def handle_checkmate(robot_controller, move_log_data):
    """
    Handles the physical execution of a checkmate.
    """
    logger.info(f"{robot_controller.name}: Handling Checkmate...")

#TODO: Implement checkmate handling if any physical action is needed
def handle_promotion(robot_controller, board_before_move, promotion_move_uci, env_components, move_log_data):
    """
    Handles the physical execution of a pawn promotion move.
    """
    pass


#TODO: Implement en passant handling if any physical action is needed   
def handle_en_passant(robot_controller, board_before_move, en_passant_move_uci, env_components, move_log_data):
    logger.info(f"{robot_controller.name}: Handling En Passant...")
    pass



def move_captured_pieces_to_bins(captured_piece_ids, env_components, robot_controller):
    """
    Move captured pieces to their respective color bins.
    
    Args:
        captured_piece_ids (set): Set of IDs of pieces that were captured.
        env_components (dict): Environment components including piece_id_to_piece_type and bin_ids.
        robot_controller (RobotController): The robot controller to use for moving pieces.
    """
    if not captured_piece_ids:
        logger.info("No pieces captured, no need to move to bins.")
        return True

    piece_id_to_type = env_components.get('piece_id_to_piece_type', {})
    bin_ids = env_components.get('bin_ids', {})
    
    white_bin_id = bin_ids.get("captured_white_bin")
    black_bin_id = bin_ids.get("captured_black_bin")

    if not white_bin_id or not black_bin_id:
        logger.error("Captured piece bins not found in environment components.")
        return False

    # Get bin positions (center top)
    try:
        white_bin_pos, white_bin_orn = p.getBasePositionAndOrientation(white_bin_id)
        black_bin_pos, black_bin_orn = p.getBasePositionAndOrientation(black_bin_id)
        # Calculate a position slightly above the center of the bin for dropping
        white_bin_drop_pos = [white_bin_pos[0], white_bin_pos[1], white_bin_pos[2] + config.environment.bin_size[2] + 0.05] # Z + height + clearance
        black_bin_drop_pos = [black_bin_pos[0], black_bin_pos[1], black_bin_pos[2] + config.environment.bin_size[2] + 0.05]
    except Exception as e:
        logger.error(f"Error getting bin positions: {e}")
        return False

    success = True
    for piece_id in captured_piece_ids:
        piece_type = piece_id_to_type.get(piece_id, "unknown")
        logger.info(f"Handling captured piece: ID {piece_id}, Type: {piece_type}")
        
        # Determine target bin based on piece color (simplified logic)
        # You might need a more robust way to determine piece color from type string
        if "_w" in piece_type.lower(): # Assumes piece types end with _w or _b
            target_bin_pos = white_bin_drop_pos
            target_bin_name = "White Bin"
        elif "_b" in piece_type.lower():
            target_bin_pos = black_bin_drop_pos
            target_bin_name = "Black Bin"
        else:
            logger.warning(f"Could not determine color for captured piece {piece_id} ({piece_type}). Placing in White Bin as default.")
            target_bin_pos = white_bin_drop_pos
            target_bin_name = "White Bin (Default)"

        # Get current position of the captured piece
        try:
            piece_pos, _ = p.getBasePositionAndOrientation(piece_id)
        except p.error:
            logger.warning(f"Captured piece ID {piece_id} no longer exists in simulation, skipping.")
            continue # Piece might have been removed by PyBullet already

        logger.info(f"Moving captured piece {piece_id} ({piece_type}) to {target_bin_name} at {target_bin_pos}")

        # --- CRITICAL: Use the robot's pick_and_place function ---
        # Move the piece from its current location to the bin drop position
        # You need to determine the appropriate robot controller.
        # For now, assume the provided robot_controller is used.
        # The pick_and_place function needs the piece ID and target position.
        # It will handle approach, grasp, lift, move, place, open gripper.
        move_success = robot_controller.pick_and_place_with_retry(
            object_id=piece_id,
            start_pos=piece_pos, # Current position of the captured piece
            target_pos=target_bin_pos, # Drop position above the target bin
            max_retries=config.task.max_retries # Use config value
            # You might need to pass additional parameters like move_log_data if required
        )
        
        if not move_success:
            logger.error(f"Failed to move captured piece {piece_id} to {target_bin_name}.")
            success = False # Mark overall success as False, but continue with other captured pieces
        else:
            logger.info(f"Successfully moved captured piece {piece_id} to {target_bin_name}.")

    return success

