import logging
import numpy as np
import pybullet as p

logger = logging.getLogger(__name__)

def find_piece_id_at_square(square_name: str, all_piece_ids: set, square_to_world: dict, tolerance: float = 0.015) -> int | None:
    """
    Find the PyBullet object ID of the piece located at a given square name.
    Uses PyBullet's getBasePositionAndOrientation to check piece locations.
    Delegates to a simpler state-diff function.
    """
    logger.debug(f"Finding piece ID at square '{square_name}'...")
    if square_name not in square_to_world:
        logger.error(f"Square {square_name} not found in mapping.")
        return None
    target_pos = np.array(square_to_world[square_name])

    # --- DELEGATE TO CORE STATE DIFF LOGIC ---
    piece_id = find_piece_id_at_position(target_pos, all_piece_ids, tolerance)
    if piece_id is not None:
        logger.debug(f"Found piece {piece_id} at square '{square_name}'.")
    else:
        logger.debug(f"No piece found at square '{square_name}' within tolerance {tolerance}.")
    return piece_id

def find_piece_id_at_position(target_pos: np.ndarray, all_piece_ids: set, tolerance: float = 0.015) -> int | None:
    """
    Core logic to find a piece ID near a given 3D world position.
    """
    pieces_with_distances = []
    for piece_id in all_piece_ids:
        try:
            piece_pos, _ = p.getBasePositionAndOrientation(piece_id)
            piece_pos_np = np.array(piece_pos)
            distance = np.linalg.norm(target_pos - piece_pos_np)
            pieces_with_distances.append((piece_id, distance))
        except p.error:
            logger.debug(f"Object ID {piece_id} not found, skipping.")
            continue

    pieces_with_distances.sort(key=lambda x: x[1])
    for piece_id, distance in pieces_with_distances:
        if distance <= tolerance:
            return piece_id
    return None

# Add other perception functions here (e.g., detect_board_state_from_pybullet,
# get_ee_position, verify_placement, etc.)
