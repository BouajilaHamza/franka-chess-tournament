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

# --- Helper Function You Need To Implement ---
def get_current_piece_ids_in_simulation():
    """
    Gets the current set of piece IDs present in the PyBullet simulation.
    This needs to iterate through bodies and identify which ones are chess pieces.
    You might use p.getBodyInfo, p.getBasePositionAndOrientation, or check against
    a known list of initial piece IDs and see which still exist.
    """
    current_ids = set()
    try:
        num_bodies = p.getNumBodies()
        for body_id in range(num_bodies):
            # Check if the body ID corresponds to a known piece
            # This assumes you have a way to identify piece bodies
            # Option 1: Check against initial list and see if it still exists/is valid
            # if p.getBodyInfo(body_id): # This checks if body exists
            #     # Further checks might be needed to confirm it's a piece
            #     current_ids.add(body_id)
            
            # Option 2: More robust check (if you have a global list or can identify by type/name)
            try:
                # Attempt to get info, if it fails, body likely doesn't exist
                p.getBodyInfo(body_id) 
                # Add logic here to distinguish pieces from robots, board, bins, ground
                # This is tricky. A simple way is to check if the body_id was in the initial list
                # and still has valid dynamics/getBasePositionAndOrientation works.
                # Or, check the body name if you set it uniquely for pieces.
                body_info = p.getBodyInfo(body_id)
                body_name = body_info[1].decode('utf-8') # Name is bytes, decode to string
                # print(f"Body ID: {body_id}, Name: {body_name}") # Debug print
                # If you named your pieces or have a way to identify them:
                # if "pawn" in body_name or "rook" in body_name or ... : 
                #    current_ids.add(body_id)
                
                # Simpler approach: Assume any body that existed initially and still exists is a piece
                # You need a reference to the initial set of piece IDs
                # global INITIAL_PIECE_IDS_SET # If you stored it globally
                # if body_id in INITIAL_PIECE_IDS_SET:
                #     current_ids.add(body_id)
                    
                # Placeholder: Just add all valid body IDs (WRONG, includes robots, board, etc.)
                # current_ids.add(body_id) 
                
                # --- CORRECT APPROACH (Assuming you pass the initial set or have access to it) ---
                # You need to compare against the known piece IDs.
                # Let's assume this function is called from within a context where
                # env_components['piece_ids'].keys() is available, or you pass it.
                # For this example, we'll assume access to the initial set somehow.
                # A better way is to pass the initial set or use a global tracker.
                # For now, let's assume a global or passed initial set exists.
                # THIS IS A PLACEHOLDER - YOU NEED TO IMPLEMENT THE CORRECT LOGIC HERE
                # based on how you track your initial pieces.
                # Example (incorrect but illustrative):
                # if body_id in some_known_initial_piece_id_set:
                #     current_ids.add(body_id)
                     
            except p.error:
                # Body ID doesn't exist or error occurred, skip
                continue
    except Exception as e:
        logger.error(f"Error getting current piece IDs: {e}")
    return current_ids
