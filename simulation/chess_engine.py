# chess_engine.py
import chess
import chess.engine
import logging

logger = logging.getLogger(__name__)

class ChessEngine:
    """
    Manages the game state using python-chess and interacts with Stockfish for move generation.
    """
    def __init__(self, stockfish_path="stockfish", depth=10):
        """
        Initializes the chess engine and loads Stockfish.

        Args:
            stockfish_path (str): Path to the Stockfish executable. Defaults to "stockfish".
            depth (int): Search depth for Stockfish. Higher values mean stronger play but slower time.
        """
        self.board = chess.Board() # Initialize the game board
        self.engine_path = stockfish_path
        self.engine_depth = depth
        self.engine_process = None
        self.is_initialized = False

    def initialize_engine(self):
        """Starts the Stockfish engine process."""
        if self.is_initialized:
            logger.warning("Engine already initialized.")
            return

        try:
            self.engine_process = chess.engine.SimpleEngine.popen_uci(self.engine_path)
            logger.info(f"Stockfish engine initialized from: {self.engine_path}")
            self.is_initialized = True
        except FileNotFoundError:
            logger.error(f"Stockfish executable not found at path: {self.engine_path}")
            logger.error("Please ensure Stockfish is installed and the path is correct.")
            raise FileNotFoundError(f"Stockfish executable not found at path: {self.engine_path}")
        except Exception as e:
            logger.error(f"Error initializing Stockfish engine: {e}")
            raise e

    def get_robot_move(self, current_board_state=None):
        """
        Generates the next move for the player whose turn it currently is,
        using Stockfish. Works for both White and Black turns.

        Args:
            current_board_state (chess.Board, optional): The current state of the game board.
                If not provided, uses the internal self.board state.

        Returns:
            chess.Move: The move suggested by Stockfish for the current player,
                        or None if no legal moves exist or if the game is over.
        """
        if not self.is_initialized:
            logger.error("Engine not initialized. Call initialize_engine() first.")
            raise RuntimeError("Engine not initialized. Call initialize_engine() first.")

        # Use the provided board state or the internal one
        board_to_analyze = current_board_state if current_board_state is not None else self.board

        # Check if the game is already over
        if board_to_analyze.is_game_over():
            logger.info("Game is over (checkmate, stalemate, etc.). No move to generate.")
            return None

        # Check if it's *anyone's* turn (i.e., the game isn't over and there are legal moves)
        # The engine will generate a move for the *current* player on move.
        # No need to check board_to_analyze.turn against a specific color like chess.WHITE anymore.
        # Stockfish will analyze the position and generate a move for the side to move.

        try:
            # Use the engine to find the best move for the side to move
            result = self.engine_process.play(
                board_to_analyze,
                chess.engine.Limit(depth=self.engine_depth),
            )
            move = result.move
            # Log the move using SAN based on the board state *before* the move is applied
            # This SAN reflects the move for the current player (whose turn it is)
            logger.info(f"Stockfish generated move for {'White' if board_to_analyze.turn == chess.WHITE else 'Black'}: {board_to_analyze.san(move)}")
            return move
        except Exception as e:
            logger.error(f"Error getting move from Stockfish: {e}")
            return None


    def update_internal_board(self, move):
        """
        Updates the internal board state after a move is made (by the robot or human).
        This is crucial for maintaining the correct game state for future Stockfish calls.

        Args:
            move (chess.Move): The move to apply to the internal board.
        """
        if self.board.is_legal(move):
            san_move = self.board.san(move) # <--- Get SAN here, when 'move' is still legal in 'self.board'
            self.board.push(move) # <--- Update the board state (including turn)
            logger.debug(f"Applied move {san_move} (UCI: {move}) to internal board. Turn is now: {'White' if self.board.turn == chess.WHITE else 'Black'}")
        else:
            logger.warning(f"Attempted to apply illegal move {move} to internal board: {self.board.fen()}")

    def reset_game(self):
        """Resets the internal board to the starting position."""
        self.board.reset()
        logger.info("Game board reset to starting position.")

    def get_internal_board(self):
        """Returns the current internal board state."""
        return self.board.copy() # Return a copy to prevent external modification

    def close_engine(self):
        """Closes the Stockfish engine process."""
        if self.engine_process:
            self.engine_process.quit()
            logger.info("Stockfish engine process closed.")
        self.is_initialized = False

