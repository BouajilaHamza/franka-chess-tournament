# PyBullet Chess Robot Simulation

## Overview

This project simulates a game of chess played between two Franka Panda robot arms in PyBullet. It integrates perception (state-diff), decision-making (Stockfish engine via `python-chess`), and control simulation (PyBullet IK/physics) within an interactive Streamlit interface that provides commentary and game state updates.

The goal is to create a professional, complex, and impressive demonstration of Embodied AI principles, showcasing integration across multiple domains (Perception, Planning, Control, AI, UI) in a real-world application context (Chess).

---

## Project Structure (Suggested)

```
your_project_directory/
├── README.md                 # This file
├── requirements.txt          # Python dependencies
├── config.py                 # Pydantic configuration models
├── simulation/
│   ├── __init__.py
│   ├── builder.py            # PyBullet environment builder (optional)
│   ├── robot_control.py      # IK, motion execution, gripper control
│   ├── perception.py         # State-diff board state detection
│   └── game_logic.py         # Stockfish integration, move validation
├── ui/
│   ├── __init__.py
│   └── streamlit_app.py      # Streamlit interface
├── main.py                   # Main execution script
└── assets/                   # (Optional) Images, URDFs if not in PyBullet paths
    ├── board.urdf
    ├── pawn.urdf
    └── ...
```

---

## Dependencies

*   `pybullet`
*   `numpy`
*   `streamlit`
*   `python-chess`
*   `stockfish` (or `python-chess` with a Stockfish binary)
*   `pydantic` (for configuration management)
*   `logging` (standard library)
*   `time` (standard library)

Add these to a `requirements.txt` file.

---

## Roadmap

### Phase 1: Core Simulation & Decision Engine

*   **Status:** Planned / In Progress
*   **Goal:** Establish the basic PyBullet simulation environment and Stockfish integration.
*   **Tasks:**
    *   [ ] **Setup PyBullet Environment:**
        *   [*] Load chess board URDF.
        *   [*] Load two Franka Panda arms URDFs, positioned appropriately (e.g., opposing sides).
        *   [*] Load 32 individual piece URDFs (or separate URDFs per type) and place them in the initial chess position.
        *   [*] Define 3D coordinates for each square (`square_to_world_coords`).
        *   [ ] Implement basic IK (`p.calculateInverseKinematics`) for both arms to move to specific square coordinates (approach, grasp, place).
        *   [ ] Implement basic `move_piece_simulated(start_square, end_square, robot_id)` function for *one* arm, including approach, grasp, lift, move, place, release, retreat.
    *   [ ] **Integrate Stockfish:**
        *   [ ] Use `python-chess` to manage the game state (`chess.Board`).
        *   [ ] Implement a function `get_robot_move(game_board, engine_color)` that calls Stockfish to get the next move based on the current `game_board` state for the specified color.

### Phase 2: Perception (State-Diff)

*   **Status:** Planned
*   **Goal:** Enable the system to perceive the current board state and detect human moves (if human interaction is added later) or verify robot moves.
*   **Tasks:**
    *   [ ] **Implement State-Diff Perception:**
        *   [ ] Create a function `detect_board_state_from_pybullet()` that iterates through the loaded piece objects, reads their positions using `p.getBasePositionAndOrientation`, and maps them to the corresponding square on the `python-chess.Board` object.
        *   [ ] Create a function `detect_move_execution(previous_state, current_state)` to identify the move made by a robot based on state changes (optional, verification).

### Phase 3: Streamlit Interface (Communication Hub)

*   **Status:** Planned
*   **Goal:** Create the Streamlit interface for commentary, game state display, and potentially chat.
*   **Tasks:**
    *   [ ] **Basic UI:**
        *   [ ] Create Streamlit app (`streamlit_app.py`).
        *   [ ] Add a display area for the current board state (text representation or simple diagram).
        *   [ ] Add area for game logs/commentary (e.g., `st.text_area`).
        *   [ ] Add control buttons (Start Game, Reset, Pause/Resume).
    *   [ ] **Integrate Simulation:**
        *   [ ] Ensure the Streamlit app can trigger the main game loop (perception -> decision -> control).
        *   [ ] Send commentary/logs from the simulation loop to the Streamlit interface (likely via session state or file writing/reading if complex).

### Phase 4: Full Game Loop (Robot vs Robot)

*   **Status:** Planned
*   **Goal:** Integrate all components into a complete game loop where two robots play against each other.
*   **Tasks:**
    *   [ ] **Implement Main Game Loop:**
        *   [ ] Initialize the board state.
        *   [ ] Implement a loop alternating turns between 'White' (Robot 1) and 'Black' (Robot 2).
        *   [ ] On each turn: Call `detect_board_state_from_pybullet()` to get the current state -> Call `get_robot_move()` for the current player's color -> Execute the move using the corresponding robot's `move_piece_simulated` function.
        *   [ ] Update the `python-chess` board state after each move execution.
        *   [ ] Log significant events (moves, checks, checkmate, stalemate) to the Streamlit commentary area.
    *   [ ] **Refine Control:**
        *   [ ] Ensure IK parameters (`residualThreshold`, `maxNumIterations`) are tuned for high precision on the chessboard (e.g., `1e-6`, 500).
        *   [ ] Implement error handling and recovery for IK failures or failed grasp/placement attempts (e.g., return to home position).
        *   [ ] Add verification steps (e.g., check piece position after placement using `detect_board_state_from_pybullet`).

### Phase 5: Polish & Advanced Features (Optional)

*   **Status:** Planned
*   **Goal:** Enhance the simulation's robustness, user interaction, and impressiveness.
*   **Tasks:**
    *   [ ] **Human Interaction (Optional):** Allow a human to play against one of the robots via PyBullet interaction or Streamlit input.
    *   [ ] **Chat/Commentary Enhancement:** Implement a simple, rule-based system for the robots to "comment" on the game state (e.g., "Interesting strategy!", "Watch out for that knight!").
    *   [ ] **KPIs & Logging:** Add basic metrics tracking (e.g., game duration, number of moves, move success rate for the robots).
    *   [ ] **Documentation:** Write detailed docstrings and comments explaining the logic of each function and module.

---

## Key Learning Points & Concepts

*   **Embodied AI:** Understanding the integration of perception, planning, and control in a physical (simulated) environment.
*   **Simulation (PyBullet):** Loading URDFs, managing physics, using IK, understanding simulation parameters (timestep, gravity, damping).
*   **AI Integration:** Using `python-chess` and Stockfish for game logic and decision-making.
*   **Perception (State-Diff):** Mapping simulated object positions to a game state representation.
*   **Control Simulation:** Using IK to plan and execute robot movements in simulation.
*   **Software Architecture:** Structuring a multi-component application, using configuration management (Pydantic), integrating different libraries.
*   **User Interface:** Creating an interactive web interface with Streamlit.
*   **Debugging & Integration:** Troubleshooting issues when connecting multiple complex systems.

---

## Notes

*   This project focuses on simulating the Embodied AI aspects *before* considering real-world deployment.
*   The precision requirements for chess demand careful tuning of IK parameters.
*   The Streamlit interface serves as the "communication hub," providing context and commentary for the action happening in the PyBullet window.
*   State-diff perception is used for simplicity and speed in simulation, acknowledging the difference from real-world vision systems.
*   Focus on building each component incrementally and testing integration points thoroughly.
