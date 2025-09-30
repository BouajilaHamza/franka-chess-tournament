from sqlmodel import Session, select
from ui.models import Experiment, Move, FailureDetail
from ui.database_setup import engine
from ui.schemas import MoveData
import logging
from datetime import datetime




logger = logging.getLogger(__name__)
DATABASE_URL = "sqlite:///./robot_performance.db"

class MetricsLoggerSQLModel:
    def __init__(self, db_url=DATABASE_URL):
        self.db_url = db_url # Not directly used here as engine is global, but kept for consistency
        self.current_experiment_id = None
        self.move_counter = 0

    def start_experiment(self, name, notes=""):
        """Starts a new experiment run and gets an experiment ID.
        Args:
            name (str): The name of the experiment.
            notes (str, optional): Additional notes about the experiment.
        Returns:
            int: The ID of the started experiment.
        """
        try:
            with Session(engine) as session:
                experiment = Experiment(name=name, notes=notes)
                session.add(experiment)
                session.commit()
                session.refresh(experiment) # Get the ID after commit
                self.current_experiment_id = experiment.id
                self.move_counter = 0
                logger.info(f"Started SQLModel experiment '{name}' with ID {self.current_experiment_id}")
                return self.current_experiment_id
        except Exception as e:
            logger.error(f"Error starting SQLModel experiment: {e}")
            self.current_experiment_id = None
            return None

    def log_move(self, move_log_data:MoveData):
        """Logs data for a single move/pick-and-place attempt using SQLModel.
        Args:
            move_data_dict (dict): A dictionary containing move data.
        Returns:
            int: The ID of the logged move (if successful).
        """
        if self.current_experiment_id is None:
            logger.warning("Warning: No active SQLModel experiment. Move not logged.")
            return

        self.move_counter += 1

        # --- Map dictionary data to SQLModel instance ---
        try:
            move_record = Move(**move_log_data)

            with Session(engine) as session:
                session.add(move_record)
                session.commit()
                session.refresh(move_record) # Get the ID
                logger.info(f"Logged SQLModel move {self.move_counter} (DB ID: {move_record.id}) for experiment {self.current_experiment_id}")
                return move_record.id # Return DB ID in case failure details need it

        except ValueError as ve: # Catch enum conversion errors
            logger.error(f"Value error logging SQLModel move (likely enum mismatch): {ve}")
        except Exception as e:
            logger.error(f"Error logging SQLModel move: {e}")
        return None

    def log_failure_detail(self, move_db_id, error_message="", snapshot_data=None):
        """Logs detailed failure information for a specific move.
        Args:
            move_db_id (int): The ID of the move that failed.
            error_message (str, optional): The error message.
            snapshot_data (dict, optional): A dictionary containing snapshot data.
        Returns:
            None
        """
        if move_db_id is None:
            logger.warning("Cannot log failure detail: move_db_id is None.")
            return
        try:
            # Convert snapshot_data dict to string if needed, or store path
            snapshot_str = str(snapshot_data) if snapshot_data else None

            failure_detail_record = FailureDetail(
                move_id=move_db_id,
                error_message=error_message,
                snapshot_data=snapshot_str
            )

            with Session(engine) as session:
                session.add(failure_detail_record)
                session.commit()
                session.refresh(failure_detail_record)
                logger.info(f"Logged SQLModel failure detail for move DB ID {move_db_id}")

        except Exception as e:
            logger.error(f"Error logging SQLModel failure detail: {e}")

    def end_experiment(self):
        """Marks the current experiment as ended."""
        if self.current_experiment_id is None:
            logger.warning("Warning: No active SQLModel experiment to end.")
            return
        try:
            with Session(engine) as session:
                statement = select(Experiment).where(Experiment.id == self.current_experiment_id)
                results = session.exec(statement)
                experiment = results.one_or_none()
                if experiment:
                    experiment.end_time = datetime.utcnow()
                    experiment.status = "completed"
                    session.add(experiment)
                    session.commit()
                    logger.info(f"Ended SQLModel experiment ID {self.current_experiment_id}")
                else:
                    logger.warning(f"SQLModel Experiment ID {self.current_experiment_id} not found to end.")
        except Exception as e:
            logger.error(f"Error ending SQLModel experiment: {e}")
        finally:
            self.current_experiment_id = None
            self.move_counter = 0