from sqlmodel import create_engine, Session, SQLModel
import logging

logger = logging.getLogger(__name__)

DATABASE_URL = "sqlite:///./robot_performance.db"

# Create engine
engine = create_engine(DATABASE_URL, echo=True) # echo=True for SQL logging, disable in production

def create_db_and_tables():
    """Creates the database and tables."""
    SQLModel.metadata.create_all(engine)
    logger.info("Database and tables created (if they didn't exist).")

def get_session():
    """Provides a session for database interactions."""
    with Session(engine) as session:
        yield session

# def initialize_database():
#     create_db_and_tables()

    # try:
    #     with Session(engine) as session:
    #         # Example: Create a dummy experiment to test
    #         dummy_exp = Experiment(name="SQLModel_Test_Setup")
    #         session.add(dummy_exp)
    #         session.commit()
    #         session.refresh(dummy_exp)
    #         logger.info(f"Test experiment created with ID: {dummy_exp.id}")
    #         # Clean up
    #         session.delete(dummy_exp)
    #         session.commit()
    #         logger.info("Test experiment cleaned up.")
    # except Exception as e:
    #     logger.error(f"Error testing database connection/creation: {e}")
