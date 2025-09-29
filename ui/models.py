from sqlmodel import SQLModel, Field, Relationship
from typing import Optional, List
from datetime import datetime
import enum


class FailureTypeEnum(str, enum.Enum):
    IK = "IK"
    OMPL = "OMPL"
    GRASP = "Grasp"
    EXECUTION = "Execution"
    TIMEOUT = "Timeout"

class AlgorithmUsedEnum(str, enum.Enum):
    IK = "IK"
    OMPL = "OMPL"
    HYBRID = "Hybrid"

class Experiment(SQLModel, table=True,extend_existing=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str = Field(unique=True, index=True)
    start_time: datetime = Field(default_factory=datetime.utcnow)
    end_time: Optional[datetime] = None
    config_hash: Optional[str] = None
    git_commit: Optional[str] = None
    notes: Optional[str] = None
    status: str = Field(default="running") # Consider using Enum

    # Relationships (Optional but powerful)
    moves: List["Move"] = Relationship(back_populates="experiment")

class Move(SQLModel, table=True,extend_existing=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    experiment_id: int = Field(foreign_key="experiment.id", index=True)
    move_number: int
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    source_square: Optional[str] = None
    target_square: Optional[str] = None
    piece_type: Optional[str] = None
    success: bool
    failure_type: Optional[FailureTypeEnum] = None # Using Enum
    total_time_seconds: Optional[float] = None
    planning_time_seconds: Optional[float] = None
    execution_time_seconds: Optional[float] = None
    placement_error_mm: Optional[float] = None
    min_collision_proximity_mm: Optional[float] = None
    algorithm_used: Optional[AlgorithmUsedEnum] = None # Using Enum
    retries: int = Field(default=0)

    # Relationships
    experiment: Optional[Experiment] = Relationship(back_populates="moves")
    failure_details: List["FailureDetail"] = Relationship(back_populates="move")

class FailureDetail(SQLModel, table=True,extend_existing=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    move_id: int = Field(foreign_key="move.id", index=True)
    error_message: Optional[str] = None
    snapshot_data: Optional[str] = None # Could store JSON string or path

    # Relationships
    move: Optional[Move] = Relationship(back_populates="failure_details")

# --- Optional: Pre-calculated Metrics Model ---
# You could still have a table for this, updated periodically
# class ExperimentMetric(SQLModel, table=True):
#     experiment_id: int = Field(primary_key=True, foreign_key="experiment.id")
#     # ... other metric fields (success_rate_percent, etc.) ...