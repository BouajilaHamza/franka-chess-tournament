from pydantic import BaseModel,ConfigDict
from typing import Optional
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
    
    

class ExperimentData(BaseModel):
    id: Optional[int] = None
    name: str
    start_time: str
    end_time: Optional[str] = None
    config_hash: Optional[str] = None
    git_commit: Optional[str] = None
    notes: Optional[str] = None
    status: str
    
    moves: Optional[list["MoveData"]]


class MoveData(BaseModel):
    model_config = ConfigDict(validate_assignment=True) # Enable validation on assignment

    id: int = None
    move_number: int = None
    timestamp: str = None
    robot_name: str = None
    robot_color: str = None
    source_square: Optional[str] = None
    target_square: Optional[str] = None
    piece_type: Optional[str] = None
    piece_id: Optional[int] = None
    success: bool = None
    failure_type: Optional[FailureTypeEnum] = None
    total_time_seconds: Optional[float] = None
    planning_time_seconds: Optional[float] = None
    execution_time_seconds: Optional[float] = None
    placement_error_mm: Optional[float] = None
    min_collision_proximity_mm: Optional[float] = None
    algorithm_used: str = None
    retries: int = None
    attempt_number: Optional[int] = 0
    experiment:Optional[ExperimentData] = None
    failure_details: Optional[list["FailureDetail"]] = None

class FailureDetail(BaseModel):
    id: Optional[int] = None
    move_id: int
    error_message: Optional[str] = None
    snapshot_data: Optional[str] = None
    move:Optional["MoveData"]

