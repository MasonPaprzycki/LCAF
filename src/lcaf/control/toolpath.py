from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Dict
from typing import Any

class OperationType(Enum):
    STRIKE = auto()
    REHEAT = auto()
    INSPECT = auto()
    MOVE_ONLY = auto()
    DWELL = auto()
    CUSTOM = auto()

@dataclass
class ToolpathOperation:

    step: int
    operation: OperationType

    x: float
    y: float
    die_gap: float
    rotation: float

    target_temperature: float

    metadata: Dict[str, Any] = field(default_factory=dict)