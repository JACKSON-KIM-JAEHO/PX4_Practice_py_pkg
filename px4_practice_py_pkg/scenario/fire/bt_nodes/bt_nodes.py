from px4_practice_py_pkg.modules.base_bt_nodes import (
    Sequence,
    Fallback,
    ReactiveSequence,
    ReactiveFallback,
    Parallel,
)
# ===============================
# Shared ROS2 Node
# ===============================
from .shared_node import ROS2NodeShared

# ===============================
# Action Nodes
# ===============================
from .actions.OffboardTakeoff import OffboardTakeoff
from .actions.BuildAirspeedHold import BuildAirspeedHold
from .actions.ReadyToTranstion import ReadyToTranstion
from .actions.TransitionToFW import TransitionToFW
from .actions.TransitionToMC import TransitionToMC

# ===============================
# Condition Nodes
# ===============================
from .conditions.IsFWMode import IsFWMode
from .conditions.ISMCMode import IsMCMode
from .conditions.IsTiltingToFW import IsTiltingToFW
from .conditions.IsTransitionPoint import IsTransitionPoint
from .conditions.IsReadyToTransitionMCToFW import IsReadyToTransitionMCToFW


class BTNodeList:
    CONTROL_NODES = [
        "Sequence",
        "Fallback",
        "ReactiveSequence",
        "ReactiveFallback",
        "Parallel",
    ]

    ACTION_NODES = [
        "OffboardTakeoff",
        "BuildAirspeedHold",
        "ReadyToTranstion",
        "TransitionToFW",
        "TransitionToMC",
    ]

    CONDITION_NODES = [
        "IsFWMode",
        "IsMCMode",
        "IsTiltingToFW",
        "IsReadyToTransitionMCToFW",
        "IsTransitionPoint",
    ]

    DECORATOR_NODES = []
