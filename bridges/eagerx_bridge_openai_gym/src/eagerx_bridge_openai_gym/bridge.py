from eagerx_core.basebridge import BridgeBase
from eagerx_core.utils.utils import Msg

from genpy.message import Message
from typing import Optional, Dict, Union, List


class GymBridge(BridgeBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def add_object_to_simulator(self, object_params, node_params, state_params) -> Dict:
        pass

    def pre_reset(self, **kwargs: Optional[Msg]):
        pass

    def reset(self, **kwargs: Optional[Msg]):
        pass

    def callback(self, node_tick: int, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
        pass
