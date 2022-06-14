from typing import Optional
from scipy.signal import butter, sosfilt
import numpy as np
from gym.spaces import Box

# IMPORT EAGERX
import eagerx
import eagerx.core.register as register
from eagerx.utils.utils import Msg


class ButterworthFilter(eagerx.Node):
    @staticmethod
    @register.spec("ButterworthFilter", eagerx.Node)
    def spec(
        spec,
        name: str,
        rate: float,
        index: int = 0,
        N: int = 2,
        Wn: float = 1,
        btype: str = "lowpass",
        process: Optional[int] = eagerx.NEW_PROCESS,
        color: Optional[str] = "grey",
    ):
        """
        Butterworth filter implementation based on scipy.signal/butter, scipy.signal/sosfilt.

        :param spec: Not provided by user.
        :param name: Node name
        :param rate: Rate at which callback is called.
        :param index: Index (related to Float32MultiArray.data[index])
        :param N: The order of the filter
        :param Wn: The critical frequency or frequencies
        :param btype: {'lowpass', 'highpass', 'bandpass', 'bandstop'}
        :param process: {0: NEW_PROCESS, 1: ENVIRONMENT, 2: ENGINE, 3: EXTERNAL}
        :param color: console color of logged messages. {'black', 'red', 'green', 'yellow', 'blue', 'magenta', 'cyan', 'white', 'grey'}
        :return:
        """
        # Modify default node params
        spec.config.update(name=name, rate=rate, process=process, color=color, inputs=["signal"], outputs=["filtered"])

        # Modify custom node params
        spec.config.update(N=N, Wn=Wn, btype=btype)

        # Add converter & space_converter
        spec.inputs.signal.window = "$(config N)"
        spec.inputs.signal.processor = eagerx.Processor.make("GetIndex", index=index)

    def initialize(self, N, Wn, btype):
        for i in self.inputs:
            if i["name"] == "signal":
                assert (
                    int(i["window"]) >= N
                ), "The window size of the signal {} is too small to create a filter with order {}.".format(i["window"], N)
        self.filter = butter(N, Wn, btype, output="sos", fs=self.rate)
        self.N = N

    @register.states()
    def reset(self):
        pass

    @register.inputs(signal=Box(low=-3, high=-3, shape=(1,), dtype="float32"))
    @register.outputs(filtered=Box(low=-3, high=-3, shape=(1,), dtype="float32"))
    def callback(self, t_n: float, signal: Optional[Msg] = None):
        if len(signal.msgs) >= self.N:
            unfiltered = [signal.msgs[i][0] for i in range(-self.N, 0)]
            filtered = np.array([sosfilt(self.filter, unfiltered)[-1]], dtype="float32")
        else:
            filtered = signal.msgs[-1]
        return dict(filtered=filtered)
