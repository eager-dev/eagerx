from typing import Optional
from scipy.signal import butter, sosfilt

# IMPORT ROS
from std_msgs.msg import Float32MultiArray

# IMPORT EAGERX
import eagerx.core.register as register
from eagerx.utils.utils import return_typehint
from eagerx.core.entities import Node, Processor, SpaceConverter
from eagerx.core.constants import process


class ButterworthFilter(Node):
    @staticmethod
    @register.spec("ButterworthFilter", Node)
    def spec(
        spec,
        name: str,
        rate: float,
        index: int = 0,
        N: int = 2,
        Wn: float = 1,
        btype: str = "lowpass",
        process: Optional[int] = process.NEW_PROCESS,
        color: Optional[str] = "grey",
    ):
        """
        Butterworth filter implementation based on scipy.signal/butter, scipy.signal/sosfilt.

        :param spec: Not provided by user.
        :param name: Node name
        :param rate: Rate at which callback is called.
        :param index: Index of the
        :param N: The order of the filter
        :param Wn: The critical frequency or frequencies
        :param btype: {'lowpass', 'highpass', 'bandpass', 'bandstop'}
        :param process: {0: NEW_PROCESS, 1: ENVIRONMENT, 2: BRIDGE, 3: EXTERNAL}
        :param color: console color of logged messages. {'black', 'red', 'green', 'yellow', 'blue', 'magenta', 'cyan', 'white', 'grey'}
        :return:
        """
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(ButterworthFilter)

        # Modify default node params
        params = dict(
            name=name,
            rate=rate,
            process=process,
            color=color,
            inputs=["signal"],
            outputs=["filtered"],
        )
        spec.set_parameters(params)

        # Modify custom node params
        spec.set_parameters({"N": N, "Wn": Wn, "btype": btype})

        # Register standard converters, space_converters, and processors

        # Add converter & space_converter
        c = Processor.make("GetIndex_Float32MultiArray", index=index)
        sc = SpaceConverter.make("Space_Float32MultiArray", [-3], [3], dtype="float32")
        mapping = dict(window="$(default N)", space_converter=sc, converter=c)
        spec.set_parameters(mapping, "inputs", "signal")

    def initialize(self, N, Wn, btype):
        for i in self.inputs:
            if i["name"] == "signal":
                assert (
                    int(i["window"]) >= N
                ), "The window size of the signal {} is too small to create a filter with order {}.".format(
                    i["window"], N
                )
        self.filter = butter(N, Wn, btype, output="sos", fs=self.rate)
        self.N = N

    @register.states()
    def reset(self):
        pass

    @register.inputs(signal=Float32MultiArray)
    @register.outputs(filtered=Float32MultiArray)
    def callback(self, t_n: float, signal: Optional[Float32MultiArray] = None) -> return_typehint(Float32MultiArray):
        msgs = signal.msgs
        if len(msgs) >= self.N:
            unfiltered = [msgs[i].data[0] for i in range(-self.N, 0)]
            filtered = msgs[-1].data if None in unfiltered else [sosfilt(self.filter, unfiltered)[-1]]
        elif len(msgs) > 0:
            filtered = msgs[-1].data
        else:
            filtered = [0.0]
        return dict(filtered=Float32MultiArray(data=filtered))
