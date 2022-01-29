# ROS IMPORTS
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

# EAGERx IMPORTS
from eagerx.bridges.real.bridge import RealBridge
from eagerx.bridges.ode.bridge import OdeBridge
from eagerx.core.entities import Object, EngineNode, SpaceConverter, EngineState, Processor
from eagerx.core.specs import ObjectSpec, AgnosticSpec, SpecificSpec
from eagerx.core.engine_graph import EngineGraph
import eagerx.core.register as register


class Mops(Object):
    @staticmethod
    @register.sensors(mops_output=Float32MultiArray, action_applied=Float32MultiArray, image=Image)
    @register.actuators(mops_input=Float32MultiArray)
    @register.simstates(model_state=Float32MultiArray, model_parameters=Float32MultiArray)
    @register.agnostic_params(always_render=False, mops_rate=30, render_shape=[480, 480], opencv_cam_index=0)
    def agnostic(spec: AgnosticSpec):
        """Agnostic definition of the Mops"""
        # Register standard converters, space_converters, and processors
        import eagerx.converters  # noqa # pylint: disable=unused-import

        # Set observation properties: (space_converters, rate, etc...)
        c = Processor.make('AngleDecomposition', angle_idx=0)
        sc = SpaceConverter.make('Space_Float32MultiArray', low=[-1, -1, -9], high=[1, 1, 9], dtype='float32')
        mapping = dict(converter=c, space_converter=sc, rate='$(default mops_rate)')
        spec.set_parameters('sensors', 'mops_output', mapping)

        sc = SpaceConverter.make('Space_Float32MultiArray', low=[-3], high=[3], dtype='float32')
        mapping = dict(space_converter=sc, rate='$(default mops_rate)')
        spec.set_parameters('sensors', 'action_applied', mapping)

        sc = SpaceConverter.make('Space_Image', low=0, high=1, shape='(default render_shape)', dtype='float32')
        mapping = dict(space_converter=sc, rate='$(default mops_rate)')
        spec.set_parameters('sensors', 'image', mapping)

        # Set actuator properties: (space_converters, rate, etc...)
        sc = SpaceConverter.make('Space_Float32MultiArray', low=[-3], high=[3], dtype='float32')
        mapping = dict(space_converter=sc, rate='$(default mops_rate)', window=1)
        spec.set_parameters('actuators', 'mops_input', mapping)

        # Set model_state properties: (space_converters)
        sc = SpaceConverter.make('Space_Float32MultiArray', low=[-3.14159265359, -9], high=[3.14159265359, 9], dtype='float32')
        spec.set_space_converter('states', 'model_state', sc)

        # Set model_parameters properties: (space_converters)
        low = [1.7955e-04, 5.3580e-02, 4.1610e-02, 1.3490e-04, 4.7690e-02, 9.3385e+00, 1.4250e+00, 1.7480e-03]
        high = [1.98450e-04, 5.92200e-02, 4.59900e-02, 1.49100e-04, 5.27100e-02, 1.03215e+01, 1.57500e+00, 1.93200e-03]
        sc = SpaceConverter.make('Space_Float32MultiArray', low=low, high=high, dtype='float32')
        spec.set_space_converter('states', 'model_parameters', sc)

    @staticmethod
    @register.spec('Mops', Object)
    def spec(spec: ObjectSpec, name: str, sensors=['mops_output', 'action_applied', 'image'], states=['model_state'],
             mops_rate=30, always_render=False, render_shape=[480, 480], camera_index=2):
        """Object spec of Mops"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(Mops)

        # Modify default node params
        # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
        default = dict(name=name, sensors=sensors, actuators=['mops_input'], states=states)
        spec.set_parameters(default)

        # Add custom params
        params = dict(mops_rate=mops_rate, always_render=always_render, render_shape=render_shape, opencv_cam_index=camera_index)
        spec.set_parameters(params)

        # Add bridge implementation
        Mops.ode_bridge(spec)
        Mops.real_bridge(spec)

    @classmethod
    @register.bridge(OdeBridge)   # This decorator pre-initializes bridge implementation with default object_params
    def ode_bridge(cls, spec: SpecificSpec, graph: EngineGraph):
        """Engine-specific implementation (OdeBridge) of the object."""
        # Import any object specific entities for this bridge
        import eagerx_dcsc_setups.mops.ode  # noqa # pylint: disable=unused-import

        # Set object arguments (nothing to set here in this case)
        object_params = dict(ode='eagerx_dcsc_setups.mops.ode.mops_ode/mops_ode')
        spec.set_parameters(object_params)

        # Create simstates (no agnostic states defined in this case)
        spec.set_state('model_state', EngineState.make('OdeSimState'))
        spec.set_state('model_parameters', EngineState.make('OdeSimState'))

        # Create sensor engine nodes
        # Rate=None, because we will connect them to sensors (thus uses the rate set in the agnostic specification)
        obs = EngineNode.make('OdeOutput', 'obs', rate=None, process=2)
        applied = EngineNode.make('ActionApplied', 'applied', rate=None, process=2)
        image = EngineNode.make('MopsImage', 'image', shape='$(default render_shape)', rate=None, process=0)

        # Create actuator engine nodes
        # Rate=None, because we will connect it to an actuator (thus uses the rate set in the agnostic specification)
        action = EngineNode.make('OdeInput', 'action', rate=None, process=2)

        # Connect all engine nodes
        graph.add([obs, applied, image, action])
        graph.connect(source=('obs', 'outputs', 'observation'), sensor='mops_output')
        graph.connect(source=('action', 'outputs', 'action_applied'), target=('applied', 'inputs', 'action_applied'), skip=True)
        graph.connect(source=('applied', 'outputs', 'action_applied'), sensor='action_applied')
        graph.connect(source=('obs', 'outputs', 'observation'), target=('image', 'inputs', 'theta'))
        graph.connect(source=('image', 'outputs', 'image'), sensor='image')
        graph.connect(actuator='mops_input', target=('action', 'inputs', 'action'))

        # Check graph validity (commented out)
        # graph.is_valid(plot=True)

    @classmethod
    @register.bridge(RealBridge)   # This decorator pre-initializes bridge implementation with default object_params
    def real_bridge(cls, spec: SpecificSpec, graph: EngineGraph):
        """Engine-specific implementation (RealBridge) of the object."""
        # Import any object specific entities for this bridge
        import eagerx_dcsc_setups.mops.real  # noqa # pylint: disable=unused-import

        # Set object arguments (nothing to set here in this case)
        object_params = dict(driver_launch_file='$(find eagerx_dcsc_setups)/launch/mops.launch')
        spec.set_parameters(object_params)

        # Create simstates (no agnostic states defined in this case)
        spec.set_state('model_state', EngineState.make('RandomActionAndSleep', sleep_time=1.0, repeat=1))

        # Create sensor engine nodes
        # Rate=None, because we will connect them to sensors (thus uses the rate set in the agnostic specification)
        obs = EngineNode.make('MopsOutput', 'obs', rate=None, process=0)
        applied = EngineNode.make('ActionApplied', 'applied', rate=None, process=0)
        image = EngineNode.make('CameraRender', 'image', camera_idx='$(default opencv_cam_index)', shape='$(default render_shape)', rate=None, process=0)

        # Create actuator engine nodes
        # Rate=None, because we will connect it to an actuator (thus uses the rate set in the agnostic specification)
        action = EngineNode.make('MopsInput', 'action', rate=None, process=0)

        # Connect all engine nodes
        graph.add([obs, applied, image, action])
        graph.connect(source=('obs', 'outputs', 'mops_output'), sensor='mops_output')
        graph.connect(source=('action', 'outputs', 'action_applied'), target=('applied', 'inputs', 'action_applied'), skip=True)
        graph.connect(source=('applied', 'outputs', 'action_applied'), sensor='action_applied')
        graph.connect(source=('image', 'outputs', 'image'), sensor='image')
        graph.connect(actuator='mops_input', target=('action', 'inputs', 'mops_input'))

        # Check graph validity (commented out)
        # graph.is_valid(plot=True)