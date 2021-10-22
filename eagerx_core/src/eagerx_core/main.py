# ROS packages required
import rospy
from eagerx_core.params import RxObjectParams, RxNodeParams
from eagerx_core.env import Env
from eagerx_core.utils.utils import launch_roscore
from eagerx_core.utils.node_utils import configure_connections

if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.INFO)

    # Define converter
    IntUInt64Converter = {'converter_type': 'eagerx_core.converter/IntUInt64Converter', 'test_arg': 'test'}

    # Define nodes
    N1 = RxNodeParams.create('N1', 'eagerx_core', 'process',   rate=1, outputs={'out_1': 'N1/P1', 'out_2': 'N2/P2'})
    N3 = RxNodeParams.create('N3', 'eagerx_core', 'realreset', rate=1, outputs={'out_1': 'N3/P3'})
    N4 = RxNodeParams.create('N4', 'eagerx_core', 'process',   rate=3, outputs={'out_1': 'N4/P4'}, output_converters={'out_1': IntUInt64Converter})
    N5 = RxNodeParams.create('N5', 'eagerx_core', 'process',   rate=8, outputs={'out_1': 'N5/P5'}, output_converters={'out_1': IntUInt64Converter})

    # Define object
    viper = RxObjectParams.create('obj', 'eagerx_core', 'viper', position=[1, 1, 1], actuators=['N7'])

    # Define action/observations/states
    actions, observations, states = Env.define_actions(), Env.define_observations(), Env.define_states()

    # Connect nodes
    connections = [{'source': (viper, 'states', 'N9'),  'target': (observations, 'obs_1', 'all')},
                   {'source': (actions, 'act_1'),       'target': (N1, 'inputs', 'in_1')},
                   {'source': (viper, 'sensors', 'N6'), 'target': (N3, 'inputs', 'in_1')},
                   {'source': (N1, 'out_2'),            'target': (N3, 'feedthroughs', 'out_1')},
                   {'source': (N3, 'out_1'),            'target': (N4, 'inputs', 'in_1')},
                   {'source': (N4, 'out_1'),            'target': (N5, 'inputs', 'in_1'), 'converter': IntUInt64Converter},
                   {'source': (N5, 'out_1'),            'target': (viper, 'actuators', 'N7'), 'converter': IntUInt64Converter},
                   {'source': (states, 'N1'),           'target': (N1, 'states', 'state_1')},
                   {'source': (states, 'N4'),           'target': (N4, 'states', 'state_1')},
                   {'source': (states, 'N5'),           'target': (N5, 'states', 'state_1')},
                   {'source': (states, 'N8'),           'target': (viper, 'states', 'N8')},
                   {'source': (states, 'N8'),           'target': (N3, 'states', 'state_1')},  # todo: meaning of states is different here
                   {'source': (states, 'N9'),           'target': (viper, 'states', 'N9')},
                   ]
    configure_connections(connections)

    # Define bridge
    bridge = RxNodeParams.create('bridge', 'eagerx_core', 'bridge', rate=1, num_substeps=10)

    # Initialize Environment
    env = Env(name='rx',
              rate=1,
              actions=actions,
              observations=observations,
              states=states,
              bridge=bridge,
              nodes=[N1, N3, N4, N5])

    # Register objects
    env.register_object(object=viper)

    # First reset
    obs = env.reset()
    for j in range(20000):
        print('\n[Episode %s]' % j)
        for i in range(3):
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
        obs = env.reset()
    print('\n[Finished]')
    rospy.sleep(100000)

    # todo: CheckEnv(env): i/o correct, fully connected when RealReset without Env
    # todo: Visualize and perform checks on DAGs (https://mungingdata.com/python/dag-directed-acyclic-graph-networkx/, https://pypi.org/project/graphviz/)
    # todo: Make graph gui (https://towardsdatascience.com/visualizing-networks-in-python-d70f4cbeb259)
    # todo: Verify DAG properties in case of a real reset

    # todo: implement env rx pipeline
    # todo: implement real_time rx pipeline

    # todo: HINT: check-out old version of env.py on desktop for flow of events for a reset.
    # todo: are states still passed to simstate node (or, nodes in general)?
    # todo: pre-define (reset_)states (similar to actions & observations), with converter and state_space
    # todo: state_address/set uses UInt64, but must use msg_type from config.
    # todo: Also add states as input to reset(..) of realreset node --> separate to states & reset_states

    # todo: how can the converter infer msg_type when providing string as "from"?
    # todo: how to define action/observation when providing string as "from"?
    # todo: resolve the "sleep" & "connect" in env.reset()

    # todo: cleanup eagerx_core.__init__: refactor to separate rxpipelines.py, rxoperators.py
    # todo: change structure of callback/reset input: unpack to descriptive arguments e.g. reset(tick, state)
    # todo: print statements of callback inside ProcessNode: color specified as additional argument
    # todo: Create a register_node function in the RxNode class to initialize a node inside the process of another node.
    # todo: differentiate between real_reset and sim_reset states in StateNode.reset(...)
    # todo: replace rospy.sleep(..) with time.sleep(..)
    # todo: replace reset info with rospy.logdebug(...), so that we log it if warn level is debug
    # todo; how to deal with ROS messages in single_process? Risk of changing content & is it threadsafe? copy-on-write?
