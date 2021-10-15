# ROS packages required
import rospy
from eagerx_core.params import RxObjectParams, RxNodeParams
from eagerx_core.env import Env
from eagerx_core.utils.utils import launch_roscore

if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.INFO)

    # Test converter
    input_converters = {'in_1': {'converter_type': 'eagerx_core.converter/IntUInt64Converter',
                                 'test_arg': 'test'}}

    # Nodes
    # todo: Connect i/o after creating each node. This will fill the "inputs/outputs/states/feedthroughs" arguments in default!
    # todo: While creating the node, one must already specify the converters? Or, only when making the connection?
    # todo: Probably, output converter must be defined at creation. Input converter must be created when making a connection.
    # todo: How to connect Nodes with the Nodes of objects?
    # todo: Standardize the addresses? E.g. /rx/node_name/inputs/alias, /rx/node_name/outputs/alias
    # todo: .connect_input(..), connect_feedthrough(...), connect_state(...)
    # todo: how to infer space from ProcessNode --> for each input/output, a 'space_to_i/o' converter must be specified.
    # (Node, output, converter, Node, cname, c)
    # ((N3, 'out_1'), None, (N1, 'inputs', 'in_1'))                 todo: node_output --> node_input
    # ((N3, 'out_1'), None, (viper, 'actuators', 'N7', 'action_1')) todo: node_output --> obj_actuator input
    # ('obj/states/N8', None, (N3, 'states', 'state_1'))            todo: address --> node_state  (Cannot perform any checks on message types
    N1 = RxNodeParams.create('N1', 'eagerx_core', 'process',   rate=1, inputs={'in_1': 'N0/P0'}, outputs={'out_1': 'N1/P1', 'out_2': 'N2/P2'},
                             input_converters=input_converters)
    N3 = RxNodeParams.create('N3', 'eagerx_core', 'realreset', rate=1, inputs={'in_1': 'N1/P1'}, outputs={'out_1': 'N3/P3'}, states={'state_1': 'obj/states/N8'}, feedthroughs={'out_1': 'N2/P2'})
    N4 = RxNodeParams.create('N4', 'eagerx_core', 'process',   rate=3, inputs={'in_1': 'N3/P3'}, outputs={'out_1': 'N4/P4'})
    N5 = RxNodeParams.create('N5', 'eagerx_core', 'process',   rate=8, inputs={'in_1': 'N4/P4'}, outputs={'out_1': 'obj/actuators/N7'})

    # Define bridge
    bridge = RxNodeParams.create('bridge', 'eagerx_core', 'bridge', rate=1, num_substeps=10)

    # Define object
    viper = RxObjectParams.create('obj', 'eagerx_core', 'viper', position=[1, 1, 1], actuators=['N7'])

    # Define connections
    # N1_to_N3_converter = {'converter_type': 'eagerx_core.converter/IntUInt64Converter', 'test_arg': 'test'}
    # connect = [{'from': (N1, 'out_1'), 'to': (N3, 'inputs',       'in_1'), 'converter': N1_to_N3_converter},
    #            {'from': (N1, 'out_2'), 'to': (N3, 'feedthroughs', 'out_1'), 'converter': None},
    #            ]

    # Define environment actions and observations: dict[obj/node][cname]
    actions, observations = dict(), dict()
    actions.update(N1.get_action('in_1'))
    observations.update(viper.get_state('N9'))

    # Initialize Environment
    env = Env(name='rx',
              rate=1,
              actions=actions,
              observations=observations,
              bridge=bridge,
              nodes=[N1, N3, N4, N5])

    # Register objects
    # todo: register_node(node=...)
    env.register_object(object=viper)

    # Initialize nodes
    env.initialize_node_pipelines()

    # todo: remove this sleep & connect
    rospy.sleep(1.0)
    env.mb.connect_io()
    env.mb.print_io_status()

    rospy.loginfo("Training starts.")
    env.reset()
    # todo: CheckEnv(env): i/o correct, fully connected when RealReset without Env, no duplicate addresses in Env actions
    # todo: make env.reset() more robust (i.e. something with .wait) so that sleep inside reset_cb not necessary
    #  perhaps use future for this? (https://github.com/ReactiveX/RxPY/issues/528)
    # todo: Visualize and perform checks on DAGs (https://mungingdata.com/python/dag-directed-acyclic-graph-networkx/, https://pypi.org/project/graphviz/)
    # todo: Property of DAG when performing real reset?

    # todo: implement env rx pipeline
    # todo: implement real_time rx pipeline

    # todo: remove tick as input to bridge
    # todo: How to infer spaces --> use converter "SpaceUInt64Converter". How to pass converter args?
    # todo: input topic msg_type must be inferred from param server, or add converter
    # todo: differentiate between real_reset and sim_reset states in StateNode.reset(...)
    # todo: replace rospy.sleep(..) with time.sleep(..)
    # todo: remove feedthrough_rate==node_rate constraint for real_reset by repeating last received feedthrough action
    # todo: state_address/set uses UInt64, but must use msg_type from config.
    # todo: Replace reset info with rospy.logdebug(...), so that we log it if warn level is debug
    # todo; How to deal with ROS messages in single_process? Risk of changing content & is it threadsafe? copy-on-write?

    for j in range(20000):
        print('\n[Episode %s]' % j)
        for i in range(3):
            env.step()
        env.reset()
    print('\n[Finished]')
    rospy.sleep(100000)
