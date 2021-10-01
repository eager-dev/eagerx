# ROS packages required
import rospy
from eagerx_core.params import RxBridgeParams, RxInput, RxOutput, RxObjectParams, RxNodeParams, RealResetNode
from eagerx_core.env import Env
from eagerx_core.utils.utils import launch_roscore

if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.INFO)

    # Nodes
    # todo: how to infer space from ProcessNode --> for each input/output, a 'space_to_i/o' converter must be specified.
    # todo: node.add_action('name') --> ('msg', 'RxOutput', 'converter', 'space') --> perhaps put converter inside RxOutput?
    N1 = RxNodeParams.create('N1', 'eagerx_core', 'process', rate=1, inputs={'in_1': 'env/Pe'},
                             outputs={'out_1': 'N1/P1', 'out_2': 'N2/P2'})
    N3 = RxNodeParams.create('N3', 'eagerx_core', 'realreset', rate=1, inputs={'in_1': 'N1/P1'}, outputs={'out_1': 'N3/P3'},
                             states={'state_1': 'obj/states/N8'}, feedthroughs={'ft_1': 'N2/P2'})
    N4 = RxNodeParams.create('N4', 'eagerx_core', 'process', rate=4, inputs={'in_1': 'N3/P3'}, outputs={'out_1': 'N4/P4'})
    N5 = RxNodeParams.create('N5', 'eagerx_core', 'process', rate=8, inputs={'in_1': 'N4/P4'}, outputs={'out_1': 'obj/actuators/N7'})

    # Define bridge
    bridge = RxNodeParams.create('bridge', 'eagerx_core', 'bridge', rate=1, num_substeps=10)

    # Define object
    viper = RxObjectParams.create('obj', 'eagerx_core', 'viper', position=[1, 1, 1], actuators=['N7'])

    # Define environment actions and observations
    actions, observations, states = dict(), dict(), dict()
    actions.update(viper.get_action())  # dict['obj']['N7'] = ('msg', RxOutput) msg = RxOutput.converter(), space = RxOutput.infer_space()
    # actions.update(N1.get_action('Pe'))  # dict['N1']['Pe'] = ('msg', RxInput) msg = RxInput.converter(), space = RxInput.infer_space()

    # Initialize Environment
    # todo: check no duplicate addresses in actions
    # todo: change rates of all actions to Env rate
    env = Env(name='rx',
              actions=[RxOutput('Pe', 'env/Pe', 'UInt64', rate=1)],
              observations=[RxInput('N9', 'obj/states/N9', 'UInt64')],
              bridge=bridge,
              # nodes=[])    # todo: this is not working, because we block in done_handler for a "true" flag of N8, but a sim reset will not send a true flag!
              nodes=[N1, N3, N4, N5])

    # Register objects
    # todo: register_node(node=...)
    env.register_object(object=viper)

    # Initialize nodes
    env.initialize_node_pipelines()

    # todo: remove this sleep & connect
    rospy.sleep(1.0)
    env.mb.connect_io()
    # env.mb.print_io_status()

    rospy.loginfo("Training starts.")
    env.reset()
    # todo: CheckEnv(env): i/o correct, fully connected when RealReset without Env, no duplicate addresses in Env actions
    # todo: make env.reset() more robust (i.e. something with .wait) so that sleep inside reset_cb not necessary
    #  perhaps use future for this? (https://github.com/ReactiveX/RxPY/issues/528)
    # todo: Visualize and perform checks on DAGs (https://mungingdata.com/python/dag-directed-acyclic-graph-networkx/, https://pypi.org/project/graphviz/)
    # todo: Property of DAG when performing real reset?
    # todo: implement bridge rx pipeline
    # todo: input topic msg_type must be inferred from param server, or add converter
    # todo: implement env rx pipeline (how to infer spaces --> use converter "ROSmsg_to_space". How to pass converter args?)
    #  Perhaps, make convert classes that have an "infer_space" method.
    # todo: implement real_time rx pipeline
    # todo: differentiate between real_reset and sim_reset states in StateNode.reset(...)
    # todo: create RxDeadLockResolver that tracks all reset messages and re-sends them after a timeout period.
    # todo: replace rospy.sleep(..) with time.sleep(..)
    # todo: state_address/set uses UInt64, but must use msg_type from config.
    # todo; How to deal with ROS messages in single_process? Risk of changing content & is it threadsafe? copy-on-write?
    # todo: msg keys in output streams must be unique for bridge inputs. (i.e., SimNodes cannot use the same name shadow for their address).
    #  Or, we don't care because the bridge does not perform any computation with the inputs. Create a "null" converter?

    for j in range(20000):
        print('\n[Episode %s]' % j)
        for i in range(3):
            env.step()
        env.reset()
    print('\n[Finished]')
    rospy.sleep(100000)
