

# ROS packages required
import rospy
from eagerx_core.params import RxBridgeParams, RxInput, RxOutput, RealResetNode, RxObjectParams
from eagerx_core.params import ProcessNode
from eagerx_core.env import Env
from eagerx_core.utils.utils import launch_roscore

if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eagerx_core', anonymous=True, log_level=rospy.INFO)

    # Nodes
    N1 = ProcessNode(name='N1', rate=1, inputs=['Pe'], inputs_address=['env/Pe'], outputs=['P1', 'P2'], outputs_address=['N1/P1', 'N2/P2'])
    N3 = RealResetNode(name='N3', rate=1, extra_arg='test', inputs=['P1'], inputs_address=['N1/P1'], outputs=['P3'], outputs_address=['N3/P3'],
                       feedthrough_address=['N2/P2'], feedthrough_to=['P3'], feedthrough_converter=['UInt64_to_int'], feedthrough_converter_module=['eagerx_core'],
                       states=['N8'], states_address=['obj/states/N8'])
    N4 = ProcessNode(name='N4', rate=4, inputs=['P3'], inputs_address=['N3/P3'], outputs=['P4'], outputs_address=['N4/P4'])
    N5 = ProcessNode(name='N5', rate=8, inputs=['P4'], inputs_address=['N4/P4'], outputs=['P5'], outputs_address=['N5/P5'])

    # Define bridge
    bridge = RxBridgeParams(rate=1, num_substeps=1)

    # Define object
    object = RxObjectParams.create('obj', 'eagerx_core', 'viper', position=[1, 1, 1], actuators=['N7'])

    # Initialize Environment
    env = Env(name='rx',
              actions=[RxOutput('Pe', 'env/Pe', 'UInt64', rate=1)],
              observations=[RxInput('N9', 'obj/states/N9', 'UInt64')],
              bridge=bridge,
              # nodes=[])    # todo: this is not working, because we block in done_handler for a "true" flag of N8, but a sim reset will not send a true flag!
              nodes=[N1, N3, N4, N5])

    # Register objects
    env.register_object(object=object)

    env.initialize_node_pipelines()

    # todo: remove this sleep & connect
    rospy.sleep(1.0)
    env.mb.connect_io()
    # env.mb.print_io_status()

    rospy.loginfo("Training starts.")
    env.reset()
    # todo: make env.reset() more robust (i.e. something with .wait) so that sleep inside reset_cb not necessary
    #  perhaps use future for this? (https://github.com/ReactiveX/RxPY/issues/528)
    # todo: Visualize and perform checks on DAGs (https://mungingdata.com/python/dag-directed-acyclic-graph-networkx/, https://pypi.org/project/graphviz/)
    # todo: implement bridge rx pipeline
    # todo: implement env rx pipeline (how to infer spaces --> use converter "ROSmsg_to_space". How to pass converter args?)
    #  Perhaps, make convert classes that have an "infer_space" method.
    # todo: implement real_time rx pipeline
    # todo: differentiate between real_reset and sim_reset states in StateNode.reset(...)
    # todo: create RxDeadLockResolver that tracks all reset messages and resends them after a timeout period.
    # todo: replace rospy.sleep(..) with time.sleep(..)
    # todo: state_address/set uses UInt64, but must use msg_type from config.
    # todo; How to deal with ROS messages in single_process? Risk of changing content & is it threadsafe?
    # todo: msg keys in output streams must be unique for bridge inputs. (i.e., SimNodes cannot use the same name shadow for their address).
    #  Or, we don't care because the bridge does not perform any computation with the inputs. Create a "null" converter?

    for j in range(20000):
        print('\n[Episode %s]' % j)
        for i in range(5):
            env.step()
        env.reset()
    print('\n[Finished]')
    rospy.sleep(100000)
