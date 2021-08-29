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
                       states=['position'], states_address=['obj/states/position'])
    N4 = ProcessNode(name='N4', rate=1, inputs=['P3'], inputs_address=['N3/P3'], outputs=['P4'], outputs_address=['N4/P4'])
    N5 = ProcessNode(name='N5', rate=8, inputs=['P4'], inputs_address=['N4/P4'], outputs=['P5'], outputs_address=['N5/P5'])

    # Define bridge
    bridge = RxBridgeParams(rate=10, num_substeps=1)

    # Define object
    object = RxObjectParams.create('obj', 'eagerx_core', 'viper', position=[1, 1, 1], actuators=['ref_pos'])

    # Initialize Environment
    env = Env(name='rx',
              actions=[RxOutput('Pe', 'env/Pe', 'UInt64', rate=1)],
              observations=[RxInput('position', 'obj/states/orientation', 'UInt64')],
              bridge=bridge,
              nodes=[N1, N3, N4, N5])

    # Register objects
    env.register_object(object=object)

    env.initialize_node_pipelines()

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
    # todo: ops.with_latest_from(SS) could lead to resets with old states

    # todo: clean-up parameters per node on ros parameter server
    #  5) Add rate to RxInput, as non_reactive requires a rate

    # todo: create reactive register pipeline.
    #  1) Create Subjects to accumulate stepper inputs, reset_states and integrate into stepper pipeline.
    #  4) Make bridge send reset msg (Is) for non_reactive topics.

    for j in range(20000):
        print('\n[Episode %s]' % j)
        for i in range(6):
            env.step()
        env.reset()
    print('\n[Finished]')
    rospy.sleep(100000)
