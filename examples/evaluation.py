from time import perf_counter

# EAGERx imports
from eagerx import Object, Bridge, Node
from eagerx import initialize, log, process as p

initialize("eagerx_core", anonymous=True, log_level=log.DEBUG)

# Environment imports
from eagerx.core.env import EagerEnv
from eagerx.core.graph import Graph

# Implementation specific
import eagerx.bridges.test  # noqa # pylint: disable=unused-import


def evaluation(num_nodes, is_reactive, rtf, process):
    initialize("eagerx_core", anonymous=True, log_level=log.DEBUG)

    # Define object
    obj = Object.make("Evaluation", "eval", process=process)

    # Define graph
    graph = Graph.create(objects=[obj])
    graph.connect(action="act_1", target=obj.actuators.act_1)

    # Add nodes to agnostic graph
    if num_nodes == 0:
        graph.connect(source=obj.sensors.sens_1, observation="obs_1")
    else:
        nodes = []
        n = Node.make("EvalNode", f"node_{0}", 1, process=process, color="blue")
        graph.add(n)
        nodes.append(n)
        for idx in range(1, num_nodes):
            n = Node.make("EvalNode", f"node_{idx}", 1, process=process, color="blue")
            graph.add(n)
            nodes.append(n)

            # Interconnect nodes
            graph.connect(source=nodes[idx - 1].outputs.out_1, target=n.inputs.in_1)

        # Add connection between env and object
        graph.connect(source=nodes[-1].outputs.out_1, observation="obs_1")
        graph.connect(source=obj.sensors.sens_1, target=nodes[0].inputs.in_1)

    graph.gui()

    # Define bridge
    p_bridge = p.ENVIRONMENT if process == p.BRIDGE else process
    bridge = Bridge.make("TestBridge", rate=1, is_reactive=is_reactive, real_time_factor=rtf, process=p_bridge)

    # Initialize Environment
    env = EagerEnv(name="rx", rate=1, graph=graph, bridge=bridge)

    # Evaluate performance
    def run_info(iter, iter_tot, start, end, start_rel, iter_rel):
        t = end - start
        t_rel = end - start_rel
        print(
            f"N={num_nodes}, R={is_reactive}, P={process} | iter={iter}/{iter_tot} | {iter/t: .2f} Hz | {(iter-iter_rel)/t_rel: .2f} Hz (last {t_rel: .2f} s)"
        )

    env.reset()
    action = env.action_space.sample()
    iter_tot = 20000
    start = perf_counter()
    start_rel = start
    rel = 2
    iter_rel = 0
    for j in range(1, iter_tot + 1):
        env.step(action)
        end = perf_counter()
        if (end - start_rel) > rel:
            run_info(j, iter_tot, start, end, start_rel, iter_rel)
            start_rel = end
            iter_rel = j
    end = perf_counter()
    run_info(iter_tot, iter_tot, start, end, start_rel, iter_rel)


if __name__ == "__main__":
    evaluation(10, is_reactive=False, rtf=2000, process=p.NEW_PROCESS)
