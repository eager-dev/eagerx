# ROS IMPORTS
from std_msgs.msg import UInt64, Bool, String

# RX IMPORTS
import rx
from rx import operators as ops
from rx.disposable import CompositeDisposable
from rx.scheduler import EventLoopScheduler, ThreadPoolScheduler
from rx.subject import ReplaySubject, Subject, BehaviorSubject

# EAGERX IMPORTS
from eagerx.core.constants import DEBUG

from eagerx.core.rx_operators import (
    cb_ft,
    spy,
    trace_observable,
    flag_dict,
    switch_to_reset,
    combine_dict,
    init_channels,
    init_real_reset,
    merge_dicts,
    init_state_inputs_channel,
    init_state_resets,
    init_callback_pipeline,
    get_object_params,
    extract_inputs_and_reactive_proxy,
    initialize_reactive_proxy_reset,
    switch_with_check_pipeline,
    node_reset_flags,
    filter_dict_on_key,
    get_node_params,
    extract_node_reset,
    throttle_callback_trigger,
    with_latest_from,
)


def init_node_pipeline(
    ns,
    rate_node,
    node,
    inputs,
    outputs,
    F,
    SS_ho,
    SS_CL_ho,
    R,
    RR,
    E,
    real_reset,
    feedthrough,
    state_inputs,
    state_outputs,
    targets,
    cb_ft,
    is_reactive,
    real_time_factor,
    simulate_delays,
    disposables,
    event_scheduler=None,
):
    # Node ticks
    Rn = ReplaySubject()  # Reset flag for the node (Nc=Ns and r_signal)
    Nc = Subject()  # Number completed callbacks (i.e. send Topics):
    Ns = BehaviorSubject(0)  # Number of started callbacks (i.e. number of planned Topics).

    # Throttle the callback trigger
    Nct = throttle_callback_trigger(rate_node, Nc, E, is_reactive, real_time_factor, event_scheduler, node)

    # Create input channels
    zipped_inputs, zipped_input_flags = init_channels(
        ns,
        Nct,
        rate_node,
        inputs,
        is_reactive,
        real_time_factor,
        simulate_delays,
        E,
        node=node,
        scheduler=event_scheduler,
    )

    # Create action channels
    zipped_feedthrough, zipped_action_flags, d_rr = init_real_reset(
        ns,
        Nct,
        rate_node,
        RR,
        real_reset,
        feedthrough,
        targets,
        is_reactive,
        real_time_factor,
        simulate_delays,
        E,
        event_scheduler,
        node=node,
    )

    # Zip inputs & action channels
    zipped_channels = rx.zip(zipped_inputs, zipped_feedthrough).pipe(
        ops.share(), ops.observe_on(event_scheduler)
    )  # this is required, otherwise a block.

    # New routine with merge
    PR = R.pipe(
        ops.observe_on(event_scheduler),
        ops.map(lambda x: True),
        ops.merge(zipped_channels),
        switch_to_reset(),
        ops.share(),
    )

    # Create reset signal
    Rr, P = PR.pipe(ops.partition(lambda value: isinstance(value, bool)))

    # Create accumulator: (acc)
    d_Ns = P.pipe(ops.scan(lambda acc, x: acc + 1, 0)).subscribe(Ns)

    # Create callback stream
    input_stream = Ns.pipe(ops.skip(1), ops.zip(P), ops.share())
    d_msg, output_stream = init_callback_pipeline(
        ns,
        node.callback_cb,
        cb_ft,
        input_stream,
        real_reset,
        targets,
        state_outputs,
        outputs,
        event_scheduler,
        node=node,
    )

    # Publish output msg as ROS topic and to subjects if single process
    for o in outputs:
        d = output_stream.pipe(
            ops.filter(lambda x: x is not None),
            ops.pluck(o["name"]),
            ops.filter(lambda x: x is not None),
            ops.map(o["converter"].convert),
            ops.share(),
        ).subscribe(o["msg"])
        # Add disposable
        d_msg += [d]

    # Publish output msg as ROS topic and to subjects if single process
    Nc_obs = output_stream.pipe(ops.scan(lambda acc, x: acc + 1, 0))
    Nc_empty = output_stream.pipe(ops.scan(lambda acc, x: acc + 1 if x is None else acc, 0), ops.start_with(0))

    # Increase ticks
    d_Nc = Nc_obs.subscribe(Nc, scheduler=event_scheduler)
    d_Rn = Nc_obs.pipe(
        ops.start_with(0),  # added to simulated first zero from BS(0) of Nc
        ops.zip(Nc_empty),
        ops.map(lambda x: x[0] - x[1]),
        ops.combine_latest(Ns, Rr, Nc_empty),
        ops.filter(lambda value: value[0] == value[1] - value[3]),
        ops.take(1),
        ops.merge(rx.never()),
        # spy('post-filter', node),
    ).subscribe(Rn)

    # Create reset flags for the set_states
    ss_flags = init_state_inputs_channel(ns, state_inputs, event_scheduler, node=node)
    SS_ho.on_next(ss_flags.pipe(ops.take(1), ops.start_with(None)))
    SS_CL_ho.on_next(ss_flags.pipe(ops.start_with(None)))

    # Combine flags and subscribe F to all the flags zipped together
    d_flag = (
        rx.zip(zipped_input_flags, zipped_action_flags)
        .pipe(ops.map(lambda x: merge_dicts(x[0], x[1])))
        .subscribe(F, scheduler=event_scheduler)
    )

    # Dispose
    disposables.clear()
    d = CompositeDisposable([Rn, Nc, Ns, d_Rn, d_Nc, d_Ns, d_flag] + d_msg + d_rr)
    disposables.add(d)
    return {"Rn": Rn, "dispose": d}


def init_node(
    ns,
    rate_node,
    node,
    inputs,
    outputs,
    feedthrough=tuple(),
    state_inputs=tuple(),
    targets=tuple(),
):
    # Initialize scheduler
    event_scheduler = EventLoopScheduler()
    eps_disp = CompositeDisposable()
    reset_disp = CompositeDisposable(eps_disp)

    # Gather reactive properties
    is_reactive = node.is_reactive
    real_time_factor = node.real_time_factor
    simulate_delays = node.simulate_delays

    # Track node I/O
    node_inputs = []
    node_outputs = []

    # Prepare reset topic
    R = Subject()
    reset_input = dict(name="reset", address=ns + "/reset", msg_type=UInt64, msg=R)
    node_inputs.append(reset_input)

    # Prepare real_reset topic
    RR = Subject()
    real_reset_input = dict(name="real_reset", address=ns + "/real_reset", msg_type=UInt64, msg=RR)
    node_inputs.append(real_reset_input)

    # End reset
    E = Subject()
    end_reset = dict(name="end_reset", address=ns + "/end_reset", msg_type=UInt64, msg=E)
    node_inputs.append(end_reset)

    # Prepare node reset topic
    node_reset = dict(name=node.ns_name, msg_type=Bool)
    node_reset["address"] = node.ns_name + "/end_reset"
    node_reset["msg"] = Subject()
    node_outputs.append(node_reset)

    # Real reset checks
    state_outputs = []
    real_reset = len(feedthrough) > 0
    assert not real_reset or (
        real_reset and len(targets) > 0
    ), "Cannot initialize real reset node (%s). If len(feedthroughs) is provided, then len(targets) > 0. must hold."

    # Prepare input topics
    flags = []
    for i in inputs:
        # Subscribe to input topic
        Ir = Subject()
        i["msg"] = Ir

        # Subscribe to input reset topic
        Is = Subject()
        i["reset"] = Is

        # Prepare initial flag
        flag = i["reset"].pipe(flag_dict(i["name"]), ops.first(), ops.merge(rx.never()))
        flags.append(flag)

    # Prepare output topics
    for i in outputs:
        # Prepare output topic
        i["msg"] = Subject()

        # Initialize reset topic
        i["reset"] = Subject()

    # Prepare action topics (used by RealResetNode)
    for i in feedthrough:
        # Subscribe to input topic
        Ar = Subject()
        i["msg"] = Ar

        # Subscribe to input reset topic
        As = Subject()
        i["reset"] = As

        # Prepare initial flag
        flag = i["reset"].pipe(flag_dict(i["address"]), ops.first(), ops.merge(rx.never()))
        flags.append(flag)

    # Prepare state topics
    for i in state_inputs:
        # Initialize desired state message
        S = Subject()
        i["msg"] = S

        D = Subject()
        i["done"] = D

    for i in targets:
        # Initialize target state message
        S = Subject()
        i["msg"] = S

        D = Subject()
        i["done"] = D

        # Initialize done flag for desired state
        done_output = dict(name=i["name"], address=i["address"] + "/done", msg_type=Bool, msg=D)
        state_outputs.append(done_output)

    # Prepare initial flags
    F_init = rx.zip(*flags).pipe(spy("zip", node), ops.map(lambda x: merge_dicts({}, x)))

    # Reset flags
    F = Subject()
    f = rx.merge(F, F_init)
    ss_flags = init_state_inputs_channel(ns, state_inputs, event_scheduler, node=node)
    check_SS_CL, SS_CL, SS_CL_ho = switch_with_check_pipeline(init_ho=ss_flags)
    latched_ss_flags = rx.combine_latest(SS_CL, ss_flags).pipe(ops.map(lambda x: x[1]), ops.take(1))
    check_SS, SS, SS_ho = switch_with_check_pipeline(init_ho=latched_ss_flags)

    # Node ticks
    Rn_ho = BehaviorSubject(BehaviorSubject((0, 0, True)))
    Rn = Rn_ho.pipe(ops.switch_latest(), ops.map(lambda x: x[0]))

    # Create reset switch_latest
    Rr = R.pipe(ops.map(lambda x: True))  # ops.observe_on(event_scheduler),  # seemed to make ROS variant crash

    # Prepare "ready-to-reset" signal (i.e. after node receives reset signal and stops sending any output msgs).
    RrRn = rx.zip(Rr.pipe(spy("Rr", node)), Rn.pipe(spy("Rn", node))).pipe(
        ops.map(lambda x: x[1]), spy("SEND_FLAGS", node), ops.share()
    )

    # Send output flags
    for i in outputs:
        d = RrRn.pipe(ops.map(lambda x: UInt64(data=x))).subscribe(i["reset"])
        reset_disp.add(d)

    # Reset node pipeline
    reset_trigger = rx.zip(RrRn, f.pipe(spy("F", node)), SS.pipe(spy("SS", node))).pipe(
        with_latest_from(SS_CL),
        ops.map(lambda x: x[0][:-1] + (x[1],)),
        spy("RENEW_PIPE", node),
        ops.map(lambda x: x[-1]),
        ops.share(),
    )  # x: SS_CL
    reset_obs = reset_trigger.pipe(
        ops.map(
            lambda x: init_node_pipeline(
                ns,
                rate_node,
                node,
                inputs,
                outputs,
                F,
                SS_ho,
                SS_CL_ho,
                R,
                RR,
                E,
                real_reset,
                feedthrough,
                state_inputs,
                state_outputs,
                targets,
                cb_ft,
                is_reactive,
                real_time_factor,
                simulate_delays,
                eps_disp,
                event_scheduler=event_scheduler,
            )
        ),
        trace_observable("init_node_pipeline", node),
        ops.share(),
    )
    d = reset_obs.pipe(ops.pluck("Rn")).subscribe(Rn_ho)
    reset_disp.add(d)

    # Dispose old pipeline, run reset callback
    reset_msg = reset_obs.pipe(
        ops.pluck("dispose"),
        ops.buffer_with_count(2, skip=1),
        ops.start_with(None),
        # zipped with Rn_ho so that Rn has switched before sending "reset topic"
        ops.zip(reset_trigger, Rn_ho.pipe(ops.skip(1)), check_SS, check_SS_CL),
        ops.map(lambda x: x[1]),  # x[1]=Nc
        ops.share(),
        spy("RESET", node, log_level=DEBUG),
        ops.map(lambda x: node.reset_cb(**x)),
        trace_observable("cb_reset", node),
        ops.share(),
    )

    # Send node reset message
    d = reset_msg.pipe(ops.map(lambda x: Bool(data=True))).subscribe(node_reset["msg"])
    reset_disp.add(d)

    rx_objects = dict(
        inputs=inputs,
        outputs=outputs,
        feedthrough=feedthrough,
        state_inputs=state_inputs,
        state_outputs=state_outputs,
        targets=targets,
        node_inputs=node_inputs,
        node_outputs=node_outputs,
        disposable=reset_disp,
    )
    return rx_objects


def init_bridge_pipeline(
    ns,
    rate_node,
    node,
    zipped_channels,
    outputs,
    Nct_ho,
    DF,
    RRn_ho,
    SS_ho,
    SS_CL_ho,
    state_inputs,
    is_reactive,
    real_time_factor,
    simulate_delays,
    E,
    disposables,
    event_scheduler=None,
):
    # Node ticks
    RRn = Subject()
    RRn_ho.on_next(RRn)
    Nc = Subject()  # Number completed callbacks (i.e. send Topics): initialized at zero to kick of chain reaction
    Ns = BehaviorSubject(0)  # Number of started callbacks (i.e. number of planned Topics).

    # Throttle the callback trigger
    Nct = throttle_callback_trigger(rate_node, Nc, E, is_reactive, real_time_factor, event_scheduler, node)
    Nct_ho.on_next(Nct)

    # Create a tuple with None, to be consistent with feedthrough pipeline of init_node_pipeline
    zipped_channels = zipped_channels.pipe(ops.map(lambda i: (i, None)))

    # New routine with merge
    PR = DF.pipe(
        ops.filter(lambda x: all([df for df in x])),
        spy("DF filtered", node),
        ops.observe_on(event_scheduler),
        ops.map(lambda x: True),
        ops.merge(zipped_channels),
        switch_to_reset(),
        ops.share(),
    )

    # Create reset signal
    RRr, P = PR.pipe(ops.partition(lambda value: isinstance(value, bool)))

    # Create accumulator: (acc)
    d_Ns = P.pipe(ops.scan(lambda acc, x: acc + 1, 0)).subscribe(Ns)

    # Create callback stream
    input_stream = Ns.pipe(ops.skip(1), ops.zip(P), ops.share())
    d_msg, output_stream = init_callback_pipeline(
        ns,
        node.callback_cb,
        cb_ft,
        input_stream,
        False,
        tuple(),
        tuple(),
        outputs,
        event_scheduler,
        node,
    )

    # Publish output msg as ROS topic and to subjects if single process
    for o in outputs:
        d = output_stream.pipe(
            ops.pluck(o["name"]),
            ops.filter(lambda x: x is not None),
            ops.map(o["converter"].convert),
            ops.share(),
        ).subscribe(o["msg"])
        # Add disposable
        d_msg += [d]

    # After outputs have been send, increase the completed callback counter
    Nc_obs = output_stream.pipe(ops.scan(lambda acc, x: acc + 1, 0))

    # Increase ticks
    d_Nc = Nc_obs.subscribe(Nc, scheduler=event_scheduler)
    d_RRn = Nc_obs.pipe(
        ops.start_with(0),  # added to simulated first zero from BS(0) of Nc
        ops.combine_latest(Ns, RRr),
        ops.filter(lambda value: value[0] == value[1]),
        ops.take(1),
        ops.merge(rx.never()),
    ).subscribe(RRn)

    # Create reset flags for the set_states
    ss_flags = init_state_inputs_channel(ns, state_inputs, event_scheduler, node=node)
    SS_ho.on_next(ss_flags.pipe(ops.take(1), ops.start_with(None)))
    SS_CL_ho.on_next(ss_flags.pipe(ops.start_with(None)))

    # Dispose
    disposables.clear()
    d = CompositeDisposable([RRn, Nc, Ns, d_RRn, d_Nc, d_Ns] + d_msg)
    disposables.add(d)
    return {"dispose": d}


def init_bridge(
    ns,
    rate_node,
    node,
    inputs_init,
    outputs,
    state_inputs,
    node_names,
    target_addresses,
    message_broker,
):
    ###########################################################################
    # Initialization ##########################################################
    ###########################################################################
    # Prepare scheduler
    event_scheduler = EventLoopScheduler()
    eps_disp = CompositeDisposable()
    reset_disp = CompositeDisposable(eps_disp)

    # Gather reactive properties
    is_reactive = node.is_reactive
    real_time_factor = node.real_time_factor
    simulate_delays = node.simulate_delays

    # Prepare output topics
    for i in outputs:
        # Prepare output topic
        i["msg"] = Subject()

        # Initialize reset topic
        i["reset"] = Subject()

    # Prepare input topics
    for i in inputs_init:
        # Subscribe to input topic
        Ir = Subject()
        i["msg"] = Ir

        # Subscribe to input reset topic
        Is = Subject()
        i["reset"] = Is

    # Prepare state topics
    for i in state_inputs:
        # Initialize desired state message
        S = Subject()
        i["msg"] = S

        D = Subject()
        i["done"] = D

    ss_flags = init_state_inputs_channel(ns, state_inputs, event_scheduler, node=node)
    check_SS_CL, SS_CL, SS_CL_ho = switch_with_check_pipeline(init_ho=ss_flags)
    latched_ss_flags = rx.combine_latest(SS_CL, ss_flags).pipe(ops.map(lambda x: x[1]), ops.take(1))
    check_SS, SS, SS_ho = switch_with_check_pipeline(init_ho=latched_ss_flags)

    # Prepare target_addresses
    df_inputs = []
    dfs = []
    for i in target_addresses:
        address = "%s/%s" % (ns, i)
        done = Subject()
        df_inputs.append(dict(address=address, done=done))
        dfs.append(done.pipe(ops.map(lambda msg: bool(msg.data))))

    # Track node I/O
    node_inputs = []
    node_outputs = []

    ###########################################################################
    # Registry: node flags ####################################################
    ###########################################################################
    init_node_flags = []
    for i in node_names:
        nf = dict(name=i, address="%s/%s/end_reset" % (ns, i), msg=Subject(), msg_type=Bool)
        node_inputs.append(nf)
        init_node_flags.append(nf)

    ###########################################################################
    # Registry: real reset states #############################################
    ###########################################################################
    # Resettable real states (to dynamically add state/done flags to DF for RR)
    RR = Subject()
    RRr = RR.pipe(ops.map(lambda x: True))

    # Switch to latest state reset done flags
    DF = rx.combine_latest(RRr, *dfs)

    ###########################################################################
    # Registry ################################################################
    ###########################################################################
    # Object register (to dynamically add input reset flags to F for reset)
    NR = Subject()
    node_registry = dict(name="register_node", address=ns + "/register_node", msg=NR, msg_type=String)
    node_inputs.append(node_registry)

    # Node registry pipeline
    node_params = NR.pipe(
        spy("nodes", node, log_level=DEBUG),
        ops.map(get_node_params),
        ops.filter(lambda params: params is not None),
        ops.map(node.register_node),
        ops.map(lambda args: extract_node_reset(ns, *args)),
        ops.share(),
    )

    # Object register (to dynamically add input reset flags to F for reset)
    OR = Subject()
    object_registry = dict(name="register_object", address=ns + "/register_object", msg=OR, msg_type=String)
    node_inputs.append(object_registry)

    # Object registry pipeline
    object_params = OR.pipe(
        ops.map(get_object_params),
        ops.filter(lambda params: params is not None),
        ops.map(lambda params: (params[1],) + node.register_object(*params)),
        ops.map(lambda i: extract_inputs_and_reactive_proxy(ns, *i)),
        ops.share(),
    )

    rx_objects = rx.merge(object_params, node_params).pipe(
        ops.map(
            lambda i: (
                i,
                message_broker.add_rx_objects(
                    node.ns_name,
                    inputs=i["inputs"],
                    reactive_proxy=i["reactive_proxy"],
                    state_inputs=i["state_inputs"],
                    node_inputs=i["node_flags"],
                ),
            )
        ),
        ops.map(lambda i: i[0]),
        ops.scan(
            combine_dict,
            dict(
                inputs=inputs_init,
                reactive_proxy=[],
                sp_nodes=[],
                launch_nodes=[],
                state_inputs=[],
                node_flags=init_node_flags,
            ),
        ),
        ops.share(),
    )

    # Make sure that all nodes are initialized, before passing it to start_reset_input
    REG_cum = rx.merge(OR, NR).pipe(ops.scan(lambda acc, x: acc + 1, 0))
    rx_objects = rx.zip(rx_objects, REG_cum)

    ###########################################################################
    # Start reset #############################################################
    ###########################################################################
    # Prepare start_reset input
    SR = Subject()
    start_reset_input = dict(name="start_reset", address=ns + "/start_reset", msg=SR, msg_type=UInt64)
    node_inputs.append(start_reset_input)

    # Latch on '/rx/start_reset' event
    rx_objects = SR.pipe(
        ops.combine_latest(REG_cum),
        ops.filter(lambda x: x[0].data == x[1]),  # cum_registered == REG_cum
        ops.combine_latest(rx_objects),
        spy("SR", node, log_level=DEBUG, mapper=lambda x: (x[0][0], x[0][1], x[1][1])),
        ops.filter(lambda x: x[0][1] == x[1][1]),  # cum_registered == REG_cum && REG_cum == rx_objects[1]
        ops.map(lambda i: i[1][0]),  # rx_objects
        ops.share(),
    )
    inputs = rx_objects.pipe(ops.pluck("inputs"))
    simstate_inputs = rx_objects.pipe(ops.pluck("state_inputs"))
    reactive_proxy = rx_objects.pipe(ops.pluck("reactive_proxy"))
    node_flags = rx_objects.pipe(ops.pluck("node_flags"))

    # Prepare output for reactive proxy
    RM = Subject()
    check_reactive_proxy = reactive_proxy.pipe(ops.map(lambda rx: initialize_reactive_proxy_reset(rate_node, RM, rx, node)))

    # Zip initial input flags
    check_F_init, F_init, F_init_ho = switch_with_check_pipeline()
    F_init = F_init.pipe(ops.first(), ops.merge(rx.never()))
    d = inputs.pipe(
        ops.map(
            lambda inputs: rx.zip(*[i["reset"].pipe(flag_dict(i["name"])) for i in inputs]).pipe(
                ops.map(lambda x: merge_dicts({}, x)), ops.start_with(None)
            )
        )
    ).subscribe(F_init_ho)
    reset_disp.add(d)
    F = Subject()
    f = rx.merge(F, F_init)

    # Zip node flags
    check_NF, NF, NF_ho = switch_with_check_pipeline()
    d = node_flags.pipe(ops.map(lambda node_flags: node_reset_flags(ns, node_flags, node))).subscribe(NF_ho)
    reset_disp.add(d)

    # Dynamically initialize new state pipeline
    ResetTrigger = Subject()
    ss_flags = simstate_inputs.pipe(
        ops.map(lambda s: init_state_resets(ns, s, ResetTrigger, event_scheduler, node)),
        ops.share(),
    )
    check_simSS, simSS, simSS_ho = switch_with_check_pipeline()
    d = ss_flags.pipe(ops.map(lambda obs: obs.pipe(ops.start_with(None)))).subscribe(simSS_ho)
    reset_disp.add(d)

    # Before starting real_reset procedure, wait for EngineState pipeline to be initialized.
    # This, so that the first time, the engine states are run.
    # Some bridges/simulators might require that for setting initial state.
    ER = Subject()
    end_register = dict(name="end_register", address=ns + "/end_register", msg=ER, msg_type=UInt64)
    node_outputs.append(end_register)

    # Zip switch checks to indicate end of '/rx/start_reset' procedure, and start of '/rx/real_reset'
    d = (
        rx.zip(check_F_init, check_reactive_proxy, check_NF, check_simSS)
        .pipe(ops.map(lambda i: message_broker.connect_io()), ops.map(lambda i: UInt64()))
        .subscribe(ER)
    )
    reset_disp.add(d)

    ###########################################################################
    # Real reset ##############################################################
    ###########################################################################
    # Prepare real_reset output
    real_reset_input = dict(name="real_reset", address=ns + "/real_reset", msg=RR, msg_type=UInt64)
    node_inputs.append(real_reset_input)

    # Zip switch checks to indicate end of '/rx/start_reset' procedure, and start of '/rx/real_reset'
    d = (
        rx.zip(check_F_init, check_reactive_proxy, check_NF)
        .pipe(
            ops.map(lambda i: message_broker.connect_io()),
        )
        .subscribe(RR)
    )
    reset_disp.add(d)

    # Real reset routine. Cuts-off tick_callback when RRr is received, instead of Rr
    check_RRn, RRn, RRn_ho = switch_with_check_pipeline(init_ho=BehaviorSubject((0, 0, True)))
    pre_reset_trigger = rx.zip(RRn.pipe(spy("RRn", node)), RRr.pipe(spy("RRr", node)), SS.pipe(spy("SS", node))).pipe(
        with_latest_from(SS_CL),
        ops.map(lambda x: x[0][:-1] + (x[1],)),
        ops.map(lambda x: (x, node.pre_reset_cb(**x[-1]))),
        # Run pre-reset callback
        spy("PRE-RESET", node, log_level=DEBUG),
        trace_observable("cb_pre_reset", node),
        ops.share(),
    )

    d = pre_reset_trigger.pipe(
        ops.map(lambda x: x[0][0]),
        ops.map(lambda x: UInt64(data=x[0] + 1)),
        ops.share(),  # x[0][0]=Nc
    ).subscribe(RM)
    reset_disp.add(d)
    ss_cl = pre_reset_trigger.pipe(ops.map(lambda x: x[0][-1]))  # x[0][-1]=ss_cl

    ###########################################################################
    # Reset ###################################################################
    ###########################################################################
    # Prepare reset output
    R = Subject()
    reset_output = dict(name="reset", address=ns + "/reset", msg=R, msg_type=UInt64)
    node_outputs.append(reset_output)

    # Send reset message
    d = RM.subscribe(R)
    reset_disp.add(d)
    Rr = R.pipe(ops.map(lambda x: True))
    d = rx.zip(f.pipe(spy("F", node)), Rr.pipe(spy("Rr", node))).pipe(ops.share()).subscribe(ResetTrigger)
    reset_disp.add(d)

    # Send reset messages for all outputs (Only '/rx/bridge/outputs/tick')
    [reset_disp.add(RM.subscribe(o["reset"])) for o in outputs]

    ###########################################################################
    # Reset: initialize episode pipeline ######################################
    ###########################################################################
    # Prepare end_reset output
    end_reset = dict(name="end_reset", address=ns + "/end_reset", msg=Subject(), msg_type=UInt64)
    node_outputs.append(end_reset)

    # Dynamically initialize new input pipeline
    check_Nct, Nct, Nct_ho = switch_with_check_pipeline()
    inputs_flags = inputs.pipe(
        ops.zip(ResetTrigger),
        ops.map(lambda i: i[0]),
        ops.map(
            lambda inputs: init_channels(
                ns,
                Nct,
                rate_node,
                inputs,
                is_reactive,
                real_time_factor,
                simulate_delays,
                end_reset["msg"],
                event_scheduler,
                node,
            )
        ),
        ops.share(),
    )

    # Switch to latest zipped inputs pipeline
    check_z_inputs, z_inputs, z_inputs_ho = switch_with_check_pipeline()
    d = inputs_flags.pipe(ops.map(lambda i: i[0].pipe(ops.start_with(None)))).subscribe(z_inputs_ho, scheduler=event_scheduler)
    reset_disp.add(d)

    # Switch to latest zipped flags pipeline
    check_z_flags, z_flags, z_flags_ho = switch_with_check_pipeline()
    d = inputs_flags.pipe(ops.map(lambda i: i[1].pipe(ops.start_with(None)))).subscribe(z_flags_ho, scheduler=event_scheduler)
    reset_disp.add(d)
    d = z_flags.subscribe(F)
    reset_disp.add(d)

    # Initialize rest of episode pipeline
    pipeline_trigger = rx.zip(check_z_flags, check_z_inputs)
    reset_obs = pipeline_trigger.pipe(
        ops.map(
            lambda x: init_bridge_pipeline(
                ns,
                rate_node,
                node,
                z_inputs,
                outputs,
                Nct_ho,
                DF,
                RRn_ho,
                SS_ho,
                SS_CL_ho,
                state_inputs,
                is_reactive,
                real_time_factor,
                simulate_delays,
                end_reset["msg"],
                eps_disp,
                event_scheduler=event_scheduler,
            )
        ),
        trace_observable("init_bridge_pipeline", node),
        ops.share(),
    )

    ###########################################################################
    # End reset ###############################################################
    ###########################################################################
    # Send '/end_reset' after reset has finished
    d = reset_obs.pipe(
        ops.pluck("dispose"),
        ops.buffer_with_count(2, skip=1),
        # ops.map(lambda x: x[0].dispose()),
        ops.start_with(None),
        ops.zip(
            ss_cl,
            reset_obs,
            simSS,
            NF.pipe(spy("NF", node)),
            check_SS,
            check_SS_CL,
        ),
        ops.map(lambda x: node.reset_cb(**x[1])),
        spy("POST-RESET", node, log_level=DEBUG),
        trace_observable("cb_post_reset", node),
        ops.map(lambda x: UInt64(data=0)),
        ops.share(),
    ).subscribe(end_reset["msg"])
    reset_disp.add(d)

    rx_objects = dict(
        inputs=inputs_init,
        outputs=outputs,
        node_inputs=node_inputs,
        node_outputs=node_outputs,
        state_inputs=list(state_inputs) + df_inputs,
        disposable=reset_disp,
    )
    return rx_objects


def init_supervisor(ns, node, outputs=tuple(), state_outputs=tuple()):
    # Initialize schedulers
    tp_scheduler = ThreadPoolScheduler(max_workers=5)
    reset_disp = CompositeDisposable()

    # Prepare states
    done_outputs = []
    for s in state_outputs:
        # Prepare done flag
        s["done"] = Subject()
        done_outputs.append(
            dict(
                name=s["name"],
                address=s["address"] + "/done",
                msg_type=Bool,
                msg=s["done"],
            )
        )

        # Prepare state message (IMPORTANT: after done flag, we modify address afterwards)
        s["msg"] = Subject()
        s["address"] += "/set"

    ###########################################################################
    # Start reset #############################################################
    ###########################################################################
    SR = Subject()  # ---> Not a node output, but used in node.reset() to kickstart reset pipeline (send self.cum_registered).
    start_reset = dict(name="start_reset", address=ns + "/start_reset", msg=Subject(), msg_type=UInt64)
    d = SR.subscribe(start_reset["msg"], scheduler=tp_scheduler)
    reset_disp.add(d)

    ###########################################################################
    # End register ############################################################
    ###########################################################################
    ER = Subject()
    end_register = dict(name="reset", address=ns + "/end_register", msg_type=UInt64, msg=ER)
    real_reset = dict(name="real_reset", address=ns + "/real_reset", msg=Subject(), msg_type=UInt64)
    d = ER.subscribe(real_reset["msg"])
    reset_disp.add(d)

    # Publish state msgs
    # msgs = SR.pipe(ops.skip(1), ops.map(node._get_states), ops.share())
    msgs = ER.pipe(ops.map(node._get_states), ops.share())
    for s in state_outputs:
        d = msgs.pipe(
            ops.pluck(s["name"] + "/done"),
            trace_observable("done", node),
            ops.share(),
        ).subscribe(s["done"])
        reset_disp.add(d)

        d = msgs.pipe(
            filter_dict_on_key(s["name"]),
            ops.filter(lambda msg: msg is not None),
            ops.map(s["converter"].convert),
            ops.share(),
        ).subscribe(s["msg"])
        reset_disp.add(d)

    ###########################################################################
    # Reset ###################################################################
    ###########################################################################
    R = Subject()
    reset = dict(name="reset", address=ns + "/reset", msg_type=UInt64, msg=R)

    # Prepare node reset topic
    node_reset = dict(
        name=node.ns_name,
        address=node.ns_name + "/end_reset",
        msg_type=Bool,
        msg=Subject(),
    )

    # Reset pipeline
    d = R.pipe(ops.map(lambda x: Bool(data=True))).subscribe(node_reset["msg"], scheduler=tp_scheduler)
    reset_disp.add(d)

    ###########################################################################
    # Register ################################################################
    ###########################################################################
    REG_OBJECT = Subject()  # ---> Not a node output, but used in node.register_object() to kickstart register pipeline.
    register_object = dict(
        name="register_object",
        address=ns + "/register_object",
        msg=Subject(),
        msg_type=String,
    )
    REG_NODE = Subject()  # ---> Not a node output, but used in node.register_node() to kickstart register pipeline.
    register_node = dict(
        name="register_node",
        address=ns + "/register_node",
        msg=Subject(),
        msg_type=String,
    )

    # Register pipeline
    d = REG_OBJECT.subscribe(register_object["msg"], scheduler=tp_scheduler)
    reset_disp.add(d)
    d = REG_NODE.subscribe(register_node["msg"], scheduler=tp_scheduler)
    reset_disp.add(d)

    ###########################################################################
    # End reset ###############################################################
    ###########################################################################
    tick = dict(name="tick", address=ns + "/bridge/outputs/tick", msg=Subject(), msg_type=UInt64)
    end_reset = dict(name="end_reset", address=ns + "/end_reset", msg=Subject(), msg_type=UInt64)
    d = end_reset["msg"].pipe(spy("RESET END", node, log_level=DEBUG)).subscribe(tick["msg"])
    reset_disp.add(d)

    # Create node inputs & outputs
    node_inputs = [reset, end_reset, end_register]
    node_outputs = [register_object, register_node, start_reset, tick, node_reset, real_reset]
    outputs = []

    # Create return objects
    env_subjects = dict(register_object=REG_OBJECT, register_node=REG_NODE, start_reset=SR)
    rx_objects = dict(
        node_inputs=node_inputs,
        node_outputs=node_outputs,
        outputs=outputs,
        state_outputs=state_outputs + tuple(done_outputs),
        disposable=reset_disp,
    )
    return rx_objects, env_subjects
