# ROS IMPORTS
import rospy
from std_msgs.msg import Bool, UInt64

# RX IMPORTS
import rx
from rx import Observable, typing, operators as ops
from rx.disposable import Disposable, SingleAssignmentDisposable, CompositeDisposable
from rx.subject import Subject, BehaviorSubject
from rx.internal.concurrency import synchronized

# EAGERX IMPORTS
from eagerx.core.converters import Identity
from eagerx.core.specs import RxInput
from eagerx.core.constants import (
    SILENT,
    DEBUG,
    INFO,
    ERROR,
    WARN,
    FATAL,
    TERMCOLOR,
    ROS,
)
from eagerx.utils.utils import (
    get_attribute_from_module,
    initialize_converter,
    get_param_with_blocking,
    Info,
    Msg,
    Stamp,
    get_opposite_msg_cls,
    get_module_type_string,
    msg_type_error,
)

# OTHER IMPORTS
import time
from collections import deque
from termcolor import cprint
import datetime
import logging
import traceback
from os import getpid
from threading import current_thread, RLock
from typing import Callable, Optional, List, Any

print_modes = {TERMCOLOR: "eagerx_core.TERMCOLOR", ROS: "eagerx_core.ROS"}
ros_log_fns = {
    SILENT: lambda print_str: None,
    DEBUG: rospy.logdebug,
    INFO: rospy.loginfo,
    WARN: rospy.logwarn,
    ERROR: rospy.logerr,
    FATAL: rospy.logfatal,
}


def cb_ft(cb_input, is_reactive):
    # Fill output msg with number of node ticks
    output_msgs = dict()
    for key, msg in cb_input.items():
        if key not in ["node_tick", "t_n"]:
            if len(msg.msgs) > 0:
                output_msgs[key] = msg.msgs[-1]
            else:
                assert not is_reactive, "Actions must always be fed through if we are running reactively."
                output_msgs[key] = None
    return output_msgs


def print_info(
    node_name,
    color,
    id=None,
    trace_type=None,
    value=None,
    date=None,
    print_mode=TERMCOLOR,
    log_level=DEBUG,
):
    if print_mode == TERMCOLOR:
        if date:
            cprint("[" + str(date)[:40].ljust(20) + "]", color, end="")
        cprint("[" + str(getpid())[:5].ljust(5) + "]", color, end="")
        cprint(
            "[" + current_thread().name.split("/")[-1][:15].ljust(15) + "]",
            color,
            end="",
        )
        cprint(
            "[" + node_name.split("/")[-1][:12].ljust(12) + "]",
            color,
            end="",
            attrs=["bold"],
        )
        if id:
            cprint("[" + id.split("/")[-1][:12].ljust(12) + "]", color, end="")
        cprint((" %s: %s" % (trace_type, value)), color)
    elif print_mode == ROS:
        print_str = ""
        print_str += "[" + str(getpid())[:5].ljust(5) + "]"
        print_str += "[" + current_thread().name.split("/")[-1][:15].ljust(15) + "]"
        print_str += "[" + node_name.split("/")[-1][:12].ljust(12) + "]"
        if id:
            print_str += "[" + id.split("/")[-1][:12].ljust(12) + "]"
        print_str += " %s: %s" % (trace_type, value)
        ros_log_fns[log_level](print_str)
    else:
        raise ValueError("Print mode not recognized. Only print_modes %s are available." % (print_modes.values()))


def spy(id: str, node, log_level: int = DEBUG, mapper: Callable = lambda msg: msg):
    node_name = node.ns_name
    color = node.color
    print_mode = node.print_mode
    effective_log_level = logging.getLogger("rosout").getEffectiveLevel()

    def _spy(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):

                if node.log_level >= effective_log_level and log_level >= effective_log_level:
                    print_info(
                        node_name,
                        color,
                        id,
                        trace_type="",
                        value=str(mapper(value)),
                        print_mode=print_mode,
                        log_level=log_level,
                    )
                observer.on_next(value)

            return source.subscribe(on_next, observer.on_error, observer.on_completed, scheduler)

        return rx.create(subscribe)

    return _spy


def trace_observable(
    id: str,
    node,
    trace_next=False,
    trace_next_payload=False,
    trace_subscribe=False,
    date=None,
):
    node_name = node.ns_name
    color = node.color
    print_mode = node.print_mode

    def _trace(source):
        def on_subscribe(observer, scheduler):
            def on_next(value):
                if trace_next is True:
                    if trace_next_payload is True:
                        print_info(
                            node_name,
                            color,
                            id,
                            "on_next",
                            value,
                            date=date or datetime.datetime.now(),
                            print_mode=print_mode,
                            log_level=DEBUG,
                        )
                    else:
                        print_info(
                            node_name,
                            color,
                            id,
                            "on_next",
                            "",
                            date=date or datetime.datetime.now(),
                            print_mode=print_mode,
                            log_level=DEBUG,
                        )
                observer.on_next(value)

            def on_completed():
                value = ""
                print_info(
                    node_name,
                    color,
                    id,
                    "on_completed",
                    value,
                    date=date or datetime.datetime.now(),
                    print_mode=print_mode,
                    log_level=DEBUG,
                )
                observer.on_completed()

            def on_error(error):
                if isinstance(error, Exception):
                    error_traceback = "%s, %s" % (
                        error,
                        traceback.print_tb(error.__traceback__),
                    )
                    print_info(
                        node_name,
                        color,
                        id,
                        "on_error",
                        error_traceback,
                        date=date or datetime.datetime.now(),
                    )
                else:
                    print_info(
                        node_name,
                        color,
                        id,
                        "on_error",
                        error,
                        date=date or datetime.datetime.now(),
                        print_mode=print_mode,
                        log_level=rospy.ERROR,
                    )
                observer.on_error(error)

            def dispose():
                if trace_subscribe is True:
                    value = ""
                    print_info(
                        node_name,
                        color,
                        id,
                        "dispose",
                        value,
                        date=date or datetime.datetime.now(),
                        print_mode=print_mode,
                        log_level=DEBUG,
                    )
                disposable.dispose()

            if trace_subscribe is True:
                value = ""
                print_info(
                    node_name,
                    color,
                    id,
                    "on_subscribe",
                    value,
                    date=date or datetime.datetime.now(),
                    print_mode=print_mode,
                    log_level=DEBUG,
                )
            disposable = source.subscribe(
                on_next=on_next,
                on_error=on_error,
                on_completed=on_completed,
            )
            return Disposable(dispose)

        return rx.create(on_subscribe)

    return _trace


def flag_dict(name):
    def _init_flag_dict(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):
                flag_dict = {name: value.data}
                observer.on_next(flag_dict)

            return source.subscribe(on_next, observer.on_error, observer.on_completed, scheduler)

        return rx.create(subscribe)

    return _init_flag_dict


def filter_dict():
    def _filter_dict(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):
                d = dict()
                for node_name, flag in value.items():
                    if flag is True:
                        continue
                    d[node_name] = flag
                observer.on_next(d)

            return source.subscribe(on_next, observer.on_error, observer.on_completed, scheduler)

        return rx.create(subscribe)

    return _filter_dict


def switch_to_reset():
    def _switch_to_reset(source):
        def subscribe(observer, scheduler=None):
            reset_mode = [False]

            def on_next(value):
                if isinstance(value, bool):
                    reset_mode[0] = True  # if we receive a reset flag, turn on reset mode
                    observer.on_next(value)
                else:
                    if not reset_mode[0]:  # Check if we haven't previously received a reset message
                        observer.on_next(value)

            return source.subscribe(on_next, observer.on_error, observer.on_completed, scheduler)

        return rx.create(subscribe)

    return _switch_to_reset


def combine_dict(acc, x):
    for key, item in acc.items():
        item += x[key]
    return acc


def create_msg_tuple(name: str, node_tick: int, msg: List[Any], stamp: List[Stamp], done: bool = None):
    info = Info(name=name, node_tick=node_tick, t_in=stamp, done=done)
    return Msg(info, msg)


def remap_state(name, is_reactive, real_time_factor):
    def _remap_state(source):
        def subscribe(observer, scheduler=None):
            start = time.time()
            seq = [0]

            def on_next(value):
                node_tick = value[0][0]
                msg = value[0][1]
                done = value[1]
                wc_stamp = time.time()
                if is_reactive:
                    sim_stamp = None
                else:
                    sim_stamp = (wc_stamp - start) / real_time_factor
                stamp = Stamp(seq[0], sim_stamp, wc_stamp)
                res = create_msg_tuple(name, node_tick, [msg], [stamp], done=done)
                seq[0] += 1
                observer.on_next(res)

            return source.subscribe(on_next, observer.on_error, observer.on_completed, scheduler)

        return rx.create(subscribe)

    return _remap_state


def remap_target(name, is_reactive, real_time_factor):
    def _remap_target(source):
        def subscribe(observer, scheduler=None):
            start = time.time()
            seq = [0]

            def on_next(value):
                node_tick = value[0]
                msg = value[1]
                wc_stamp = time.time()
                if is_reactive:
                    sim_stamp = None
                else:
                    sim_stamp = (wc_stamp - start) / real_time_factor
                stamp = Stamp(seq[0], sim_stamp, wc_stamp)
                res = create_msg_tuple(name, node_tick, [msg], [stamp])
                seq[0] += 1
                observer.on_next(res)

            return source.subscribe(on_next, observer.on_error, observer.on_completed, scheduler)

        return rx.create(subscribe)

    return _remap_target


def filter_info_for_printing(info):
    info_dict = dict()
    if info.rate_in:
        info_dict["rate_in"] = info.rate_in
    info_dict["t_in"] = [t.sim_stamp for t in info.t_in]
    if info.t_node:
        info_dict["t_node"] = [t.sim_stamp for t in info.t_node]
    if info.done:
        info_dict["done"] = info.done
    return info_dict


def remap_cb_input(mode=0):
    def _remap_cb_input(value):
        # mode=0 (info only), mode=1 (msgs only), mode=2 (all)
        if mode == 2:
            return value
        if isinstance(value, tuple):
            mapped_value = tuple([value[0].copy(), value[1].copy()])
            for i in mapped_value:
                for key, msg in i.items():
                    if key not in ["node_tick", "t_n"]:
                        if mode == 0:
                            i[key] = filter_info_for_printing(msg.info)
                        else:
                            i[key] = msg.msgs
        else:
            mapped_value = value.copy()
            for key, msg in mapped_value.items():
                if key not in ["node_tick", "t_n"]:
                    if mode == 0:
                        mapped_value[key] = filter_info_for_printing(msg.info)
                    else:
                        mapped_value[key] = msg.msgs
        return mapped_value

    return _remap_cb_input


def regroup_inputs(node, rate_node=1, is_input=True, perform_checks=True):
    node_name = node.ns_name
    color = node.color
    print_mode = node.print_mode

    def _regroup_inputs(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):
                # Regroups all inputs into a single dict
                if is_input:
                    node_tick = value[0].info.node_tick
                    t_n = round(node_tick / rate_node, 12)
                    res = dict(node_tick=node_tick, t_n=t_n)
                else:
                    res = dict()
                for msg in value:
                    res[msg.info.name] = msg

                # Perform checks
                if perform_checks and is_input:
                    node_ticks = []
                    for msg in value:
                        node_ticks.append(msg.info.node_tick)
                    if len(node_ticks) > 0:
                        if not len(set(node_ticks)) == 1:
                            print_info(
                                node_name,
                                color,
                                "regroup_inputs",
                                trace_type="",
                                value="Not all node_ticks are the same: %s" % str(value),
                                print_mode=print_mode,
                                log_level=rospy.ERROR,
                            )

                # Send regrouped input
                observer.on_next(res)

            return source.subscribe(on_next, observer.on_error, observer.on_completed, scheduler)

        return rx.create(subscribe)

    return _regroup_inputs


def expected_inputs(idx_n, rate_in, rate_node, delay):
    if idx_n < 0:
        return 0
    elif idx_n == 0:
        return 1
    else:
        # N = idx_n + 1, because idx_n starts at 0
        N_t_min_1 = idx_n
        N_t = idx_n + 1
        # Note: idx_n=1 after initial tick, corresponding to T=0
        # Hence, T_t=dt_n * (idx_n-1), T_t+1=dt_n * idx_n
        sum_t_min_1 = calculate_inputs(N_t_min_1, rate_in, rate_node, delay)
        sum_t = calculate_inputs(N_t, rate_in, rate_node, delay)
        return sum_t - sum_t_min_1


def calculate_inputs(N_node, rate_in, rate_node, delay):
    N_in = max(0, int((rate_in * (N_node - 1) - rate_node * rate_in * delay) // rate_node))  # Current timestep
    # N_in = max(0, int(((rate_in * (N_node - 1) - rate_node * rate_in * delay) / rate_node)))  # Current timestep
    # N_in = max(0, int(round(((dt_n * (N_node - 1) - delay) / dt_i), 11)))  # Current timestep
    # N_in = max(0, int(((dt_n * (N_node - 1) - delay) / dt_i)))  # Current timestep
    return N_in


def generate_msgs(
    source_Nc: Observable,
    rate_node: float,
    name: str,
    rate_in: float,
    params: dict,
    is_reactive: bool,
    real_time_factor: float,
    simulate_delays: bool,
    node=None,
):
    dt_i = 1 / rate_in

    def _generate_msgs(source_msg: Observable):
        window = params["window"]
        skip = int(params["skip"])

        def subscribe(observer: typing.Observer, scheduler: Optional[typing.Scheduler] = None) -> CompositeDisposable:
            start = time.time()
            msgs_queue: List = []
            t_i_queue: List = []
            num_queue: List = []
            tick_queue: List = []
            msgs_window = deque(maxlen=window)
            t_i_window = deque(maxlen=window)
            t_n_window = deque(maxlen=window)
            lock = RLock()

            @synchronized(lock)
            def next(i):
                if len(tick_queue) > 0:
                    if not is_reactive or len(msgs_queue) >= num_queue[0]:
                        try:
                            tick = tick_queue.pop(0)
                            if is_reactive:
                                # determine num_msgs
                                num_msgs = num_queue.pop(0)
                                msgs = msgs_queue[:num_msgs]
                                t_i = t_i_queue[:num_msgs]
                                msgs_queue[:] = msgs_queue[num_msgs:]
                                t_i_queue[:] = t_i_queue[num_msgs:]
                            else:  # Empty complete buffer
                                msgs = msgs_queue.copy()
                                t_i = t_i_queue.copy()
                                msgs_queue[:] = []
                                t_i_queue[:] = []
                        except Exception as ex:  # pylint: disable=broad-except
                            observer.on_error(ex)
                            return

                        # Determine t_n stamp
                        wc_stamp = time.time()
                        seq = tick
                        if is_reactive:
                            sim_stamp = round(tick / rate_node, 12)
                        else:
                            sim_stamp = (wc_stamp - start) / real_time_factor
                        t_n = Stamp(seq, sim_stamp, wc_stamp)

                        if window > 0:
                            msgs_window.extend(msgs)
                            t_i_window.extend(t_i)
                            t_n_window.extend([t_n] * len(msgs))
                            wmsgs = list(msgs_window)
                            wt_i = list(t_i_window)
                            wt_n = list(t_n_window)
                        else:
                            wmsgs = msgs
                            wt_i = t_i
                            wt_n = [t_n] * len(msgs)
                        res = Msg(Info(name, tick, rate_in, wt_n, wt_i, None), wmsgs)
                        observer.on_next(res)

            # Determine Nc logic
            def on_next_Nc(x):
                if is_reactive:
                    # Caculate expected number of message to be received
                    delay = params["delay"] if simulate_delays else 0.0
                    num_msgs = expected_inputs(x - skip, rate_in, rate_node, delay)
                    num_queue.append(num_msgs)
                tick_queue.append(x)
                next(x)

            subscriptions = []
            sad = SingleAssignmentDisposable()
            sad.disposable = source_Nc.subscribe(on_next_Nc, observer.on_error, observer.on_completed, scheduler)
            subscriptions.append(sad)

            def on_next_msg(x):
                msgs_queue.append(x[1])
                wc_stamp = time.time()
                seq = x[0]
                if is_reactive:
                    sim_stamp = round(x[0] * dt_i, 12)
                else:
                    sim_stamp = (wc_stamp - start) / real_time_factor
                t_i_queue.append(Stamp(seq, sim_stamp, wc_stamp))
                next(x)

            sad = SingleAssignmentDisposable()
            if not is_reactive and simulate_delays:
                source_msg_delayed = source_msg.pipe(ops.delay(params["delay"] / real_time_factor))
            else:
                source_msg_delayed = source_msg
            sad.disposable = source_msg_delayed.subscribe(on_next_msg, observer.on_error, observer.on_completed, scheduler)
            subscriptions.append(sad)

            return CompositeDisposable(subscriptions)

        return rx.create(subscribe)

    return _generate_msgs


def create_channel(
    ns,
    Nc,
    rate_node,
    inpt,
    is_reactive,
    real_time_factor,
    simulate_delays,
    E,
    scheduler,
    is_feedthrough,
    node,
):
    if is_feedthrough:
        name = inpt["feedthrough_to"]
    else:
        name = inpt["name"]

    # Readable format
    Is = inpt["reset"]
    Ir = inpt["msg"].pipe(
        ops.observe_on(scheduler),
        ops.map(inpt["converter"].convert),
        ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
        ops.share(),
    )

    # Get rate from rosparam server
    if inpt["external_rate"] and inpt["external_rate"] > 0:
        rate = inpt["external_rate"]
    else:
        rate_str = "%s/rate/%s" % (ns, inpt["address"][len(ns) + 1 :])
        rate = get_param_with_blocking(rate_str)

    # Create input channel
    if real_time_factor == 0:
        Nc = Nc.pipe(ops.observe_on(scheduler), ops.start_with(0))
    else:
        Nc = Nc.pipe(ops.observe_on(scheduler))

    channel = Ir.pipe(
        generate_msgs(
            Nc,
            rate_node,
            name,
            rate,
            params=inpt,
            is_reactive=is_reactive,
            real_time_factor=real_time_factor,
            simulate_delays=simulate_delays,
            node=node,
        ),
        ops.share(),
    )

    # Create reset flag
    flag = Ir.pipe(
        ops.map(lambda val: val[0] + 1),
        ops.start_with(0),
        ops.combine_latest(Is.pipe(ops.map(lambda msg: msg.data))),  # Depends on ROS reset msg type
        ops.filter(lambda value: not is_reactive or value[0] == value[1]),
        ops.map(lambda x: {name: x[0]}),
    )
    return channel, flag


def init_channels(
    ns,
    Nc,
    rate_node,
    inputs,
    is_reactive,
    real_time_factor,
    simulate_delays,
    E,
    scheduler,
    node,
    is_feedthrough=False,
):
    # Create channels
    channels = []
    flags = []
    for i in inputs:
        channel, flag = create_channel(
            ns,
            Nc,
            rate_node,
            i,
            is_reactive,
            real_time_factor,
            simulate_delays,
            E,
            scheduler,
            is_feedthrough,
            node,
        )
        channels.append(channel)
        if is_feedthrough:
            name = i["address"]
        else:
            name = i["name"]
        flag_name = "flag [%s]" % name.split("/")[-1][:12].ljust(4)
        # flag.pipe(spy(f'sub_{flag_name}', node)).subscribe(print)
        flag = flag.pipe(spy(flag_name, node))  # , ops.take(1), ops.merge(rx.never()))
        flags.append(flag)
    zipped_flags = rx.zip(*flags).pipe(ops.map(lambda x: merge_dicts({}, x)))
    zipped_channels = rx.zip(*channels).pipe(
        ops.combine_latest(
            E.pipe(ops.observe_on(scheduler))
        ),  # Latch output on '/end_reset' --> Can only receive 1 each episode.
        ops.map(lambda x: x[0]),
        regroup_inputs(node, rate_node=rate_node),
        ops.share(),
    )
    return zipped_channels, zipped_flags


def init_real_reset(
    ns,
    Nc,
    rate_node,
    RR,
    real_reset,
    feedthrough,
    targets,
    is_reactive,
    real_time_factor,
    simulate_delays,
    E,
    scheduler,
    node,
):
    # Create real reset pipeline
    dispose = []
    if real_reset:
        for i in feedthrough:
            rate_str = "%s/rate/%s" % (ns, i["address"][len(ns) + 1 :])
            rate_in = get_param_with_blocking(rate_str)
            if not rate_in == rate_node:
                raise ValueError(
                    "Rate of the reset node (%s) must be exactly the same as the feedthrough node rate (%s)."
                    % (rate_node, rate_in)
                )

        # Create zipped action channel
        zipped_channels, zipped_flags = init_channels(
            ns,
            Nc,
            rate_node,
            feedthrough,
            is_reactive,
            real_time_factor,
            simulate_delays,
            E,
            scheduler,
            node,
            is_feedthrough=True,
        )

        # Create switch subject
        target_signal = rx.zip(*[t["msg"] for t in targets])
        RR_ho = BehaviorSubject(zipped_channels)
        d_RR_ho = RR.pipe(
            ops.combine_latest(target_signal),  # make switch logic wait for all targets to be received.
            ops.map(lambda x: Nc.pipe(ops.map(lambda x: None), ops.start_with(None))),
        ).subscribe(RR_ho)
        rr_channel = RR_ho.pipe(ops.switch_latest())

        # Add disposables
        dispose += [RR_ho, d_RR_ho]
    else:
        # Create switch Subject
        zipped_flags = rx.never().pipe(ops.start_with({}))
        rr_channel = Nc.pipe(ops.map(lambda x: None), ops.start_with(None))

    return rr_channel, zipped_flags, dispose


def init_target_channel(states, scheduler, node):
    channels = []
    for s in states:
        c = s["msg"].pipe(
            ops.map(s["converter"].convert),
            ops.share(),
            ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
            remap_target(s["name"], node.is_reactive, node.real_time_factor),
        )
        channels.append(c)
    return rx.zip(*channels).pipe(regroup_inputs(node, is_input=False))


def merge_dicts(dict_1, dict_2):
    if isinstance(dict_2, dict):
        dict_2 = (dict_2,)
    for d in dict_2:
        dict_1.update(d)
    return dict_1


def init_state_inputs_channel(ns, state_inputs, scheduler, node):
    if len(state_inputs) > 0:
        channels = []
        for s in state_inputs:
            d = s["done"].pipe(
                ops.map(lambda msg: bool(msg.data)),
                ops.scan(lambda acc, x: x if x else acc, False),
            )
            c = s["msg"].pipe(
                ops.map(s["converter"].convert),
                ops.share(),
                ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
                ops.start_with((-1, None)),
                ops.combine_latest(d),
                ops.filter(lambda x: x[0][0] >= 0 or x[1]),
                remap_state(s["name"], node.is_reactive, node.real_time_factor),
            )
            channels.append(c)
        return rx.zip(*channels).pipe(regroup_inputs(node, is_input=False), ops.merge(rx.never()))
    else:
        return rx.never().pipe(ops.start_with(dict()))


def call_state_reset(state):
    def _call_state_reset(source):
        def subscribe(observer, scheduler=None):
            def on_next(state_msg):
                state.reset(state=state_msg.msgs[0], done=state_msg.info.done)
                observer.on_next(state_msg)

            return source.subscribe(on_next, observer.on_error, observer.on_completed, scheduler)

        return rx.create(subscribe)

    return _call_state_reset


def init_state_resets(ns, state_inputs, trigger, scheduler, node):
    if len(state_inputs) > 0:
        channels = []

        for s in state_inputs:
            d = s["done"].pipe(
                ops.map(lambda msg: bool(msg.data)),
                ops.scan(lambda acc, x: x if x else acc, False),
            )
            c = s["msg"].pipe(
                ops.map(s["converter"].convert),
                ops.share(),
                ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
                ops.start_with((-1, None)),
                ops.combine_latest(d),
                ops.filter(lambda x: x[0][0] >= 0 or x[1]),
                remap_state(s["name"], node.is_reactive, node.real_time_factor),
            )

            done, reset = trigger.pipe(
                with_latest_from(c),
                ops.take(1),
                ops.merge(rx.never()),
                ops.map(lambda x: x[1]),
                ops.partition(lambda x: x.info.done),
            )
            reset = reset.pipe(call_state_reset(s["state"]))
            rs = rx.merge(
                done.pipe(spy("done [%s]" % s["name"].split("/")[-1][:12].ljust(4), node)),
                reset.pipe(spy("reset [%s]" % s["name"].split("/")[-1][:12].ljust(4), node)),
            )

            channels.append(rs)
        return rx.zip(*channels).pipe(regroup_inputs(node, is_input=False), ops.merge(rx.never()))
    else:
        return rx.never().pipe(ops.start_with(dict()))


def init_callback_pipeline(
    ns,
    cb_tick,
    cb_ft,
    stream,
    real_reset,
    targets,
    state_outputs,
    outputs,
    scheduler,
    node,
):
    d_msg = []
    if real_reset:
        target_stream = init_target_channel(targets, scheduler, node)

        # Split stream into feedthrough (ft) and reset stream
        reset_stream, ft_stream = stream.pipe(ops.partition(lambda x: x[1][1] is None))

        # Either feedthrough action or run callback
        ft_stream = ft_stream.pipe(
            ops.map(lambda x: x[1][1]),
            spy("CB_FT", node, log_level=DEBUG, mapper=remap_cb_input(mode=0)),
            ops.map(lambda val: cb_ft(val, node.is_reactive)),
            ops.share(),
        )
        reset_stream = reset_stream.pipe(
            ops.map(lambda x: x[1][0]),
            ops.combine_latest(target_stream),
            spy("CB_RESET", node, log_level=DEBUG, mapper=remap_cb_input(mode=0)),
            ops.map(lambda val: cb_tick(**val[0], **val[1])),
            ops.share(),
        )
        output_stream = rx.merge(reset_stream, ft_stream)

        # Send done flags
        for s in state_outputs:
            d = reset_stream.pipe(
                spy("reset", node, log_level=DEBUG),
                ops.pluck(s["name"] + "/done"),
                ops.share(),
            ).subscribe(s["msg"])

            # Add disposable
            d_msg += [d]
    else:
        output_stream = stream.pipe(
            ops.filter(lambda x: x[1][1] is None),
            ops.map(lambda x: x[1][0]),
            spy("CB_TICK", node, log_level=DEBUG, mapper=remap_cb_input(mode=0)),
            ops.map(lambda val: cb_tick(**val)),
            ops.share(),
        )
    return d_msg, output_stream


def get_node_params(msg):
    node_name = msg.data
    node_params = get_param_with_blocking(node_name)
    if node_params is None:
        rospy.logwarn(
            "Parameters for object registry request (%s) not found on parameter server. Timeout: object (%s) not registered."
            % (msg.data, msg.data)
        )
    return node_params


def extract_node_reset(ns, node_params, sp_nodes, launch_nodes):
    name = node_params["name"]
    nf = dict(name=name, address="%s/%s/end_reset" % (ns, name), msg=Subject(), msg_type=Bool)
    return dict(
        inputs=[],
        reactive_proxy=[],
        state_inputs=[],
        node_flags=[nf],
        sp_nodes=sp_nodes,
        launch_nodes=launch_nodes,
    )


def get_object_params(msg):
    obj_name = msg.data

    obj_params = get_param_with_blocking(obj_name)
    if obj_params is None:
        rospy.logwarn(
            "Parameters for object registry request (%s) not found on parameter server. Timeout: object (%s) not registered."
            % (msg.data, msg.data)
        )
        return None

    # Get state parameters from ROS param server
    state_params = obj_params["states"]

    # Get parameters from ROS param server
    node_params = []
    for node_name in obj_params["node_names"]:
        params = get_param_with_blocking(node_name)
        node_params.append(params)
    return obj_params, node_params, state_params


def extract_inputs_and_reactive_proxy(ns, node_params, state_params, sp_nodes, launch_nodes):
    inputs = []
    state_inputs = []
    reactive_proxy = []
    node_flags = []

    # Process states
    for i in state_params:
        name = i["name"]

        # Convert to classes
        i["msg_type"] = get_attribute_from_module(i["msg_type"])
        if "converter" in i and isinstance(i["converter"], dict):
            i["converter"] = initialize_converter(i["converter"])
        elif "converter" not in i:
            i["converter"] = Identity()

        # Initialize rx objects
        i["msg"] = Subject()  # S
        i["done"] = Subject()  # D

        # Create a new state
        s = dict()
        s.update(i)
        state_inputs.append(s)

    # Process nodes
    converted_outputs = dict()  # {address: (msg_type_out, converter, msg_type_ros, source)}
    for params in node_params:
        name = params["name"]

        # Process node flags
        nf = dict(
            name=name,
            address="%s/%s/end_reset" % (ns, name),
            msg=Subject(),
            msg_type=Bool,
        )
        node_flags.append(nf)

        for i in params["outputs"]:
            ros_msg_type = get_opposite_msg_cls(i["msg_type"], i["converter"])

            # Infer input (ROS) message type from  output msg_type and converter
            if not i["converter"] == Identity().get_yaml_definition():
                if params["name"].split("/")[-2] == "sensors":  # if output is also the sensor output
                    split_name = params["name"].split("/")
                    source = ("/".join(split_name[:-2]), split_name[-2], split_name[-1])
                else:
                    source = (params["name"], "outputs", i["name"])
                converted_outputs[i["address"]] = (
                    get_attribute_from_module(i["msg_type"]),
                    i["converter"],
                    ros_msg_type,
                    source,
                )

            # Create a new input topic for each SimNode output topic
            n = RxInput(
                name=i["address"],
                address=i["address"],
                msg_type=get_module_type_string(ros_msg_type),
                external_rate=None,
                window=0,
            ).build()

            # Convert to classes
            n["msg_type"] = get_attribute_from_module(n["msg_type"])
            if isinstance(i["converter"], dict):
                n["converter"] = initialize_converter(i["converter"])
            else:
                n["converter"] = i["converter"]

            # Initialize rx objects
            n["msg"] = Subject()  # Ir
            n["reset"] = Subject()  # Is
            inputs.append(n)

        for i in params["inputs"]:
            if i["external_rate"]:
                # Convert to classes
                i["msg_type"] = get_attribute_from_module(i["msg_type"])
                if isinstance(i["converter"], dict):
                    i["converter"] = initialize_converter(i["converter"])

                # Initialize rx reset output for reactive input
                i["reset"] = Subject()

                # Create a new output topic for each SimNode reactive input (bridge sends reset msg)
                o = dict()
                o.update(i)
                reactive_proxy.append(o)

    # Check that converted outputs do not break the object's simulation graph (some nodes might expect a non-converted output message).
    for params in node_params:
        for i in params["inputs"]:
            if i["address"] in converted_outputs:
                # determine message conversion
                msg_type_out = converted_outputs[i["address"]][0]
                converter_out = converted_outputs[i["address"]][1]
                msg_type_ros = converted_outputs[i["address"]][2]
                converter_in = i["converter"]
                msg_type_in = get_opposite_msg_cls(msg_type_ros, i["converter"])
                msg_type_in_yaml = get_attribute_from_module(i["msg_type"])

                target = (params["name"], "inputs", params["inputs"][0]["name"])
                source = converted_outputs[i["address"]][3]

                msg_type_str = msg_type_error(
                    source,
                    target,
                    msg_type_out,
                    converter_out,
                    msg_type_ros,
                    converter_in,
                    msg_type_in,
                    msg_type_in_yaml,
                )
                assert msg_type_in == msg_type_in_yaml, msg_type_str

    return dict(
        inputs=inputs,
        reactive_proxy=reactive_proxy,
        state_inputs=state_inputs,
        node_flags=node_flags,
        sp_nodes=sp_nodes,
        launch_nodes=launch_nodes,
    )


def initialize_reactive_proxy_reset(rate_node, RM, reactive_proxy, node):
    for rp in reactive_proxy:
        if "disposable" in rp:
            continue
        rate_in = rp["external_rate"]
        rp["disposable"] = RM.pipe(
            ops.map(lambda msg: msg.data),
            ops.map(
                lambda idx_n: 1 + int((idx_n - 1) * rate_in // rate_node)
            ),  # We subtract -1 from msg to account for the initial tick.
            ops.map(lambda i: UInt64(data=i)),
        ).subscribe(rp["reset"])
    return None


def switch_with_check_pipeline(init_ho=None):
    if init_ho is None:
        stream_ho = Subject()
    else:
        stream_ho = BehaviorSubject(init_ho)
    check, stream = stream_ho.pipe(ops.switch_latest(), ops.partition(lambda event: event is None))
    return check, stream, stream_ho


def node_reset_flags(ns, node_flags, node):
    flags = [
        nf["msg"].pipe(
            flag_dict(nf["name"]),
            spy("has_reset", node),
            ops.start_with({nf["name"]: False}),
        )
        for nf in node_flags
    ]
    init_dict = dict()
    for nf in node_flags:
        init_dict[nf["name"]] = False
    stream = rx.merge(*flags).pipe(
        ops.scan(lambda acc, x: merge_dicts(acc, x), init_dict),
        ops.filter(lambda x: any([value for key, value in x.items()])),
        filter_dict(),
        spy("awaiting", node, log_level=DEBUG),
        ops.filter(lambda x: len(x) == 0),
        ops.start_with(None),
    )
    return stream


def filter_dict_on_key(key):
    def _filter_dict_on_key(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):
                if key in value:
                    observer.on_next(value[key])

            return source.subscribe(on_next, observer.on_error, observer.on_completed, scheduler)

        return rx.create(subscribe)

    return _filter_dict_on_key


def throttle_with_time(dt, node, rate_tol: float = 0.95, log_level: int = INFO):
    time_fn = time.perf_counter
    node_name = node.ns_name
    color = node.color
    print_mode = node.print_mode
    effective_log_level = logging.getLogger("rosout").getEffectiveLevel()
    log_time = 2  # [s]

    def _throttle_with_time(source):
        def subscribe(observer, scheduler=None):
            # Timing
            tic = [None]
            cum_cbs = [0]
            cum_delay = [0]
            cum_sleep = [0]

            # Logging
            last_cum_cbs = [0]
            last_cum_delay = [0]
            last_cum_sleep = [0]
            last_Nc = [0]
            last_time = [None]

            def on_next(Nc):
                if tic[0] is None:
                    tic[0] = time_fn()
                elif last_time[0] is None:
                    last_time[0] = time_fn()
                toc = time_fn()
                sleep_time = dt - (toc - tic[0])
                if sleep_time > 0:  # sleep if overdue is negative
                    time.sleep(sleep_time)
                    cum_sleep[0] += sleep_time
                else:  # If we are overdue, then next tick is shifted by overdue
                    cum_delay[0] += -sleep_time
                    cum_cbs[0] += 1
                tic[0] = toc + max(sleep_time, 0)

                # Logging
                curr = time_fn()
                if last_time[0] and (curr - last_time[0]) > log_time:
                    # Calculate statistics since last logged instance
                    log_window = curr - last_time[0]
                    log_cbs = cum_cbs[0] - last_cum_cbs[0]
                    # log_delay = cum_delay[0] - last_cum_delay[0]
                    log_sleep = cum_sleep[0] - last_cum_sleep[0]
                    log_Nc = Nc - last_Nc[0]

                    # Log statistics if not keeping rate
                    Nc_expected = log_window / dt  # [ticks]
                    rate_ratio = log_Nc / Nc_expected
                    cbs_ratio = log_cbs / log_Nc
                    sleep_ratio = log_sleep / log_window
                    # delay_ratio = log_delay / log_window
                    if rate_ratio < rate_tol and node.log_level >= effective_log_level and WARN >= effective_log_level:
                        print_str = f"Running at {rate_ratio*100:.2f}% of rate ({1/dt} Hz) | {sleep_ratio*100:.2f}% sleep | {100 - sleep_ratio*100:.2f}% computation | {cbs_ratio*100: .2f}% callbacks delayed |"
                        print_info(
                            node_name,
                            "red",
                            f"last {log_window:.2f} s",
                            trace_type="",
                            value=print_str,
                            print_mode=print_mode,
                            log_level=WARN,
                        )
                    elif node.log_level >= effective_log_level and log_level >= effective_log_level:
                        print_str = f"Running at {rate_ratio*100:.2f}% of rate ({1/dt} Hz) | {sleep_ratio*100:.2f}% sleep | {100 - sleep_ratio*100:.2f}% computation | {cbs_ratio*100: .2f}% callbacks delayed |"
                        print_info(
                            node_name,
                            color,
                            f"last {log_window:.2f} s",
                            trace_type="",
                            value=print_str,
                            print_mode=print_mode,
                            log_level=log_level,
                        )

                    # Set baseline statistics
                    last_time[0] = curr
                    last_cum_cbs[0] = cum_cbs[0]
                    last_cum_delay[0] = cum_delay[0]
                    last_cum_sleep[0] = cum_sleep[0]
                    last_Nc[0] = Nc

                # Send tick for next callback
                observer.on_next(Nc)

            return source.subscribe(on_next, observer.on_error, observer.on_completed, scheduler)

        return rx.create(subscribe)

    return _throttle_with_time


def throttle_callback_trigger(rate_node, Nc, E, is_reactive, real_time_factor, scheduler, node):
    # return Nc
    if is_reactive and real_time_factor == 0:
        Nct = Nc
    else:
        assert (
            real_time_factor > 0
        ), "The real_time_factor must be larger than zero when *not* running reactive (i.e. asychronous)."
        wc_dt = 1 / (rate_node * real_time_factor)
        Nct = Nc.pipe(
            ops.scan(lambda acc, x: acc + 1, 0),
            ops.start_with(0),
            ops.observe_on(scheduler),
            throttle_with_time(wc_dt, node),
            ops.share(),
        )
    return Nct


def with_latest_from(*sources: Observable):
    def _with_latest_from(parent: Observable) -> Observable:
        NO_VALUE = NotSet()

        def subscribe(observer, scheduler=None):
            def subscribe_all(parent, *children):
                parent_queued = [None]
                values = [NO_VALUE for _ in children]

                def subscribe_child(i, child):
                    subscription = SingleAssignmentDisposable()

                    def on_next(value):
                        with parent.lock:
                            values[i] = value
                            if parent_queued[0] is not None and NO_VALUE not in values:
                                result = (parent_queued[0],) + tuple(values)
                                parent_queued[0] = None
                                observer.on_next(result)

                    subscription.disposable = child.subscribe_(on_next, observer.on_error, scheduler=scheduler)
                    return subscription

                parent_subscription = SingleAssignmentDisposable()

                def on_next(value):
                    with parent.lock:
                        if NO_VALUE not in values:
                            result = (value,) + tuple(values)
                            observer.on_next(result)
                        else:
                            parent_queued[0] = value

                disp = parent.subscribe_(on_next, observer.on_error, observer.on_completed, scheduler)
                parent_subscription.disposable = disp

                children_subscription = [subscribe_child(i, child) for i, child in enumerate(children)]

                return [parent_subscription] + children_subscription

            return CompositeDisposable(subscribe_all(parent, *sources))

        return Observable(subscribe)

    return _with_latest_from


class NotSet:
    """Sentinel value."""

    def __eq__(self, other):
        return self is other

    def __repr__(self):
        return "NotSet"
