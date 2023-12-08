# RX IMPORTS
import rx
from rx import Observable, typing, operators as ops
from rx.disposable import Disposable, SingleAssignmentDisposable, CompositeDisposable
from rx.subject import Subject, BehaviorSubject
from rx.internal.concurrency import synchronized

# EAGERX IMPORTS
import eagerx.utils.utils
from eagerx.core.constants import (  # noqa
    SILENT,
    DEBUG,
    INFO,
    ERROR,
    WARN,
    FATAL,
)
from eagerx.utils.utils import (
    Info,
    Msg,
    Stamp,
)

# OTHER IMPORTS
import time
from math import floor
from collections import deque
from termcolor import colored
import datetime
import traceback
from os import getpid
from threading import current_thread, RLock
from typing import Callable, Optional, List, Any
import numpy as np


def cb_ft(cb_input, sync):
    # Fill output msg with number of node ticks
    output_msgs = dict()
    for key, msg in cb_input.items():
        if key not in ["node_tick", "t_n"]:
            if len(msg.msgs) > 0:
                output_msgs[key] = msg.msgs[-1]
            else:
                assert not sync, "Actions must always be fed through if we are running reactively."
                output_msgs[key] = None
    return output_msgs


def print_info(
    node_name,
    color,
    id=None,
    trace_type=None,
    value=None,
    date=None,
    log_level=DEBUG,
):
    msg = ""
    if date:
        msg += f"[{str(date)[:40].ljust(20)}]"
    # Add process ID
    msg += f"[{str(getpid())[:5].ljust(5)}]"
    # Add thread ID
    msg += f"[{current_thread().name.split('/')[-1][:15].ljust(15)}]"
    msg += f"[{node_name.split('/')[-1][:12].ljust(12)}]"
    if id:
        msg += f"[{id.split('/')[-1][:12].ljust(12)}]"
    msg += f" {trace_type}: {value}\n"
    print(colored(msg, color), end="")


def spy(id: str, node, log_level: int = DEBUG, mapper: Callable = lambda msg: msg):
    node_name = node.ns_name
    color = node.color

    effective_log_level = node.backend.log_level

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
):  # pragma: no cover
    node_name = node.ns_name
    color = node.color

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
                        log_level=ERROR,
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
                flag_dict = {name: value}
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


def create_msg_tuple(name: str, node_tick: int, msg: List[Any], stamp: List[Stamp], done: bool = None):
    info = Info(name=name, node_tick=node_tick, t_in=stamp, done=done)
    return Msg(info, msg)


def remap_state(name, sync, real_time_factor):
    def _remap_state(source):
        def subscribe(observer, scheduler=None):
            start = time.time()
            seq = [0]

            def on_next(value):
                node_tick = value[0][0]
                msg = value[0][1]
                done = value[1]
                wc = time.time()
                if sync:
                    sc = None
                else:
                    sc = (wc - start) / real_time_factor
                stamp = Stamp(seq[0], sc, wc)
                res = create_msg_tuple(name, node_tick, [msg], [stamp], done=done)
                seq[0] += 1
                observer.on_next(res)

            return source.subscribe(on_next, observer.on_error, observer.on_completed, scheduler)

        return rx.create(subscribe)

    return _remap_state


def remap_target(name, sync, real_time_factor):
    def _remap_target(source):
        def subscribe(observer, scheduler=None):
            start = time.time()
            seq = [0]

            def on_next(value):
                node_tick = value[0]
                msg = value[1]
                wc = time.time()
                if sync:
                    sc = None
                else:
                    sc = (wc - start) / real_time_factor
                stamp = Stamp(seq[0], sc, wc)
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
    info_dict["t_in"] = [t.sc for t in info.t_in]
    if info.t_node:
        info_dict["t_node"] = [t.sc for t in info.t_node]
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
                                log_level=ERROR,
                            )

                # Send regrouped input
                observer.on_next(res)

            return source.subscribe(on_next, observer.on_error, observer.on_completed, scheduler)

        return rx.create(subscribe)

    return _regroup_inputs


def expected_inputs(N_out, rate_in, rate_out, delay, skip: bool):
    # In case of skip=True and the output rate is an exact multiple of the input rate, we need to add a slight offset
    eps = 1e-9

    # Constants
    offset = int(skip) * (int((rate_out - eps) / rate_in) if rate_out > rate_in else -1)

    # Alternative numerically unstable implementation
    # dt_out, dt_in = 1 / rate_out, 1 / rate_in
    # t_prev = dt_out * (N_out - 1 + offset) - delay
    # t = dt_out * (N_out + offset) - delay
    # N_in_prev = int(t_prev / dt_in)
    # N_in = int(t / dt_in)
    # delta = N_in - N_in_prev
    # j = (delta - 1 + int(not skip))
    # T = dt_out * N_out - delay - j * dt_in
    # correction = ceil(-T / dt_in)

    # Numerically stable implementation
    N_in_prev = int((rate_in * (N_out + offset - 1) - rate_out * rate_in * delay) // rate_out)
    N_in = int((rate_in * (N_out + offset) - rate_out * rate_in * delay) // rate_out)

    # Alternative (iterative) delay correction
    # delta = N_in - N_in_prev
    # for i in range(int(not skip), delta + int(not skip)):
    #     # Alternative numerically unstable implementation
    #     # t_in_delayed = dt_out * N_out - delay - i * dt_in
    #     # Numerically stable implementation
    #     t_in_delayed = (N_out * rate_in - delay * rate_out * rate_in - i * rate_out) / rate_in
    #     if t_in_delayed < 0:
    #         delta -= 1

    # Alternative delay correction
    delta = N_in - N_in_prev
    j = delta - 1 + int(not skip)
    # Numerically stable implementation
    T = (rate_in * N_out - rate_out * rate_in * delay - rate_out * j) / (rate_out * rate_in)
    correction = -floor(T * rate_in)
    corrected = delta - min(delta, max(0, correction))  # limits as follows: 0 < correction < delta

    # Overwrite t=0 dependencies, because there are none when skipping.
    num_est = corrected if N_out > 0 else int(not skip)
    return num_est


def generate_msgs(
    source_Nc: Observable,
    rate_node: float,
    name: str,
    rate_in: float,
    params: dict,
    sync: bool,
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
                    if not sync or len(msgs_queue) >= num_queue[0]:
                        try:
                            tick = tick_queue.pop(0)
                            if sync:
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
                        wc = time.time()
                        seq = tick
                        if sync:
                            sc = round(tick / rate_node, 12)
                        else:
                            sc = (wc - start) / real_time_factor
                        t_n = Stamp(seq, sc, wc)

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
                if sync:
                    # Calculate expected number of message to be received
                    delay = params["delay"] if simulate_delays else 0.0
                    num_msgs = expected_inputs(x, rate_in, rate_node, delay, bool(skip))
                    num_queue.append(num_msgs)
                tick_queue.append(x)
                next(x)

            subscriptions = []
            sad = SingleAssignmentDisposable()
            sad.disposable = source_Nc.subscribe(on_next_Nc, observer.on_error, observer.on_completed, scheduler)
            subscriptions.append(sad)

            def on_next_msg(x):
                msgs_queue.append(x[1])
                wc = time.time()
                seq = x[0]
                if sync:
                    sc = round(x[0] * dt_i, 12)
                else:
                    sc = (wc - start) / real_time_factor
                t_i_queue.append(Stamp(seq, sc, wc))
                next(x)

            sad = SingleAssignmentDisposable()
            if not sync and simulate_delays:
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
    sync,
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
        convert(inpt["space"], inpt["processor"], name, "inputs", node, direction="in"),
        # ops.combine_latest(E),  # Throttle with end_reset
        # ops.map(lambda x: x[0]),  # Only pass through message
        ops.observe_on(scheduler),
        ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
        ops.share(),
    )

    # Get rate from rosparam server
    rate_str = "%s/rate/%s" % (ns, inpt["address"][len(ns) + 1 :])
    rate = eagerx.utils.utils.get_param_with_blocking(rate_str, node.backend)

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
            sync=sync,
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
        ops.combine_latest(Is),  # Depends on ROS reset msg type
        ops.filter(lambda value: not sync or value[0] == value[1]),
        ops.map(lambda x: {name: x[0]}),
    )
    return channel, flag


def init_channels(
    ns,
    Nc,
    rate_node,
    inputs,
    sync,
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
            sync,
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
    sync,
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
            rate_in = eagerx.utils.utils.get_param_with_blocking(rate_str, node.backend)
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
            sync,
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
            convert(s["space"], s["processor"], s["name"], "targets", node, direction="in"),
            ops.share(),
            ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
            remap_target(s["name"], node.sync, node.real_time_factor),
        )
        channels.append(c)
    # HACK!: Why do we sometimes receive the targets twice? And is the first received the new target or the old one?
    #        In this way, we risk reusing a target twice.
    return rx.zip(*channels).pipe(regroup_inputs(node, is_input=False), ops.take(1))


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
                ops.scan(lambda acc, x: x if x else acc, False),
            )
            c = s["msg"].pipe(
                convert(s["space"], s["processor"], s["name"], "states", node, direction="in"),
                ops.share(),
                ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
                ops.start_with((-1, None)),
                ops.combine_latest(d),
                ops.filter(lambda x: x[0][0] >= 0 or x[1]),
                remap_state(s["name"], node.sync, node.real_time_factor),
            )
            channels.append(c)
        return rx.zip(*channels).pipe(regroup_inputs(node, is_input=False), ops.merge(rx.never()))
    else:
        return rx.never().pipe(ops.start_with(dict()))


def call_state_reset(state):
    def _call_state_reset(source):
        def subscribe(observer, scheduler=None):
            def on_next(state_msg):
                try:
                    state.reset(state=state_msg.msgs[0])
                except Exception as e:
                    observer.on_error(e)
                observer.on_next(state_msg)

            return source.subscribe(on_next, observer.on_error, observer.on_completed, scheduler)

        return rx.create(subscribe)

    return _call_state_reset


def init_state_resets(ns, state_inputs, trigger, scheduler, tp_scheduler, node):
    if len(state_inputs) > 0:
        channels = []

        for s in state_inputs:
            d = s["done"].pipe(
                ops.scan(lambda acc, x: x if x else acc, False),
            )
            c = s["msg"].pipe(
                convert(s["space"], s["processor"], s["name"], "states", node, direction="in"),
                ops.share(),
                ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
                ops.start_with((-1, None)),
                ops.combine_latest(d),
                ops.filter(lambda x: x[0][0] >= 0 or x[1]),
                remap_state(s["name"], node.sync, node.real_time_factor),
            )

            done, reset = trigger.pipe(
                with_latest_from(c),
                ops.take(1),
                ops.merge(rx.never()),
                ops.map(lambda x: x[1]),
                ops.partition(lambda x: x.info.done),
            )
            reset = reset.pipe(ops.observe_on(tp_scheduler), call_state_reset(s["state"]), ops.observe_on(scheduler))
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
            ops.map(lambda val: cb_ft(val, node.sync)),
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


def throttle_with_time(dt, node, rate_tol: float = 0.95, log_level: int = DEBUG):
    time_fn = lambda: time.monotonic_ns() / 1e9  # noqa: E731
    node_name = node.ns_name
    color = node.color
    effective_log_level = node.backend.log_level
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

            def on_next(value):
                Nc, start = value
                if Nc == 0:  # Do not throttle before the first callback
                    # NOTE: This is the first time we receive a value
                    sleep_time = 0.0
                else:  # Determine sleep time
                    assert tic[0] is not None, "tic is None"
                    toc = time_fn()
                    dt_comp = toc - tic[0]
                    sleep_time = dt - dt_comp  # if sleep_time > 0 then we are early, if sleep_time < 0 then we are late

                # Throttle callback
                if sleep_time > 0:  # Sleep if we are early
                    time.sleep(sleep_time)
                    cum_sleep[0] += sleep_time
                else:  # If we are overdue, the proceeed
                    cum_delay[0] += -sleep_time
                    cum_cbs[0] += 1
                tic[0] = start + Nc * dt
                # node.backend.loginfo(colored(f"[{node_name}] Nc: {Nc} | toc: {toc} | sleep_time: {sleep_time} | tic[0]: {tic[0]}", color))

                # Logging
                curr = time_fn()
                last_time[0] = last_time[0] if last_time[0] is not None else curr
                if (curr - last_time[0]) > log_time:
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
                            log_level=INFO,
                        )
                    elif node.log_level >= effective_log_level and log_level >= effective_log_level:
                        print_str = f"Running at {rate_ratio*100:.2f}% of rate ({1/dt} Hz) | {sleep_ratio*100:.2f}% sleep | {100 - sleep_ratio*100:.2f}% computation | {cbs_ratio*100: .2f}% callbacks delayed |"
                        print_info(
                            node_name,
                            color,
                            f"last {log_window:.2f} s",
                            trace_type="",
                            value=print_str,
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


def throttle_callback_trigger(rate_node, Nc, E, sync, real_time_factor, scheduler, node):
    # return Nc
    if sync and real_time_factor == 0:
        Nct = Nc
    else:
        assert (
            real_time_factor > 0
        ), "The real_time_factor must be larger than zero when *not* running reactive (i.e. asychronous)."
        wc_dt = 1 / (rate_node * real_time_factor)
        Nct = Nc.pipe(
            ops.scan(lambda acc, x: acc + 1, 0),
            ops.start_with(0),
            ops.combine_latest(E),  # .pipe(spy("Nct_E", node, log_level=INFO))),
            ops.observe_on(scheduler),
            # spy("Nct_E_CBL", node, log_level=INFO),
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


def convert(space: eagerx.Space, processor, name, component, node, direction="out"):
    OUTPUT = True if direction == "out" else False
    INPUT = True if direction == "in" else False
    space_checked = [False]
    p_msg = f" (after processing with `{processor.__class__.__qualname__}`)" if processor else ""
    assert isinstance(space, eagerx.Space), f"The space of '{name}' is not of type eagerx.Space."

    def _convert(source):
        def subscribe(observer, scheduler=None):
            def on_next(recv):
                if INPUT:
                    # Preprocess input message
                    if processor is not None:
                        recv = processor.convert(recv)

                    # Check if message complies with space (after conversion)
                    if not space_checked[0]:
                        space_checked[0] = True
                        if not space.contains(np.array(recv)):
                            shape_msg = f"(msg.shape={recv.shape} vs space.shape={space.shape})"
                            dtype_msg = f"(msg.dtype={recv.dtype} vs space.dtype={space.dtype})"
                            msg = (
                                f"[subscriber][{node.ns_name}][{component}][{name}]: Message{p_msg} does not match the defined space. "
                                f"Either a mismatch in expected shape {shape_msg}, dtype {dtype_msg}, and/or the value is out of bounds (low/high)."
                            )
                            node.backend.logwarn_once(msg, identifier=f"[{node.ns_name}][{name}]")
                    observer.on_next(recv)
                elif OUTPUT:
                    # Convert python native types to numpy arrays.
                    if isinstance(recv, float):
                        recv = np.array(recv, dtype="float32")
                    elif isinstance(recv, bool):
                        recv = np.array(recv, dtype="bool")
                    elif isinstance(recv, int):
                        recv = np.array(recv, dtype="int64")

                    # Process message
                    if processor is not None:
                        recv = processor.convert(recv)
                    else:
                        recv = recv

                    if not space_checked[0]:
                        space_checked[0] = True
                        if not space.contains(np.array(recv)):
                            shape_msg = f"(msg.shape={recv.shape} vs space.shape={space.shape})"
                            dtype_msg = f"(msg.dtype={recv.dtype} vs space.dtype={space.dtype})"
                            msg = (
                                f"[publisher][{node.ns_name}][{component}][{name}]: Message{p_msg} does not match the defined space. "
                                f"Either a mismatch in expected shape {shape_msg}, dtype {dtype_msg}, and/or the value is out of bounds (low/high)."
                            )
                            node.backend.logwarn_once(msg, identifier=f"[{node.ns_name}][{name}]")

                    observer.on_next(recv)
                else:
                    raise NotImplementedError(f"Direction not implemented: {direction}.")

            return source.subscribe(on_next, observer.on_error, observer.on_completed, scheduler)

        return rx.create(subscribe)

    return _convert
