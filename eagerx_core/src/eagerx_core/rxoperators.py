# ROS IMPORTS
import rospy
from std_msgs.msg import Bool, UInt64

# RX IMPORTS
import rx
from rx import Observable, typing, operators as ops, create
from rx.disposable import Disposable, SingleAssignmentDisposable, RefCountDisposable, CompositeDisposable
from rx.internal.utils import add_ref
from rx.subject import Subject, BehaviorSubject
from rx.internal.concurrency import synchronized

# EAGERX IMPORTS
from eagerx_core.basenode import NodeBase
from eagerx_core.baseconverter import IdentityConverter
from eagerx_core.params import RxInput
from eagerx_core.constants import SILENT, DEBUG, INFO, ERROR, FATAL, TERMCOLOR, ROS, process
from eagerx_core.utils.utils import get_attribute_from_module, initialize_converter, get_param_with_blocking, Info, Msg

# OTHER IMPORTS
import time
from collections import deque, namedtuple
from termcolor import cprint
import datetime
import logging
import traceback
from os import getpid
from threading import current_thread, RLock
from typing import Callable, Optional, List, Any

print_modes = {TERMCOLOR: 'eagerx_core.TERMCOLOR',
               ROS: 'eagerx_core.ROS'}
ros_log_fns = {SILENT: lambda print_str: None,
               DEBUG: rospy.logdebug,
               INFO: rospy.loginfo,
               ERROR: rospy.logerr,
               FATAL: rospy.logfatal}


def cb_ft(cb_input):
    # Fill output msg with number of node ticks
    output_msgs = dict()
    for key, msg in cb_input.items():
        if key not in ['node_tick', 't_n']:
            output_msgs[key] = msg.msgs[-1]
    return output_msgs


def print_info(node_name, color, id=None, trace_type=None, value=None, date=None, print_mode=TERMCOLOR, log_level=DEBUG):
    if print_mode == TERMCOLOR:
        if date:
            cprint('[' + str(date)[:40].ljust(20) + ']', color, end='')
        cprint('[' + str(getpid())[:5].ljust(5) + ']', color, end='')
        cprint('[' + current_thread().name.split('/')[-1][:15].ljust(15) + ']', color, end='')
        cprint('[' + node_name.split('/')[-1][:12].ljust(12) + ']', color, end='', attrs=['bold'])
        if id:
            cprint('[' + id.split('/')[-1][:12].ljust(12) + ']', color, end='')
        cprint((' %s: %s' % (trace_type, value)), color)
    elif print_mode == ROS:
        print_str = ''
        print_str += '[' + str(getpid())[:5].ljust(5) + ']'
        print_str += '[' + current_thread().name.split('/')[-1][:15].ljust(15) + ']'
        print_str += '[' + node_name.split('/')[-1][:12].ljust(12) + ']'
        if id:
            print_str += '[' + id.split('/')[-1][:12].ljust(12) + ']'
        print_str += ' %s: %s' % (trace_type, value)
        ros_log_fns[log_level](print_str)
    else:
        raise ValueError('Print mode not recognized. Only print_modes %s are available.' % (print_modes.values()))


def spy(id: str, node: NodeBase, log_level: int = DEBUG, mapper: Callable = lambda msg: msg):
    node_name = node.ns_name
    color = node.color
    print_mode = node.print_mode
    effective_log_level = logging.getLogger('rosout').getEffectiveLevel()

    def _spy(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):

                if node.log_level >= effective_log_level and log_level >= effective_log_level:
                    print_info(node_name, color, id, trace_type='', value=str(mapper(value)), print_mode=print_mode, log_level=log_level)
                observer.on_next(value)

            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)

        return rx.create(subscribe)
    return _spy


def trace_observable(id: str, node: NodeBase, trace_next=False, trace_next_payload=False, trace_subscribe=False, date=None):
    node_name = node.ns_name
    color = node.color
    print_mode = node.print_mode

    def _trace(source):
        def on_subscribe(observer, scheduler):
            def on_next(value):
                if trace_next is True:
                    if trace_next_payload is True:
                        print_info(node_name, color, id, 'on_next', value, date=date or datetime.datetime.now(), print_mode=print_mode, log_level=DEBUG)
                    else:
                        print_info(node_name, color, id, 'on_next', '', date=date or datetime.datetime.now(), print_mode=print_mode, log_level=DEBUG)
                observer.on_next(value)

            def on_completed():
                value = ''
                print_info(node_name, color, id, 'on_completed', value, date=date or datetime.datetime.now(), print_mode=print_mode, log_level=DEBUG)
                observer.on_completed()

            def on_error(error):
                if isinstance(error, Exception):
                    error_traceback = '%s, %s' % (error, traceback.print_tb(error.__traceback__))
                    print_info(node_name, color, id, 'on_error', error_traceback, date=date or datetime.datetime.now())
                else:
                    print_info(node_name, color, id, 'on_error', error, date=date or datetime.datetime.now(), print_mode=print_mode, log_level=rospy.ERROR)
                observer.on_error(error)

            def dispose():
                if trace_subscribe is True:
                    value = ''
                    print_info(node_name, color, id, 'dispose', value, date=date or datetime.datetime.now(), print_mode=print_mode, log_level=DEBUG)
                disposable.dispose()

            if trace_subscribe is True:
                value = ''
                print_info(node_name, color, id, 'on_subscribe', value, date=date or datetime.datetime.now(), print_mode=print_mode, log_level=DEBUG)
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

            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)

        return rx.create(subscribe)
    return _init_flag_dict


def filter_dict():
    def _filter_dict(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):
                d = dict()
                for node_name, flag in value.items():
                    if flag is True: continue
                    d[node_name] = flag
                observer.on_next(d)

            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)

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

            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)

        return rx.create(subscribe)
    return _switch_to_reset


def push_to_Nc(Nc, Rn):
    def _push_to_Nc(source):
        def subscribe(observer, scheduler=None):
            q = [0]

            def on_next(value):
                if value[0] > q[0]:
                    q[0] = value[0]
                    if not Nc.is_disposed:
                        Nc.on_next(value[0])

                if value[2] and value[0] == value[1]:
                    Rn.on_next(value)

                observer.on_next(value)

            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)

        return rx.create(subscribe)

    return _push_to_Nc


def combine_dict(acc, x):
    for key, item in acc.items():
         item += x[key]
    return acc


def publisher_to_topic(publisher):
    def _publisher_to_topic(source):
        def subscribe(observer, scheduler=None):
            def on_next(msg):
                publisher.publish(msg)
                observer.on_next(msg)

            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)

        return rx.create(subscribe)

    return _publisher_to_topic


def publish_to_topic(topic_type: Any, topic_name: str) -> Callable[[Observable], Observable]:
    """+
    The to_topic operator will take each message from the stream and publish it to a specific ROS topic.
    :param topic_type: The type of the observable data elements.
    :param topic_name: The name of the ROS2 topic to publish the messages to.
    :return: The observable data stream it operates on, i.e. it is an identity operator.
    """
    def _publish_to_topic(source) -> Observable:
        publisher = rospy.Publisher(topic_name, topic_type)
        source.subscribe(on_next=lambda msg: publisher.publish(msg))
        return source
    return _publish_to_topic


def create_msg_tuple(name: str, node_tick: int, msg: List[Any], done: bool = None):
    info = Info(name=name, node_tick=node_tick, done=done)
    return Msg(info, msg)


def remap_state(name):
    def _remap_state(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):
                node_tick = value[0][0]
                msg = value[0][1]
                done = value[1]
                res = create_msg_tuple(name, node_tick, [msg], done=done)
                observer.on_next(res)
            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)
        return rx.create(subscribe)
    return _remap_state


def remap_target(name):
    def _remap_target(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):
                node_tick = value[0]
                msg = value[1]
                res = create_msg_tuple(name, node_tick, [msg])
                observer.on_next(res)
            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)
        return rx.create(subscribe)
    return _remap_target


def remap_cb_input(mode=0):
    def _remap_cb_input(value):
        # mode=0 (info only), mode=1 (msgs only), mode=2 (all)
        if mode == 2:
            return value
        if isinstance(value, tuple):
            mapped_value = tuple([value[0].copy(), value[1].copy()])
            for i in mapped_value:
                for key, msg in i.items():
                    if key not in ['node_tick', 't_n']:
                        if mode == 0:
                            i[key] = msg.info
                        else:
                            i[key] = msg.msgs
        else:
            mapped_value = value.copy()
            for key, msg in mapped_value.items():
                if key not in ['node_tick', 't_n']:
                    if mode == 0:
                        mapped_value[key] = msg.info
                    else:
                        mapped_value[key] = msg.msgs
            return mapped_value
    return _remap_cb_input


def regroup_inputs(node: NodeBase, rate_node=1, is_input=True, perfom_checks=False):
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
                if perfom_checks and is_input:
                    node_ticks = []
                    for msg in value:
                        node_ticks.append(msg.info.node_tick)
                    if len(node_ticks) > 0:
                        if not len(set(node_ticks)) == 1:
                            print_info(node_name, color, 'regroup_inputs', trace_type='', value='Not all node_ticks are the same: %s' % str(value), print_mode=print_mode, log_level=rospy.ERROR)

                # Send regrouped input
                observer.on_next(res)

            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)

        return rx.create(subscribe)

    return _regroup_inputs


def expected_inputs(idx_n, rate_in, rate_node, delay):
    if idx_n == 0:
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
    N_in = max(0, int((rate_in * (N_node - 1) - rate_in * delay) // rate_node))  # Current timestep
    # sum_t_min_1 = max(0, int(((rate_in * (N_t_min_1 - 1) - rate_in * delay) / rate_node)))  # Current timestep
    # sum_t_min_1 = max(0, int(round(((dt_n * (N_t_min_1 - 1) - delay) / dt_i), 11)))  # Current timestep
    # sum_t_min_1 = max(0, int(((dt_n * (N_t_min_1 - 1) - delay) / dt_i)))  # Current timestep
    return N_in


def generate_msgs(source_Nc: Observable, rate_node: float, name: str, rate_in: float, window: int, delay: float, reactive: bool):
    dt_i = 1 / rate_in

    def _generate_msgs(source_msg: Observable):
        def subscribe(observer: typing.Observer,
                      scheduler: Optional[typing.Scheduler] = None) -> CompositeDisposable:
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
                # todo: if not reactive --> keep track of window length
                if len(num_queue) > 0:
                    if len(msgs_queue) >= num_queue[0]:
                        try:
                            num_msgs = num_queue.pop(0)
                            tick = tick_queue.pop(0)
                            msgs = msgs_queue[:num_msgs]
                            t_i = t_i_queue[:num_msgs]
                            msgs_queue[:] = msgs_queue[num_msgs:]
                            t_i_queue[:] = t_i_queue[num_msgs:]
                        except Exception as ex:  # pylint: disable=broad-except
                            observer.on_error(ex)
                            return

                        if window > 0:
                            wmsgs = list(msgs_window.extend(msgs))
                            wt_i = list(t_i_window.extend(t_i))
                            wt_n = list(t_n_window.extend([tick/rate_node] * len(msgs)))
                        else:
                            wmsgs = msgs
                            wt_i = t_i
                            wt_n = [round(tick/rate_node, 12)] * len(msgs)
                        res = Msg(Info(name, tick, rate_in, wt_n, wt_i), wmsgs)
                        observer.on_next(res)

            # Determine Nc logic
            def on_next_Nc(x):
                num_msgs = expected_inputs(x, rate_in, rate_node, delay)
                tick_queue.append(x)
                num_queue.append(num_msgs)
                next(num_msgs)

            subscriptions = []
            sad = SingleAssignmentDisposable()
            sad.disposable = source_Nc.subscribe(on_next_Nc, observer.on_error, observer.on_completed, scheduler)
            subscriptions.append(sad)

            def on_next_msg(x):
                msgs_queue.append(x[1])
                t_i_queue.append(round(x[0] * dt_i, 12))
                next(x)

            sad = SingleAssignmentDisposable()
            sad.disposable = source_msg.subscribe(on_next_msg, observer.on_error, observer.on_completed, scheduler)
            subscriptions.append(sad)

            return CompositeDisposable(subscriptions)
        return rx.create(subscribe)
    return _generate_msgs


def create_channel(ns, Nc, rate_node, inpt, is_reactive, real_time_factor, E, scheduler, is_feedthrough, node: NodeBase):
    if is_reactive:
        return create_reactive_channel(ns, Nc, rate_node, inpt, scheduler, is_feedthrough, real_time_factor, node)
    else:
        return create_async_channel(ns, Nc, rate_node, inpt, scheduler, is_feedthrough, real_time_factor, E, node)


def create_reactive_channel(ns, Nc, rate_node, inpt, scheduler, is_feedthrough, real_time_factor, node: NodeBase):
    # todo: remove ops.observe_on?
    if is_feedthrough:
        name = inpt['feedthrough_to']
    else:
        name = inpt['name']

    # Readable format
    Is = inpt['reset']
    Ir = inpt['msg'].pipe(ops.observe_on(scheduler),
                          ops.map(inpt['converter'].convert),
                          ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
                          ops.share())

    # Get rate from rosparam server
    try:
        # if 'is_reactive' in inpt and not inpt['is_reactive']:
        if 'rate' in inpt:
            rate = inpt['rate']
        else:
            rate_str = '%s/rate/%s' % (ns, inpt['address'][len(ns)+1:])
            rate = get_param_with_blocking(rate_str)
        dt_i = 1 / rate
    except Exception as e:
        dt_i = None
        rate = None
        print('Probably cannot find key "%s" on ros param server.' % inpt['name'] + '/rate')
        print(e)

    # Create input channel
    if real_time_factor == 0:
        Nc = Nc.pipe(ops.observe_on(scheduler),
                     ops.start_with(0))
    else:
        Nc = Nc.pipe(ops.observe_on(scheduler))

    channel = Ir.pipe(generate_msgs(Nc, rate_node, name, rate, window=0, delay=inpt['delay'], reactive=True), ops.share())

    # Create reset flag
    flag = Ir.pipe(ops.map(lambda val: val[0] + 1),
                   ops.start_with(0),
                   ops.combine_latest(Is.pipe(ops.map(lambda msg: msg.data))),  # Depends on ROS reset msg type
                   ops.filter(lambda value: value[0] == value[1]),
                   ops.map(lambda x: {name: x[0]}))
    return channel, flag


def init_channels(ns, Nc, rate_node, inputs, is_reactive, real_time_factor, E, scheduler, node: NodeBase, is_feedthrough=False):
    # Create channels
    channels = []
    flags = []
    for i in inputs:
        channel, flag = create_channel(ns, Nc, rate_node, i, is_reactive, real_time_factor, E, scheduler, is_feedthrough, node)
        channels.append(channel)
        if is_feedthrough:
            name = i['address']
        else:
            name = i['name']
        flag = flag.pipe(spy('flag [%s]' % name.split('/')[-1][:12].ljust(4), node))
        flags.append(flag)
    zipped_flags = rx.zip(*flags).pipe(ops.map(lambda x: merge_dicts({}, x)))
    zipped_channels = rx.zip(*channels).pipe(regroup_inputs(node, rate_node=rate_node), ops.share())
    return zipped_channels, zipped_flags


def init_real_reset(ns, Nc, dt_n, RR, real_reset, feedthrough, targets, is_reactive, real_time_factor, E, scheduler, node: NodeBase):
    # Create real reset pipeline
    dispose = []
    if real_reset:
        for i in feedthrough:
            rate_str = '%s/rate/%s' % (ns, i['address'][len(ns) + 1:])
            dt_i = 1 / get_param_with_blocking(rate_str)
            if not dt_i == dt_n:
                raise ValueError('Rate of the switch node (%s) must be exactly the same as the feedthrough node rate (%s).' % (dt_n, dt_i))

        # Create zipped action channel
        zipped_channels, zipped_flags = init_channels(ns, Nc, dt_n, feedthrough, is_reactive, real_time_factor, E, scheduler, node, is_feedthrough=True)

        # Create switch subject
        target_signal = rx.zip(*[t['msg'] for t in targets])
        RR_ho = BehaviorSubject(zipped_channels)
        d_RR_ho = RR.pipe(ops.combine_latest(target_signal),  # make switch logic wait for all targets to be received.
                          ops.map(lambda x: Nc.pipe(ops.map(lambda x: None), ops.start_with(None)))).subscribe(RR_ho)
        rr_channel = RR_ho.pipe(ops.switch_latest())

        # Add disposables
        dispose += [RR_ho, d_RR_ho]
    else:
        # Create switch Subject
        zipped_flags = rx.never().pipe(ops.start_with({}))
        rr_channel = Nc.pipe(ops.map(lambda x: None), ops.start_with(None))

    return rr_channel, zipped_flags, dispose


def init_target_channel(states, scheduler, node: NodeBase):
    channels = []
    for s in states:
        c = s['msg'].pipe(ops.map(s['converter'].convert), ops.share(),
                          ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
                          remap_target(s['name']))
        channels.append(c)
    return rx.zip(*channels).pipe(regroup_inputs(node, is_input=False))


def merge_dicts(dict_1, dict_2):
    if isinstance(dict_2, dict):
        dict_2 = (dict_2,)
    for d in dict_2:
        dict_1.update(d)
    return dict_1


def init_state_inputs_channel(ns, state_inputs, scheduler, node: NodeBase):
    if len(state_inputs) > 0:
        channels = []
        for s in state_inputs:
            d = s['done'].pipe(ops.map(lambda msg: bool(msg.data)),
                               ops.scan(lambda acc, x: x if x else acc, False))
            c = s['msg'].pipe(ops.map(s['converter'].convert), ops.share(),
                              ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
                              ops.start_with((-1, None)),
                              ops.combine_latest(d),
                              ops.filter(lambda x: x[0][0] >= 0 or x[1]),
                              remap_state(s['name']))
            channels.append(c)
        return rx.zip(*channels).pipe(regroup_inputs(node, is_input=False), ops.merge(rx.never()))
    else:
        return rx.never().pipe(ops.start_with(dict()))


def init_state_resets(ns, state_inputs, trigger, scheduler, node: NodeBase):
    if len(state_inputs) > 0:
        channels = []
        for s in state_inputs:
            d = s['done'].pipe(ops.map(lambda msg: bool(msg.data)),
                               ops.scan(lambda acc, x: x if x else acc, False))
            c = s['msg'].pipe(ops.map(s['converter'].convert), ops.share(),
                              ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
                              # ops.map(lambda val: dict(msg=val)),
                              ops.start_with((-1, None)),
                              # ops.start_with(dict(name=s['name'], msg=None)),
                              ops.combine_latest(d),
                              ops.filter(lambda x: x[0][0] >= 0 or x[1]),
                              remap_state(s['name']),
                              # ops.map(lambda x: merge_dicts(x[0].copy(), {'done': x[1]}))
                              )

            done, reset = trigger.pipe(ops.with_latest_from(c),
                                       ops.map(lambda x: x[1]),
                                       ops.partition(lambda x: x.info.done))
            reset = reset.pipe(ops.map(lambda x: (x, s['state'].reset(state=x.msgs[0], done=x.info.done))),
                               ops.map(lambda x: x[0]))
            rs = rx.merge(done.pipe(spy('done [%s]' % s['name'].split('/')[-1][:12].ljust(4), node)),
                          reset.pipe(spy('reset [%s]' % s['name'].split('/')[-1][:12].ljust(4), node)))

            channels.append(rs)
        return rx.zip(*channels).pipe(regroup_inputs(node, is_input=False), ops.merge(rx.never()))
    else:
        return rx.never().pipe(ops.start_with(dict()))


def init_callback_pipeline(ns, cb_tick, cb_ft, stream, real_reset, targets, state_outputs, outputs, scheduler, node: NodeBase):
    d_msg = []
    if real_reset:
        target_stream = init_target_channel(targets, scheduler, node)

        # Split stream into feedthrough (ft) and reset stream
        reset_stream, ft_stream = stream.pipe(ops.partition(lambda x: x[1][1] is None))

        # Either feedthrough action or run callback
        ft_stream = ft_stream.pipe(ops.map(lambda x: x[1][1]),
                                   spy('CB_FT', node, log_level=DEBUG, mapper=remap_cb_input(mode=0)),
                                   ops.map(lambda val: cb_ft(val)), ops.share())
        reset_stream = reset_stream.pipe(ops.map(lambda x: x[1][0]),
                                         ops.combine_latest(target_stream),
                                         spy('CB_RESET', node, log_level=DEBUG, mapper=remap_cb_input(mode=0)),
                                         ops.map(lambda val: cb_tick(**val[0], **val[1])), ops.share())
        output_stream = rx.merge(reset_stream, ft_stream)

        # Send done flags
        for s in state_outputs:
            d = reset_stream.pipe(spy('reset', node, log_level=DEBUG),
                                  ops.pluck(s['name'] + '/done'),
                                  ops.share()).subscribe(s['msg'])

            # Add disposable
            d_msg += [d]
    else:
        output_stream = stream.pipe(ops.filter(lambda x: x[1][1] is None),
                                    ops.map(lambda x: x[1][0]),
                                    spy('CB_TICK', node, log_level=DEBUG, mapper=remap_cb_input(mode=0)),
                                    ops.map(lambda val: cb_tick(**val)),
                                    ops.share())

    # Publish output msg as ROS topic and to subjects if single process
    for o in outputs:
        d = output_stream.pipe(ops.pluck(o['name']),
                               ops.map(o['converter'].convert),
                               ops.share(),
                               ).subscribe(o['msg'])

        # Add disposable
        d_msg += [d]

    return d_msg


def get_node_params(msg):
    node_name = msg.data
    node_params = get_param_with_blocking(node_name)
    if node_params is None:
        rospy.logwarn('Parameters for object registry request (%s) not found on parameter server. Timeout: object (%s) not registered.' % (msg.data, msg.data))
    return node_params


def extract_node_reset(ns, node_params, sp_nodes, launch_nodes):
    name = node_params['name']
    nf = dict(name=name, address='%s/%s/end_reset' % (ns, name), msg=Subject(), msg_type=Bool)
    return dict(inputs=[], reactive_proxy=[], state_inputs=[], node_flags=[nf], sp_nodes=sp_nodes, launch_nodes=launch_nodes)


def get_object_params(msg):
    obj_name = msg.data

    obj_params = get_param_with_blocking(obj_name)
    if obj_params is None:
        rospy.logwarn('Parameters for object registry request (%s) not found on parameter server. Timeout: object (%s) not registered.' % (msg.data, msg.data))
        return None

    # Get state parameters from ROS param server
    state_params = obj_params['states']

    # Get parameters from ROS param server
    node_params = []
    for node_name in obj_params['node_names']:
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
        name = i['name']

        # Convert to classes
        i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
        if 'converter' in i and isinstance(i['converter'], dict):
            i['converter'] = initialize_converter(i['converter'])
        elif 'converter' not in i:
            i['converter'] = IdentityConverter()

        # Initialize rx objects
        i['msg'] = Subject()  # S
        i['done'] = Subject()  # D

        # Create a new state
        s = dict()
        s.update(i)
        state_inputs.append(s)

    # Process nodes
    for params in node_params:
        name = params['name']

        # Process node flags
        nf = dict(name=name, address='%s/%s/end_reset' % (ns, name), msg=Subject(), msg_type=Bool)
        node_flags.append(nf)

        for i in params['outputs']:
            assert not params['process'] == process.BRIDGE or 'converter' not in i, 'Node "%s" has an output converter (%s) specified. That is currently not supported if launching remotely.' % (name, i['converter'])

            # Create a new input topic for each SimNode output topic
            n = RxInput(name=i['address'], address=i['address'], msg_type=i['msg_type'], msg_module=i['msg_module'],
                        is_reactive=True, window=0).get_params()

            # Convert to classes
            n['msg_type'] = get_attribute_from_module(n['msg_module'], n['msg_type'])
            if 'converter' in i and isinstance(i['converter'], dict):
                n['converter'] = initialize_converter(i['converter'])
            if 'converter' in i and not isinstance(i['converter'], dict):
                n['converter'] = i['converter']
            elif 'converter' not in i:
                n['converter'] = IdentityConverter()

            # Initialize rx objects
            n['msg'] = Subject()  # Ir
            n['reset'] = Subject()  # Is
            inputs.append(n)

        for i in params['inputs']:
            if not i['is_reactive']:
                # Convert to classes
                i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
                if 'converter' in i and isinstance(i['converter'], dict):
                    i['converter'] = initialize_converter(i['converter'])
                elif 'converter' not in i:
                    i['converter'] = IdentityConverter()

                # Initialize rx reset output for reactive input
                i['reset'] = Subject()

                # Create a new output topic for each SimNode reactive input (bridge sends reset msg)
                o = dict()
                o.update(i)
                reactive_proxy.append(o)

    return dict(inputs=inputs, reactive_proxy=reactive_proxy, state_inputs=state_inputs, node_flags=node_flags, sp_nodes=sp_nodes, launch_nodes=launch_nodes)


def initialize_reactive_proxy_reset(rate_node, RM, reactive_proxy, node):
    for rx in reactive_proxy:
        if 'disposable' in rx:
            continue
        rate_in = rx['rate']
        rx['disposable'] = RM.pipe(ops.map(lambda msg: msg.data),
                                   ops.map(lambda idx_n: 1 + int((idx_n - 1) * rate_in // rate_node)),  # We subtract -1 from msg to account for the initial tick.
                                   ops.map(lambda i: UInt64(data=i)),
                                   ).subscribe(rx['reset'])
    return None


def switch_with_check_pipeline(init_ho=None):
    if init_ho is None:
        stream_ho = Subject()
    else:
        stream_ho = BehaviorSubject(init_ho)
    check, stream = stream_ho.pipe(ops.switch_latest(), ops.partition(lambda event: event is None))
    return check, stream, stream_ho


def node_reset_flags(ns, node_flags, node: NodeBase):
    flags = [nf['msg'].pipe(flag_dict(nf['name']), spy('has_reset', node), ops.start_with({nf['name']: False})) for nf in node_flags]
    init_dict = dict()
    for nf in node_flags:
        init_dict[nf['name']] = False
    stream = rx.merge(*flags).pipe(ops.scan(lambda acc, x: merge_dicts(acc, x), init_dict),
                                   ops.filter(lambda x: any([value for key, value in x.items()])),
                                   filter_dict(),
                                   spy('awaiting', node, log_level=DEBUG),
                                   ops.filter(lambda x: len(x) == 0),
                                   ops.start_with(None))
    return stream


def from_topic(topic_type: Any, topic_name: str, node_name) -> Observable:
    def _subscribe(observer, scheduler=None) -> Disposable:
        try:
            rospy.Subscriber(topic_name, topic_type, lambda msg: observer.on_next(msg))
        except Exception as e:
            print('[%s]: %s' % (node_name, e))
        return observer
    return create(_subscribe)


def filter_dict_on_key(key):
    def _filter_dict_on_key(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):
                if key in value:
                    observer.on_next(value[key])
            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)

        return rx.create(subscribe)

    return _filter_dict_on_key


def throttle_with_time(dt, node: NodeBase):
    def _throttle_with_time(source):
        def subscribe(observer, scheduler=None):
            next_tick = [None]
            end = [None]
            cum_delay = [0]
            cum_sleep = [0]

            def on_next(value):
                end[0] = time.time()
                if next_tick[0] is None:
                    next_tick[0] = end[0] + dt
                overdue = end[0] - next_tick[0]
                if overdue < 0:  # sleep if overdue is negative
                    time.sleep(-overdue)
                    cum_sleep[0] += overdue
                    next_tick[0] = end[0] + dt
                else:  # If we are overdue, then next tick is shifted by overdue
                    cum_delay[0] += overdue
                    next_tick[0] = end[0] + dt + overdue
                observer.on_next((value, cum_delay, cum_sleep))
            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)
        return rx.create(subscribe)
    return _throttle_with_time


def throttle_callback_trigger(rate_node, Nc, E, is_reactive, real_time_factor, node: NodeBase):
    if is_reactive and real_time_factor == 0:
        Nct = Nc
    # elif is_reactive and real_time_factor > 0:
        # Nct = Nc.pipe(ops.merge(Nr.pipe(spy('Nr', node))),
        #               ops.scan(lambda acc, x: acc + 1, 0),
        #               throttle_with_time(1/(rate_node * real_time_factor), node),
        #               ops.map(lambda x: x[0]),
        #               ops.start_with(0),
        #               ops.share())
        # Nct = rx.interval(1 / (rate_node * real_time_factor)).pipe(  # ops.skip_until(E),
        #     ops.scan(lambda acc, x: acc + 1, -1),
        #     ops.combine_latest(Nc.pipe(ops.scan(lambda acc, x: acc + 1, 0),
        #                                ops.start_with(0))),
        #     ops.distinct_until_changed(key_mapper=lambda x: x,
        #                                comparer=lambda x, y: (
        #                                        x[0] == y[0] or x[1] ==
        #                                        y[1])),
        #     ops.map(lambda x: x[1]),
        #     ops.share())
    else:
        assert real_time_factor > 0, "The real_time_factor must be larger than zero when *not* running reactive (i.e. asychronous)."
        Nct = rx.interval(1 / (rate_node * real_time_factor)).pipe(  # ops.skip_until(E),
            ops.scan(lambda acc, x: acc + 1, -1),
            ops.combine_latest(Nc.pipe(ops.scan(lambda acc, x: acc + 1, 0),
                                       ops.start_with(0))),
            ops.distinct_until_changed(key_mapper=lambda x: x,
                                       comparer=lambda x, y: (
                                               x[0] == y[0] or x[1] ==
                                               y[1])),
            ops.map(lambda x: x[0]),
            ops.share())
    return Nct


def add_offset(offset, skip=0):
    def _add_offset(source):
        def subscribe(observer, scheduler=None):
            counter = [0]
            def on_next(value):
                counter[0] += 1
                if counter[0] > skip:
                    observer.on_next(value + offset)
                else:
                    observer.on_next(value)

            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)

        return rx.create(subscribe)
    return _add_offset


def create_async_channel(ns, Nc, dt_n, inpt, scheduler, is_feedthrough, real_time_factor, E, node: NodeBase):
    raise ValueError('Not implemented yet (repeat --> window).')
    if is_feedthrough:
        name = inpt['feedthrough_to']
    else:
        name = inpt['name']

    # Readable format
    Is = inpt['reset']
    Ir = inpt['msg'].pipe(ops.observe_on(scheduler),
                          ops.skip_until(E),
                          # spy('Ir_%s' % name, node),
                          ops.map(inpt['converter'].convert),
                          ops.scan(lambda acc, msg: (acc[0] + 1, msg), (-1, None)),
                          ops.share(),
                          )

    # Get rate from rosparam server
    try:
        # if 'is_reactive' in inpt and not inpt['is_reactive']:
        if 'rate' in inpt:
            rate = inpt['rate']
        else:
            rate_str = '%s/rate/%s' % (ns, inpt['address'][len(ns)+1:])
            rate = get_param_with_blocking(rate_str)
        dt_i = 1 / rate
    except Exception as e:
        print(e)
        raise ValueError('Probably cannot find key "%s" on ros param server.' % inpt['name'] + '/rate')

    # Create input channel
    num_msgs = Nc.pipe(ops.observe_on(scheduler),
                       # ops.start_with(0),
                       spy('Nct_%s' % name, node),
                       ops.map(lambda i: {'node_tick': i, 'num_msgs': expected_inputs(i, dt_i, dt_n, inpt['delay'])}),
                       ops.share())
    msg = Ir.pipe(#spy('Ir_%s' % name, node),
                  ops.buffer(num_msgs),
                  # spy('buf_%s' % name, node),
                  ops.zip(num_msgs),  # ([(0, data: "string: 0"), (0, data: "string: 1")], {'node_tick': 0, 'num_msgs': 0})
                  ops.map(lambda x: [dict(x[1], **{'msgs': msg}) for msg in x[0]]),  # [{'node_tick': 0, 'num_msgs': 0, 'msg': (0, data: "string: 0")}, {'node_tick': 0, 'num_msgs': 0, 'msg': (0, data: "string: 1")}]
                  ops.filter(lambda x: len(x) > 0),
                  spy('fil_%s' % name, node),
                  ops.share()
                  )
    channel = num_msgs.pipe(ops.filter(lambda x: x['num_msgs'] == 0),
                            ops.with_latest_from(msg.pipe(ops.start_with([]))),  # todo: msg gets dropped here
                            ops.map(get_repeat_fn(inpt['repeat'], 'msgs')),
                            ops.merge(msg),
                            remap_state_target(name, dt_i=dt_i, dt_n=dt_n),
                            spy('%s' % name, node),
                            ops.share())

    # Create reset flag
    flag = Ir.pipe(ops.map(lambda val: val[0] + 1),
                   ops.start_with(0),
                   ops.combine_latest(Is.pipe(ops.map(lambda msg: msg.data))),  # Depends on ROS reset msg type
                   # ops.filter(lambda value: value[0] == value[1]),
                   ops.map(lambda x: {name: x[0]})
                   )
    return channel, flag


