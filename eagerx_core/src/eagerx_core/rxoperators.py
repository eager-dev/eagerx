# ROS IMPORTS
import rospy
from std_msgs.msg import Bool, UInt64

# RX IMPORTS
import rx
from rx import Observable, operators as ops, create
from rx.disposable import Disposable, SingleAssignmentDisposable, RefCountDisposable
from rx.internal.utils import add_ref
from rx.subject import Subject, BehaviorSubject

# EAGERX IMPORTS
from eagerx_core.basenode import NodeBase
from eagerx_core.baseconverter import IdentityConverter
from eagerx_core.params import RxInput
from eagerx_core.constants import SILENT, DEBUG, INFO, ERROR, FATAL, TERMCOLOR, ROS
from eagerx_core.utils.utils import get_attribute_from_module, initialize_converter, get_param_with_blocking

# OTHER IMPORTS
from termcolor import cprint
import datetime
import logging
import traceback
from os import getpid
from threading import current_thread
from typing import Any, Callable

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
    for key, item in cb_input.items():
        if key not in ['node_tick', 't_n']:
            output_msgs[key] = item['msg'][0]
    return output_msgs


def print_info(node_name, color, id=None, trace_type=None, value=None, date=None, print_mode=TERMCOLOR, op_log_level=DEBUG):
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
        ros_log_fns[op_log_level](print_str)
    else:
        raise ValueError('Print mode not recognized. Only print_modes %s are available.' % (print_modes.values()))


def spy(id: str, node: NodeBase, op_log_level: int = SILENT):
    node_name = node.ns_name
    color = node.color
    print_mode = node.print_mode
    effective_log_level = logging.getLogger('rosout').getEffectiveLevel()

    def _spy(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):
                if node.log_level >= effective_log_level and op_log_level >= effective_log_level:
                    print_info(node_name, color, id, trace_type='', value=str(value), print_mode=print_mode, op_log_level=op_log_level)
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
                        print_info(node_name, color, id, 'on_next', value, date=date or datetime.datetime.now(), print_mode=print_mode, op_log_level=DEBUG)
                    else:
                        print_info(node_name, color, id, 'on_next', '', date=date or datetime.datetime.now(), print_mode=print_mode, op_log_level=DEBUG)
                observer.on_next(value)

            def on_completed():
                value = ''
                print_info(node_name, color, id, 'on_completed', value, date=date or datetime.datetime.now(), print_mode=print_mode, op_log_level=DEBUG)
                observer.on_completed()

            def on_error(error):
                if isinstance(error, Exception):
                    error_traceback = '%s, %s' % (error, traceback.print_tb(error.__traceback__))
                    print_info(node_name, color, id, 'on_error', error_traceback, date=date or datetime.datetime.now())
                else:
                    print_info(node_name, color, id, 'on_error', error, date=date or datetime.datetime.now(), print_mode=print_mode, op_log_level=rospy.ERROR)
                observer.on_error(error)

            def dispose():
                if trace_subscribe is True:
                    value = ''
                    print_info(node_name, color, id, 'dispose', value, date=date or datetime.datetime.now(), print_mode=print_mode, op_log_level=DEBUG)
                disposable.dispose()

            if trace_subscribe is True:
                value = ''
                print_info(node_name, color, id, 'on_subscribe', value, date=date or datetime.datetime.now(), print_mode=print_mode, op_log_level=DEBUG)
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


def regroup_msgs(name, dt_i=None, dt_n=None):
    def _regroup_msgs(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):
                res = dict(name=name)

                # Sort all msgs
                if not isinstance(value, list):
                    value = [value]
                msgs = [msg['msg'][1] for msg in value]
                counts = [msg['msg'][0] for msg in value]
                res['msg'] = msgs

                # Check that all node_ticks are the same
                if 'node_tick' in value[0]:
                    res['node_tick'] = value[0]['node_tick']
                    assert len(set([msg['node_tick'] for msg in value])) == 1, 'Not all node_ticks are the same: %s' % str(value)
                    # Check that all timestamps are smaller or equal to node time
                    if dt_n is not None and dt_i is not None:
                        t_n = value[0]['node_tick'] * dt_n
                        t_i = [count * dt_i if count is not None else None for count in counts]
                        res['t_i'] = t_i
                        res['dt_i'] = dt_i
                        assert len(t_i) == 0 or all(t <= t_n for t in t_i if t is not None), 'Not all t_msgs are smaller or equal to t_n: %s' % str(res)

                # Check that all num_msgs are the same
                if 'num_msgs' in value[0]:
                    res['num_msgs'] = value[0]['num_msgs']
                    assert len(set([msg['num_msgs'] for msg in value])) == 1, 'Not all num_msgs are the same: %s' % str(value)
                observer.on_next(res)

            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)

        return rx.create(subscribe)

    return _regroup_msgs


def regroup_inputs(node: NodeBase, dt_n=None):
    node_name = node.ns_name
    color = node.color
    print_mode = node.print_mode
    def _regroup_inputs(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):
                res = dict()

                # Store node tick and t_n
                if 'node_tick' in value[0]:
                    res['node_tick'] = value[0]['node_tick']
                    res['t_n'] = round(value[0]['node_tick'] * dt_n, 2)

                # Regroup msg dicts
                node_ticks = []
                for I in value:
                    try:
                        name = I['name']
                    except Exception as e:
                        pass
                    res[name] = I
                    del res[name]['name']

                    # check that num_msg is equal to number of msgs if not zero
                    if 'num_msgs' in I:
                        num_msgs = I['num_msgs']
                        assert num_msgs == 0 or len(I['msg']) == num_msgs, 'num_msgs not equal to len(msgs): %s' % str(I)

                    if 'node_tick' in I:
                        node_ticks.append(I['node_tick'])
                        del res[name]['node_tick']

                # Check that all node_ticks are the same
                if len(node_ticks) > 0:
                    if not len(set(node_ticks)) == 1:
                        print_info(node_name, color, 'regroup_inputs', trace_type='', value='Not all node_ticks are the same: %s' % str(value), print_mode=print_mode, op_log_level=rospy.ERROR)
                observer.on_next(res)

            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)

        return rx.create(subscribe)

    return _regroup_inputs


def _window_with_variable_count() -> Callable[[Observable], Observable]:
    def window_with_variable_count(source: Observable) -> Observable:
        def subscribe(observer, scheduler=None):
            m = SingleAssignmentDisposable()
            refCountDisposable = RefCountDisposable(m)
            n = [0]  # total_count
            q = []

            def create_window():
                s = Subject()
                q.append(s)
                observer.on_next(add_ref(s, refCountDisposable))

            create_window()

            def on_next(x):
                skip = x['num_msgs']
                for item in q:
                    item.on_next(x)

                n[0] += 1
                c = n[0] - skip  # count
                if c >= 0 and c % skip == 0:  # skip
                    s = q.pop(0)
                    s.on_completed()
                    create_window()
                    n[0] = 0

            def on_error(exception):
                while q:
                    q.pop(0).on_error(exception)
                observer.on_error(exception)

            def on_completed():
                while q:
                    q.pop(0).on_completed()
                observer.on_completed()

            m.disposable = source.subscribe_(on_next, on_error, on_completed, scheduler)
            return refCountDisposable

        return Observable(subscribe)

    return window_with_variable_count


def expected_inputs(idx_n, dt_i, dt_n, delay):
    if idx_n == 0:
        return 1
    else:
        # N = idx_n + 1, because idx_n starts at 0
        N_t_min_1 = idx_n
        N_t = idx_n + 1
        # Note: idx_n=1 after initial tick, corresponding to T=0
        # Hence, T_t=dt_n * (idx_n-1), T_t+1=dt_n * idx_n
        sum_t_min_1 = max(0, int((dt_n * (N_t_min_1 - 1) - delay) / dt_i))  # Current timestep
        sum_t = max(0, int((dt_n * (N_t - 1) - delay) / dt_i))  # Next timestep
        return sum_t - sum_t_min_1


def gen_msg(Ir):
    return rx.pipe(ops.map(lambda x: rx.repeat_value(x, x['num_msgs'])),
                   ops.merge(max_concurrent=1),
                   ops.zip(Ir),
                   ops.map(lambda x: dict(x[0], **{'msg': x[1]})),
                   _window_with_variable_count(),
                   ops.flat_map(lambda value: value.pipe(ops.to_iterable(), ops.map(list))),
                   ops.filter(lambda value: len(value) > 0))


def get_repeat_fn(repeat, name):
    if repeat == 'all':
        repeat_fn = lambda x: [dict(x[0], **{'msg': msg[name]}) for msg in x[1]]
    elif repeat == 'empty':
        repeat_fn = lambda x: [dict(x[0], **{'msg': (None, None)})]
    else:
        raise ValueError('Not implemented: %s' % repeat)
    return repeat_fn


def create_channel(ns, Nc, dt_n, inpt, scheduler, is_feedthrough, node: NodeBase):
    if is_feedthrough:
        name = inpt['feedthrough_to']
    else:
        name = inpt['name']

    # Readable format
    Is = inpt['reset']
    Ir = inpt['msg'].pipe(ops.observe_on(scheduler),
                          ops.map(inpt['converter'].convert), ops.share(),
                          ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
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
        print('Probably cannot find key "%s" on ros param server.' % inpt['name'] + '/rate')
        print(e)

    # Create input channel
    num_msgs = Nc.pipe(ops.observe_on(scheduler),
                       ops.start_with(0),
                       ops.map(lambda i: {'node_tick': i, 'num_msgs': expected_inputs(i, dt_i, dt_n, inpt['delay'])}))
    msg = num_msgs.pipe(gen_msg(Ir))
    channel = num_msgs.pipe(ops.filter(lambda x: x['num_msgs'] == 0),
                            ops.with_latest_from(msg),
                            ops.map(get_repeat_fn(inpt['repeat'], 'msg')),
                            ops.merge(msg),
                            regroup_msgs(name, dt_i=dt_i, dt_n=dt_n),
                            ops.share())

    # Create reset flag
    flag = Ir.pipe(ops.map(lambda val: val[0] + 1),
                   ops.start_with(0),
                   ops.combine_latest(Is.pipe(ops.map(lambda msg: msg.data))),  # Depends on ROS reset msg type
                   ops.filter(lambda value: value[0] == value[1]),
                   ops.map(lambda x: {name: x[0]})
                   )
    return channel, flag


def init_channels(ns, Nc, dt_n, inputs, scheduler, node: NodeBase, is_feedthrough=False):
    # Create channels
    channels = []
    flags = []
    for i in inputs:
        channel, flag = create_channel(ns, Nc, dt_n, i, scheduler, node=node, is_feedthrough=is_feedthrough)
        channels.append(channel)
        if is_feedthrough:
            name = i['address']
        else:
            name = i['name']
        flag = flag.pipe(spy('flag [%s]' % name.split('/')[-1][:12].ljust(4), node))
        flags.append(flag)
    zipped_flags = rx.zip(*flags).pipe(ops.map(lambda x: merge_dicts({}, x)))
    zipped_channels = rx.zip(*channels).pipe(regroup_inputs(node, dt_n=dt_n), ops.share())
    return zipped_channels, zipped_flags


def init_real_reset(ns, Nc, dt_n, RR, real_reset, feedthrough, scheduler, node: NodeBase):
    # Create real reset pipeline
    dispose = []
    if real_reset:
        for i in feedthrough:
            rate_str = '%s/rate/%s' % (ns, i['address'][len(ns) + 1:])
            dt_i = 1 / get_param_with_blocking(rate_str)
            if not dt_i == dt_n:
                raise ValueError('Rate of the switch node (%s) must be exactly the same as the feedthrough node rate (%s).' % (dt_n, dt_i))

        # Create zipped action channel
        # todo: in real world implementation, how to deal with action feedthrough? Perhaps not zip, but feedthrough every action individually.
        zipped_channels, zipped_flags = init_channels(ns, Nc, dt_n, feedthrough, scheduler, node, is_feedthrough=True)

        # Create switch subject
        RR_ho = BehaviorSubject(zipped_channels)
        d_RR_ho = RR.pipe(ops.map(lambda x: Nc.pipe(ops.map(lambda x: None), ops.start_with(None)))).subscribe(RR_ho)
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
                          ops.map(lambda val: dict(msg=val)),
                          regroup_msgs(s['name']),)
        channels.append(c)
    return rx.zip(*channels).pipe(regroup_inputs(node))


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
                              ops.map(lambda val: dict(msg=val)),
                              regroup_msgs(s['name']),
                              ops.start_with(dict(name=s['name'], msg=None)),
                              ops.combine_latest(d),
                              ops.filter(lambda x: x[0]['msg'] is not None or x[1]),
                              ops.map(lambda x: merge_dicts(x[0].copy(), {'done': x[1]})))
            channels.append(c)
        return rx.zip(*channels).pipe(regroup_inputs(node), ops.merge(rx.never()))
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
                              ops.map(lambda val: dict(msg=val)),
                              regroup_msgs(s['name']),
                              ops.start_with(dict(name=s['name'], msg=None)),
                              ops.combine_latest(d),
                              ops.filter(lambda x: x[0]['msg'] is not None or x[1]),
                              ops.map(lambda x: merge_dicts(x[0].copy(), {'done': x[1]})))

            done, reset = trigger.pipe(ops.with_latest_from(c),
                                       ops.map(lambda x: x[1]),
                                       ops.partition(lambda x: x['done']))
            reset = reset.pipe(ops.map(lambda x: (x, s['state'].reset(state=x['msg'][0], done=x['done']))),
                               ops.map(lambda x: x[0]))
            rs = rx.merge(done.pipe(spy('done [%s]' % s['name'].split('/')[-1][:12].ljust(4), node)),
                          reset.pipe(spy('reset [%s]' % s['name'].split('/')[-1][:12].ljust(4), node)))

            channels.append(rs)
        return rx.zip(*channels).pipe(regroup_inputs(node), ops.merge(rx.never()))
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
                                   spy('CB_FEED', node, op_log_level=DEBUG),
                                   ops.map(lambda val: cb_ft(val)), ops.share())
        reset_stream = reset_stream.pipe(ops.map(lambda x: x[1][0]),
                                         ops.combine_latest(target_stream),
                                         spy('CB_REAL', node, op_log_level=DEBUG),
                                         ops.map(lambda val: cb_tick(**val[0], **val[1])), ops.share())
        output_stream = rx.merge(reset_stream, ft_stream)

        # Send done flags
        for s in state_outputs:
            d = reset_stream.pipe(spy('reset', node, op_log_level=DEBUG),
                                  ops.pluck(s['name'] + '/done'),
                                  ops.share()).subscribe(s['msg'])

            # Add disposable
            d_msg += [d]
    else:
        output_stream = stream.pipe(ops.filter(lambda x: x[1][1] is None),
                                    ops.map(lambda x: x[1][0]),
                                    spy('CB_TICK', node, op_log_level=DEBUG),
                                    ops.map(lambda val: cb_tick(**val)),
                                    ops.share())

    # Publish output msg as ROS topic and to subjects if single process
    for o in outputs:
        d = output_stream.pipe(ops.pluck(o['name']),
                               ops.map(o['converter'].convert), ops.share(),
                               ops.share(),
                               ).subscribe(o['msg'])

        # Add disposable
        d_msg += [d]

    return d_msg


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
        # If simulation node is not launched locally, it will be launched by the environment. --> can only launch nodes in main thread.
        assert params['single_process'] or not params['launch_locally'], 'Only single_process simulation nodes, or nodes not launched locally are supported.'
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
            assert not params['single_process'] or (params['single_process'] and 'converter' not in i), 'Node "%s" has an output converter (%s) specified. That is currently not supported if launching remotely.' % (name, i['converter'])

            # Create a new input topic for each SimNode output topic
            n = RxInput(name=i['address'], address=i['address'], msg_type=i['msg_type'], msg_module=i['msg_module'],
                        is_reactive=True, repeat='all').get_params()

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


def initialize_reactive_proxy_reset(dt_n, RM, reactive_proxy, node):
    for rx in reactive_proxy:
        if 'disposable' in rx:
            continue
        dt_i = 1 / rx['rate']
        rx['disposable'] = RM.pipe(ops.map(lambda msg: msg.data),
                                   ops.map(lambda idx_n: 1 + int((idx_n - 1) * dt_n / dt_i)),  # We subtract -1 from msg to account for the initial tick.
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
                                   spy('awaiting', node, op_log_level=DEBUG),
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