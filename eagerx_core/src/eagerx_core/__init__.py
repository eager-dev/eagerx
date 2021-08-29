# IMPORT OTHER
from threading import current_thread, Condition
from os import getpid
from functools import wraps
import types
from typing import Callable, Any
from termcolor import colored, cprint
import datetime, traceback

# IMPORT ROS
import rospy
from std_msgs.msg import UInt64, String

# IMPORT RX
import rx
from rx import operators as ops
from rx import create
from rx.core import Observable
from rx.internal.utils import add_ref
from rx.disposable import Disposable, SingleAssignmentDisposable, RefCountDisposable
from rx.subject import Subject, BehaviorSubject, ReplaySubject
from rx.scheduler import ThreadPoolScheduler, CurrentThreadScheduler, ImmediateScheduler, EventLoopScheduler
from eagerx_core.utils.utils import get_attribute_from_module, launch_node, wait_for_node_initialization, get_param_with_blocking


only_node = ['/rx/N1', '/rx/N3', '/rx/obj/nodes/states/orientation', '/rx/obj/nodes/sensors/pos_sensors', '/rx/bridge']

node_color = {'/rx/N1': 'green',
              '/rx/N2': 'white',
              '/rx/N3': 'blue',
              '/rx/N4': 'yellow',
              '/rx/N5': 'white',
              '/rx/obj/nodes/states/orientation': 'red',
              '/rx/obj/nodes/sensors/pos_sensors': 'cyan',
              '/rx/bridge': 'magenta'}


def identity(msg):
    return msg


def UInt64_to_int(msg):
    return int(msg.data)


def cb_ft(cb_input, name='P3'):
    # Perform callback (simply print input)
    if name == 'P3':
        rospy.loginfo('[%s][%s][%s] %s: %s' % (getpid(), current_thread().name, name, 'cb_ft', cb_input))

    # Fill output msg with number of node ticks
    output_msgs = dict()
    for key, item in cb_input.items():
        if key not in ['node_tick', 't_n']:
            output_msgs[key] = item['msg'][0]
    return output_msgs


def spy(id='', node_name='', only_node=only_node, color=None):
    if only_node and not isinstance(only_node, list):
        only_node = [only_node]
    def _spy(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):
                if True:
                    # Get color based on node name
                    if node_name in node_color:
                        color = node_color[node_name]
                    else:
                        color = 'white'

                    # Print item
                    if only_node:
                        if node_name in only_node:
                            cprint('[' + str(getpid())[:5].ljust(5) + ']', color, end='')
                            cprint('[' + current_thread().name.split('/')[-1][:15].ljust(15) + ']', color, end='')
                            cprint('[' + node_name.split('/')[-1][:12].ljust(12) + ']', color, end='', attrs=['bold'])
                            cprint(' %s: %s' % (id, value), color)
                    else:
                        cprint('[' + str(getpid())[:5].ljust(5) + ']', color, end='')
                        cprint('[' + current_thread().name.split('/')[-1][:15].ljust(15) + ']', color, end='')
                        cprint('[' + node_name.split('/')[-1][:12].ljust(12) + ']', color, end='', attrs=['bold'])
                        cprint(' %s: %s' % (id, value), color)
                observer.on_next(value)

            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)

        return rx.create(subscribe)
    return _spy


def trace_observable(prefix,
                     trace_next=False, trace_next_payload=False,
                     trace_subscribe=False,
                     date=None):
    def _trace(source):
        def on_subscribe(observer, scheduler):
            def on_next(value):
                if trace_next is True:
                    if trace_next_payload is True:
                        print("{}:{} - on_next: {}".format(
                            date or datetime.datetime.now(),
                            prefix, value))
                    else:
                        print("{}:{} - on_next".format(
                            date or datetime.datetime.now(),
                            prefix))
                observer.on_next(value)

            def on_completed():
                print("{}:{} - on_completed".format(
                    date or datetime.datetime.now(),
                    prefix))
                observer.on_completed()

            def on_error(error):
                if isinstance(error, Exception):
                    print("{}:{} - on_error: {}, {}".format(
                        date or datetime.datetime.now(),
                        prefix, error,
                        traceback.print_tb(error.__traceback__)))
                else:
                    print("{}:{} - on_error: {}".format(
                        date or datetime.datetime.now(),
                        prefix, error))
                observer.on_error(error)

            def dispose():
                if trace_subscribe is True:
                    print("{}:{} - dispose".format(
                            date or datetime.datetime.now(),
                            prefix))

                disposable.dispose()

            if trace_subscribe is True:
                print("{}:{} - on_subscribe".format(
                        date or datetime.datetime.now(),
                        prefix))
            disposable = source.subscribe(
                on_next=on_next,
                on_error=on_error,
                on_completed=on_completed,
            )
            return Disposable(dispose)
        return rx.create(on_subscribe)
    return _trace


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


def from_topic(topic_type: Any, topic_name: str) -> Observable:
    def _subscribe(observer, scheduler=None) -> Disposable:
        rospy.Subscriber(topic_name, topic_type, lambda msg: observer.on_next(msg))
        return observer
    return create(_subscribe)


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
    """
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
                    node_ticks = [msg['node_tick'] for msg in value]
                    res['node_tick'] = node_ticks[0]
                    if not len(set(node_ticks)) == 1:
                        print('Not all node_ticks are the same: %s' % value)
                    # Check that all timestamps are smaller or equal to node time
                    if dt_n is not None and dt_i is not None:
                        t_n = node_ticks[0] * dt_n
                        t_i = [count * dt_i if count is not None else None for count in counts]
                        res['t_i'] = t_i
                        res['dt_i'] = dt_i
                        if len(t_i) > 0 and not all(t <= t_n for t in t_i if t is not None):
                            print('Not all t_msgs are smaller or equal to t_n: %s' % res)

                # Check that all num_msgs are the same
                if 'num_msgs' in value[0]:
                    num_msgs = [msg['num_msgs'] for msg in value]
                    res['num_msgs'] = num_msgs[0]
                    if not len(set(num_msgs)) == 1:
                        print('Not all num_msgs are the same: %s' % value)
                observer.on_next(res)

            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)

        return rx.create(subscribe)

    return _regroup_msgs


def regroup_inputs(dt_n=None):
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
                    name = I['name']
                    res[name] = I
                    del res[name]['name']

                    # check that num_msg is equal to number of msgs if not zero
                    if 'num_msgs' in I:
                        num_msgs = I['num_msgs']
                        if not (num_msgs > 0 and len(I['msg']) == num_msgs) and num_msgs > 0:
                            print('num_msgs not equal to len(msgs): %s' % I)

                    if 'node_tick' in I:
                        node_ticks.append(I['node_tick'])
                        del res[name]['node_tick']

                # Check that all node_ticks are the same
                if len(node_ticks) > 0:
                    if not len(set(node_ticks)) == 1:
                        print('Not all node_ticks are the same: %s' % value)
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


def expected_inputs(idx_n, dt_i, dt_n):
    # todo: world_stepper: elif indx_n < 0: return 0
    if idx_n == 0:
        return 1
    else:
        sig_i = int((dt_n * idx_n) / dt_i)
        sig_ii = int((dt_n * (idx_n - 1)) / dt_i)
        return sig_i - sig_ii


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


def create_channel(ns, Nc, dt_n, inpt, scheduler, is_feedthrough, node_name=''):
    if is_feedthrough:
        name = inpt['feedthrough_to']
    else:
        name = inpt['name']

    # Readable format
    Is = inpt['reset']
    Ir = inpt['msg'].pipe(ops.observe_on(scheduler),
                          ops.map(inpt['converter']), ops.share(),
                          ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)))

    # Get rate from rosparam server
    try:
        dt_i = 1 / rospy.get_param(inpt['address'] + '/rate')
    except Exception as e:
        print('Probably cannot find key "%s" on ros param server.' % inpt['name'] + '/rate')
        print(e)

    # Create input channel
    num_msgs = Nc.pipe(ops.observe_on(scheduler),
                       ops.map(lambda i: {'node_tick': i, 'num_msgs': expected_inputs(i, dt_i, dt_n)}))  # todo: expected_inputs(i-1, dt_i, dt_n)})
    msg = num_msgs.pipe(gen_msg(Ir))
    channel = num_msgs.pipe(ops.filter(lambda x: x['num_msgs'] == 0),
                            ops.with_latest_from(msg),
                            ops.share(),
                            ops.map(get_repeat_fn(inpt['repeat'], 'msg')),
                            ops.merge(msg),
                            regroup_msgs(name, dt_i=dt_i, dt_n=dt_n),
                            ops.share())

    # Create reset flag
    flag = Ir.pipe(ops.map(lambda val: val[0] + 1),
                   ops.start_with(0),
                   ops.combine_latest(Is.pipe(ops.map(lambda msg: msg.data))),  # Depends on ROS reset msg type
                   ops.filter(lambda value: value[0] == value[1]),
                   )  # .subscribe(flag, scheduler=scheduler)  # todo: scheduler change
    return channel, flag


def init_channels(ns, Nc, dt_n, inputs, scheduler, is_feedthrough=False, node_name=''):
    # Create channels
    channels = []
    flags = []
    for i in inputs:
        channel, flag = create_channel(ns, Nc, dt_n, i, scheduler, is_feedthrough=is_feedthrough, node_name=node_name)
        channels.append(channel)
        if node_name in only_node:
            if is_feedthrough:
                name = i['address']
            else:
                name = i['name']
            flag = flag.pipe(spy('flag [%s]' % name.split('/')[-1][:12].ljust(4), node_name))
        flags.append(flag)
    zipped_channels = rx.zip(*channels).pipe(regroup_inputs(dt_n=dt_n), ops.share())
    return zipped_channels, flags


def init_real_reset(ns, Nc, dt_n, RR, real_reset, feedthrough, scheduler, node_name=''):
    # Create real reset pipeline
    dispose = []
    if real_reset:
        for i in feedthrough:
            dt_i = 1 / rospy.get_param(i['address'] + '/rate')
            if not dt_i == dt_n:
                raise ValueError('Rate of the switch node (%s) must be exactly the same as the feedthrough node rate (%s).' % (dt_n, dt_i))

        # Create zipped action channel
        # todo: in real world implementation, how to deal with action feedthrough? Perhaps not zip, but feedthrough every action individually.
        zipped_channels, flags = init_channels(ns, Nc, dt_n, feedthrough, scheduler, is_feedthrough=True, node_name=node_name)

        # Create switch subject
        RR_ho = BehaviorSubject(zipped_channels)
        d_RR_ho = RR.pipe(ops.map(lambda x: Nc.pipe(ops.map(lambda x: None), ops.start_with(None)))).subscribe(RR_ho)
        rr_channel = RR_ho.pipe(ops.switch_latest())

        # Add disposables
        dispose += [RR_ho, d_RR_ho]
    else:
        # Create switch Subject
        flags = []
        rr_channel = Nc.pipe(ops.map(lambda x: None))

    return rr_channel, flags, dispose


def init_state_channel(states, scheduler, node_name=''):
    channels = []
    for s in states:
        c = s['msg'].pipe(ops.map(s['converter']), ops.share(),
                          ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
                          ops.map(lambda val: dict(msg=val)),
                          regroup_msgs(s['name']),)
        channels.append(c)
    return rx.zip(*channels).pipe(regroup_inputs())


def merge_two_dicts(dict_1, dict_2):
    dict_1.update(dict_2)
    return dict_1


def init_state_inputs_channel(ns, state_inputs, scheduler, node_name=''):
    channels = []
    for s in state_inputs:
        d = s['done'].pipe(ops.map(lambda msg: bool(msg.data)),
                           ops.scan(lambda acc, x: x if x else acc, False))
        c = s['msg'].pipe(ops.map(s['converter']), ops.share(),
                          ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
                          ops.map(lambda val: dict(msg=val)),
                          regroup_msgs(s['name']),
                          ops.combine_latest(d),
                          ops.map(lambda x: merge_two_dicts(x[0].copy(), {'done': x[1]})))
        channels.append(c)
    return rx.zip(*channels).pipe(regroup_inputs())


def init_callback_pipeline(ns, cb_tick, cb_ft, stream, real_reset, state_inputs, state_outputs, outputs, scheduler, node_name=''):
    d_msg = []
    if real_reset:
        state_stream = init_state_channel(state_inputs, scheduler, node_name=node_name)

        # Split stream into feedthrough (ft) and reset stream
        reset_stream, ft_stream = stream.pipe(ops.partition(lambda x: x[1][1] is None))

        # Either feedthrough action or run callback
        ft_stream = ft_stream.pipe(ops.map(lambda x: x[1][1]),
                                   ops.map(lambda val: cb_ft(val)), ops.share())
        reset_stream = reset_stream.pipe(ops.map(lambda x: x[1][0]),
                                         ops.combine_latest(state_stream),
                                         ops.map(lambda val: cb_tick(val)), ops.share())
        output_stream = rx.merge(reset_stream, ft_stream)

        # Send done flags
        for s in state_outputs:
            d = reset_stream.pipe(ops.pluck(s['name'] + '/done'),
                                  publisher_to_topic(s['msg_pub']), ops.share()).subscribe(s['msg'])

            # Add disposable
            d_msg += [d]
    else:
        output_stream = stream.pipe(ops.filter(lambda x: x[1][1] is None),
                                    ops.map(lambda x: x[1][0]),
                                    ops.map(lambda val: cb_tick(val)),
                                    ops.share())

    # Publish output msg as ROS topic and to subjects if single process
    for o in outputs:
        d = output_stream.pipe(ops.pluck(o['name']),
                               ops.map(o['converter']), ops.share(),
                               publisher_to_topic(o['msg_pub']),
                               ops.share()).subscribe(o['msg'])

        # Add disposable
        d_msg += [d]

    return d_msg


def init_node_pipeline(ns, dt_n, cb_tick, inputs, outputs, F, SS, R, RR, real_reset, feedthrough, state_inputs, state_outputs, cb_ft, node_name='', event_scheduler=None, thread_pool_scheduler=None):
    # Node ticks
    Rn = ReplaySubject()  # Reset flag for the node (Nc=Ns and r_signal)
    Nc = BehaviorSubject(0)  # Number completed callbacks (i.e. send Topics): initialized at zero to kick of chain reaction
    Ns = BehaviorSubject(0)  # Number of started callbacks (i.e. number of planned Topics).

    # Create input channels
    zipped_inputs, input_flags = init_channels(ns, Nc, dt_n, inputs, node_name=node_name, scheduler=thread_pool_scheduler)

    # Create action channels
    zipped_feedthrough, action_flags, d_rr = init_real_reset(ns, Nc, dt_n, RR, real_reset, feedthrough, thread_pool_scheduler, node_name=node_name)

    # Zip inputs & action channels
    zipped_channels = rx.zip(zipped_inputs, zipped_feedthrough).pipe(ops.share(), ops.observe_on(event_scheduler))

    # New routine with merge
    PR = R.pipe(ops.observe_on(event_scheduler),
                ops.map(lambda x: True),
                ops.merge(zipped_channels),
                switch_to_reset(),
                ops.share())

    # Create reset signal
    Rr, P = PR.pipe(ops.partition(lambda value: isinstance(value, bool)))

    # Create accumulator: (acc)
    d_Ns = P.pipe(ops.scan(lambda acc, x: acc + 1, 0)).subscribe(Ns)

    # Create callback stream
    input_stream = Ns.pipe(ops.skip(1), ops.zip(P), ops.share())
    d_msg = init_callback_pipeline(ns, cb_tick, cb_ft, input_stream, real_reset, state_inputs, state_outputs, outputs, event_scheduler, node_name=node_name)

    # Publish output msg as ROS topic and to subjects if single process
    Nc_obs = rx.zip(*[o['msg'] for o in outputs]).pipe(ops.scan(lambda acc, x: acc + 1, 0))

    # Increase ticks
    d_Nc = Nc_obs.subscribe(Nc, scheduler=thread_pool_scheduler)
    d_Rn = Nc_obs.pipe(ops.start_with(0),  # added to simulated first zero from BS(0) of Nc
                       ops.combine_latest(Ns, Rr),
                       ops.filter(lambda value: value[0] == value[1]),
                       ).subscribe(Rn)

    # Create reset flags for the set_states (only concerns StateNode)
    if not real_reset:
        ss_flags = init_state_inputs_channel(ns, state_inputs, event_scheduler, node_name=node_name)
    else:
        ss_flags = init_state_inputs_channel(ns, [], event_scheduler, node_name=node_name)
    d_ss_flags = ss_flags.subscribe(SS)

    # Combine flags and subscribe F to all the flags zipped together
    d_flag = rx.zip(*input_flags, *action_flags).subscribe(F)

    # Dispose
    dispose = [Rn, Nc, Ns, d_Rn, d_Nc, d_Ns, d_flag, d_ss_flags] + d_msg + d_rr
    return {'Rn': Rn, 'dispose': dispose}


def init_node(ns, dt_n, cb_tick, cb_reset, inputs, outputs, feedthrough=tuple(), state_inputs=tuple(), node_name='', scheduler=None):
    # Initialize scheduler
    if scheduler is not None:
        event_scheduler = scheduler
        thread_pool_scheduler = event_scheduler
    else:
        event_scheduler = EventLoopScheduler()
        thread_pool_scheduler = event_scheduler
        # thread_count = multiprocessing.cpu_count()
        # thread_pool_scheduler = ThreadPoolScheduler(thread_count)

    # Track node I/O
    node_inputs = []
    node_outputs = []

    # Prepare reset topic
    R = Subject()
    reset_input = dict(name='reset', address=ns + '/reset', msg_type=UInt64, msg=R)
    node_inputs.append(reset_input)

    # Prepare real_reset topic
    RR = Subject()
    real_reset_input = dict(name='real_reset', address=ns + '/real_reset', msg_type=UInt64, msg=RR)
    node_inputs.append(real_reset_input)

    # Real reset checks
    state_outputs = []
    real_reset = len(feedthrough) > 0
    assert (not real_reset or (real_reset and len(state_inputs) > 0)), 'Cannot initialize real reset node (%s). If len(feedthrough) is provided, then len(states_in) > 0. must hold.'

    # Prepare input topics
    for i in inputs:
        # Subscribe to input topic
        Ir = Subject()
        i['msg'] = Ir

        # Subscribe to input reset topic
        Is = Subject()
        i['reset'] = Is

    # Prepare output topics
    for i in outputs:
        # Prepare output topic
        i['msg'] = Subject()
        i['msg_pub'] = rospy.Publisher(i['address'], i['msg_type'], queue_size=0)

        # Initialize reset topic
        i['reset'] = Subject()
        i['reset_pub'] = rospy.Publisher(i['address'] + '/reset', UInt64, queue_size=0)

    # Prepare action topics (used by RealResetNode)
    for i in feedthrough:
        # Subscribe to input topic
        Ar = Subject()
        i['msg'] = Ar

        # Subscribe to input reset topic
        As = Subject()
        i['reset'] = As

    # Prepare state topics (used by RealResetNode)
    for i in state_inputs:
        # Initialize desired state message
        S = Subject()
        i['msg'] = S

        D = Subject()
        i['done'] = D
        if real_reset:  # used by RealResetNode
            # Initialize done flag for desired state
            done_pub = rospy.Publisher(i['address'] + '/done', UInt64, queue_size=0)
            done_output = dict(name=i['name'], address=i['address'] + '/done', msg_type=UInt64, msg=D, msg_pub=done_pub)
            state_outputs.append(done_output)

    # Flags
    flags = []
    for i in inputs + feedthrough:
        flag = i['reset'].pipe(ops.map(lambda x: x.data), ops.first(), ops.merge(rx.never()))
        flags.append(flag)
    F_init = rx.zip(*flags)

    # Reset flags
    F = Subject()
    f = rx.merge(F, F_init)
    SS = BehaviorSubject(None)

    # Node ticks
    Rn_ho = BehaviorSubject(BehaviorSubject((0, 0, True)))
    Rn = Rn_ho.pipe(ops.switch_latest())

    # Create reset switch_latest
    Rr = R.pipe(ops.map(lambda x: True))  # ops.observe_on(event_scheduler),  # seemed to make ROS variant crash

    # Reset node pipeline
    Nc_reset = rx.zip(Rn.pipe(spy('Rn', node_name, only_node=only_node)),
                      Rr.pipe(spy('Rr', node_name, only_node=only_node)),
                      f.pipe(spy('F', node_name, only_node=only_node))).pipe(spy('flags_zipped', node_name, only_node=only_node),
                                                                              ops.map(lambda x: x[0][0]), ops.share())  # x[0][0]=Nc
    reset_obs = Nc_reset.pipe(ops.map(lambda x: init_node_pipeline(ns, dt_n, cb_tick, inputs, outputs, F, SS, R, RR, real_reset, feedthrough, state_inputs, state_outputs, cb_ft, node_name=node_name, event_scheduler=event_scheduler, thread_pool_scheduler=thread_pool_scheduler)),
                              trace_observable('[trace callback]'),
                              ops.share())
    reset_obs.pipe(ops.pluck('Rn')).subscribe(Rn_ho)

    # Dispose old pipeline, run reset callback & send reset topic
    reset_msg = reset_obs.pipe(ops.pluck('dispose'),
                              ops.buffer_with_count(2, skip=1),
                              ops.map(lambda x: [d.dispose() for d in x[0]]),
                              ops.start_with(None),
                              ops.zip(Nc_reset, Rn_ho.pipe(ops.skip(1))),  # zipped with Rn_ho so that Rn has switched before sending "reset topic"
                              ops.map(lambda x: x[1]),  # x[1]=Nc
                              ops.with_latest_from(SS),  # todo: could lead to resets with old states # If we have set_states (StateNode), then append desired states & done flags
                              ops.map(lambda x: (x[0], cb_reset(x))),
                              trace_observable('[trace callback]'),
                              ops.map(lambda x: UInt64(data=x[0])),
                              ops.share())

    [reset_msg.pipe(publisher_to_topic(o['reset_pub'])).subscribe(o['reset']) for o in outputs]

    node_inputs = (reset_input, real_reset_input)
    rx_objects = dict(inputs=inputs, outputs=outputs, feedthrough=feedthrough, state_inputs=state_inputs, state_outputs=state_outputs, node_inputs=node_inputs)
    return rx_objects


def init_bridge_pipeline(ns, dt_n, cb_tick, inputs, outputs, F, DF, RR, done_flags, node_name='', event_scheduler=None, thread_pool_scheduler=None):
    # todo: Subjects that accumulate state registries: resettable_sim, resettable_real
    # todo: Subject that accumulate linkable outputs
    # todo:

    # Get non-registered real_reset states from ROS param server
    df_dict = rospy.get_param(ns + '/real_reset', dict())
    for key, item in df_dict.items():
        name = key.replace('.', '/')
        if key not in done_flags.keys():
            new_df = dict()
            new_df['name'] = name
            new_df['msg_type'] = UInt64
            # todo: Move all ROS related subscriptions to init_communication(...).
            new_df['done'] = from_topic(new_df['msg_type'], name).pipe(ops.map(lambda x: bool(x.data)), ops.filter(lambda x: x))
            done_flags[name] = new_df

    # Zip together all done flags
    dfs = [rx.of(None).pipe(ops.merge(rx.never()))]
    for key, item in done_flags.items():
        dfs.append(item['done'])
    rx.zip(*dfs).subscribe(DF)

    # Node ticks
    Rn = ReplaySubject()  # Reset flag for the node (Nc=Ns and r_signal)
    Nc = BehaviorSubject(0)  # Number completed callbacks (i.e. send Topics): initialized at zero to kick of chain reaction
    Ns = BehaviorSubject(0)  # Number of started callbacks (i.e. number of planned Topics).

    # Create input channels
    zipped_channels, input_flags = init_channels(ns, Nc, dt_n, inputs, node_name=node_name, scheduler=thread_pool_scheduler)

    # New routine with merge
    # todo: Change RR to version zipped with DF
    PR = rx.zip(RR, DF).pipe(ops.observe_on(event_scheduler),
                             ops.map(lambda x: True),
                             ops.merge(zipped_channels),
                             switch_to_reset(),
                             ops.share())

    # Create reset signal
    Rr, P = PR.pipe(ops.partition(lambda value: isinstance(value, bool)))

    # Create accumulator: (acc)
    d_Ns = P.pipe(ops.scan(lambda acc, x: acc + 1, 0)).subscribe(Ns)

    # Create callback stream
    input_stream = Ns.pipe(ops.skip(1), ops.zip(P), ops.share())
    d_msg = init_callback_pipeline(ns, cb_tick, cb_ft, input_stream, False, tuple(), tuple(), outputs, event_scheduler,
                                   node_name=node_name)

    # After outputs have been send, increase the completed callback counter
    Nc_obs = rx.zip(*[o['msg'] for o in outputs]).pipe(ops.scan(lambda acc, x: acc + 1, 0))

    # Increase ticks
    d_Nc = Nc_obs.subscribe(Nc, scheduler=thread_pool_scheduler)
    d_Rn = Nc_obs.pipe(ops.start_with(0),  # added to simulated first zero from BS(0) of Nc
                       ops.combine_latest(Ns, Rr),
                       ops.filter(lambda value: value[0] == value[1]),
                       ).subscribe(Rn)

    # Combine flags and subscribe F to all the flags zipped together
    d_flag = rx.zip(*input_flags).subscribe(F)

    # Dispose
    dispose = [Rn, Nc, Ns, d_Rn, d_Nc, d_Ns, d_flag] + d_msg
    return {'Rn': Rn, 'dispose': dispose}


def get_object_params(msg):
    obj_name = msg.data

    obj_params = get_param_with_blocking(obj_name)
    if obj_params is None:
        rospy.logwarn('Parameters for object registry request (%s) not found on parameter server. Timeout: object (%s) not registered.' % (msg.data, msg.data))
        return None

    # Get parameters from ROS param server
    params = []
    for node_name in obj_params['node_names']:
        node_params = get_param_with_blocking(node_name)
        params.append(node_params)
    return params


# todo: launch node with bridge.register() callback
def extract_topics_in_reactive_proxy(obj_params, sp_nodes, launch_nodes):
    inputs = []
    reactive_proxy = []
    for node_params in obj_params:
        name = node_params['name']

        for i in node_params['topics_out']:
            assert not node_params['single_process'] or (node_params['single_process'] and i['converter'] == 'identity'), 'Node "%s" has a non-identity output converter (%s) that is not supported if launching remotely.' % (name, i['converter'])

            # Convert to classes
            i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
            i['converter'] = get_attribute_from_module(i['converter_module'], i['converter'])

            # Initialize rx objects
            i['msg'] = Subject()  # Ir
            i['reset'] = Subject()  # Is

            # Create a new input topic for each SimNode output topic
            new_input = dict()
            new_input.update(i)
            new_input.pop('rate')
            new_input['is_reactive'] = True
            new_input['repeat'] = 'empty'
            inputs.append(new_input)

        for i in node_params['topics_in']:
            if not i['is_reactive']:
                # Convert to classes
                i['msg_type'] = get_attribute_from_module(i['msg_module'], i['msg_type'])
                i['converter'] = get_attribute_from_module(i['converter_module'], i['converter'])

                # Initialize rx reset output for reactive input
                i['reset'] = Subject()
                i['reset_pub'] = rospy.Publisher(i['address'] + '/reset', UInt64, queue_size=0)

                # Create a new output topic for each SimNode reactive input (bridge sends reset msg)
                # todo: reactive inputs must have a rate defined
                new_out = dict()
                new_out.update(i)
                new_out.pop('repeat')
                reactive_proxy.append(new_out)

    return dict(inputs=inputs, reactive_proxy=reactive_proxy, sp_nodes=sp_nodes, launch_nodes=launch_nodes)


def init_bridge(ns, dt_n, cb_tick, cb_pre_reset, cb_post_reset, cb_register_object, inputs, outputs, message_broker, node_name='', scheduler=''):
    # todo: Have message_broker as input.
    #  0) Before calling RR, ensure that all objects & custom nodes have been registered & Initialized (before connect_io is called).
    #  1) Somewhere in the rxbridge reset procedure, call mb.connect_io().
    #  2) When to add '/rx/resettable/real' flags to DF? Directly when register request comes in? Or only after they have been connected?
    #  3) When to add '/rx/inputs/reset' to F. Directly when register request comes in? Or only after they have been connected?
    #  4) When to add '/rx/non_reactive/reset' to F. Directly when register request comes in? Or only after they have been connected?
    # todo:
    #  1) Add flags directly to pipeline locations when register request comes in for all the above.
    #  1) Also, aggregate a list of inputs to be used when initializing bridge_pipeline
    #  1) Also, aggregate a list of outputs to be used when sending '/rx/non_reactive/reset' messages.
    #  2) Then, have env send /rx/start_reset --> triggers mb.connect_io().
    #  3) In this way, we are sure that all communication has been connected, before starting real_reset (so we don't miss done flags).
    #  4)
    # Initialize scheduler
    if scheduler is not None:
        event_scheduler = scheduler
        thread_pool_scheduler = event_scheduler
    else:
        event_scheduler = EventLoopScheduler()
        thread_pool_scheduler = event_scheduler

    # Prepare input topics
    for i in inputs:
        # Subscribe to input topic
        Ir = Subject()
        i['msg'] = Ir

        # Subscribe to input reset topic
        Is = Subject()
        i['reset'] = Is

    # Track node I/O
    node_inputs = []
    node_outputs = []

    # Object register (to dynamically add input reset flags to F for reset)
    OR = Subject()
    object_registry = dict()
    object_registry['address'] = ns + '/register'
    object_registry['msg'] = OR
    object_registry['msg_type'] = String
    node_inputs.append(object_registry)

    # Object registry pipeline
    # todo: call message_broker.connect_io()
    DF = BehaviorSubject(None)
    params_nodes = OR.pipe(ops.map(get_object_params),
                           ops.filter(lambda params: params is not None),
                           ops.map(lambda params: (params,) + cb_register_object(params)))
    rx_objects = params_nodes.pipe(ops.map(lambda i: extract_topics_in_reactive_proxy(*i)),
                                   ops.map(lambda i: (i, message_broker.add_rx_objects(node_name, inputs=i['inputs'], reactive_proxy=i['reactive_proxy']))),
                                   ops.map(lambda i: i[0]),
                                   ops.scan(combine_dict, dict(inputs=inputs, reactive_proxy=[], sp_nodes=[], launch_nodes=[])), ops.share())

    # todo: output reactive/reset
    reactive_out = rx_objects.pipe(ops.pluck('reactive_out'))

    # Zip initial input flags
    # todo: how to verify that we have switched?
    # todo: currently, F_init will not receive anything if no objects are registered i.e. it will block.
    # todo: After having switched, and have updated "with_latest(inputs),
    F_init_ho = Subject()
    F_init = F_init_ho.pipe(ops.switch_latest(), ops.first(), ops.merge(rx.never()), spy('F_init', node_name))
    inputs = rx_objects.pipe(ops.pluck('inputs'))
    inputs.pipe(ops.map(lambda inputs: rx.zip(*[i['reset'] for i in inputs]))).subscribe(F_init_ho)
    F = Subject()
    f = rx.merge(F, F_init)

    # Resettable real states (to dynamically add state/done flags to DF for RR)
    # todo: track state_done flags for RR --> make sure to pass them to message_broker...
    RRS = ReplaySubject()  #todo: Replay needed?
    resettable_real = dict()
    resettable_real['address'] = ns + '/resettable/real'
    resettable_real['msg'] = RRS
    resettable_real['msg_type'] = String
    node_inputs.append(resettable_real)

    # Prepare start_reset input
    SR = Subject()
    start_reset_input = dict()
    start_reset_input['address'] = ns + '/start_reset'
    start_reset_input['msg'] = SR
    start_reset_input['msg_type'] = UInt64
    node_inputs.append(start_reset_input)

    # Prepare real_reset output
    RR = Subject()
    real_reset_output = dict()
    real_reset_output['address'] = ns + '/real_reset'
    real_reset_output['msg'] = RR
    real_reset_output['msg_type'] = UInt64
    real_reset_output['msg_pub'] = rospy.Publisher(real_reset_output['address'], real_reset_output['msg_type'], queue_size=0)
    node_outputs.append(real_reset_output)

    # Prepare reset output
    reset_output = dict()
    R = Subject()
    reset_output['address'] = ns + '/reset'
    reset_output['msg'] = R
    reset_output['msg_type'] = UInt64
    reset_output['msg_pub'] = rospy.Publisher(reset_output['address'], reset_output['msg_type'], queue_size=0)
    node_outputs.append(reset_output)

    # Prepare output topics
    for i in outputs:
        # Prepare output topic
        i['msg'] = Subject()
        i['msg_pub'] = rospy.Publisher(i['address'], i['msg_type'], queue_size=0)

        # Initialize reset topic
        i['reset'] = Subject()
        i['reset_pub'] = rospy.Publisher(i['address'] + '/reset', UInt64, queue_size=0)

    # Real reset routine
    # todo: cut-off tick cb when RRr is received, instead of Rr
    done_flags = dict()
    # DF = BehaviorSubject(None)
    RRn_ho = BehaviorSubject(BehaviorSubject((0, 0, True)))
    RRn = RRn_ho.pipe(ops.switch_latest())
    RRr = RR.pipe(ops.map(lambda x: True))
    reset_msg = rx.zip(RRn.pipe(spy('RRn', node_name)),
                       RRr.pipe(spy('RRr', node_name)),
                       DF.pipe(spy('DF', node_name))
                       ).pipe(ops.map(lambda x: x[0][0]),  # x[0][0]=Nc
                              ops.map(lambda x: (x, cb_pre_reset(x))),  # Run pre-reset callback
                              trace_observable('[trace callback]'),
                              ops.map(lambda x: UInt64(data=x[0])),  # Create reset message
                              ops.share())

    # Send reset messages for all outputs (now, only '/rx/bridge/tick')
    # todo: also publish reset messages for non_reactive inputs
    [reset_msg.pipe(publisher_to_topic(o['reset_pub'])).subscribe(o['reset']) for o in outputs]

    # Send reset message
    reset_msg.pipe(publisher_to_topic(reset_output['msg_pub'])).subscribe(R)
    Rr = R.pipe(ops.map(lambda x: True))

    # Flags
    reset_obs = rx.zip(f.pipe(spy('F', node_name)),
                       Rr.pipe(spy('Rr', node_name))
                       ).pipe(ops.map(lambda x: init_bridge_pipeline(ns, dt_n, cb_tick, inputs, outputs, F, DF, RR, done_flags, node_name=node_name, event_scheduler=event_scheduler, thread_pool_scheduler=thread_pool_scheduler)),
                              trace_observable('[trace callback]'), ops.share())
    reset_obs.pipe(ops.pluck('Rn')).subscribe(RRn_ho)

    # todo: send out initial tick after reset has finished.
    # todo: DOES NOT RUN YET, BECAUSE NO SUBSCRIBERS!
    init_tick = reset_obs.pipe(ops.pluck('dispose'),
                               ops.buffer_with_count(2, skip=1),
                               ops.map(lambda x: [d.dispose() for d in x[0]]),
                               ops.start_with(None),
                               ops.zip(RRn_ho.pipe(ops.skip(1))),
                               ops.map(lambda x: cb_post_reset()),
                               trace_observable('[trace callback]'),
                               ops.share()).subscribe(lambda x: None)

    # node_inputs = (real_reset_input,)
    rx_objects = dict(inputs=[], outputs=outputs, node_inputs=node_inputs, node_outputs=node_outputs)
    return rx_objects


def thread_safe_wrapper(func, condition):
    @wraps(func)
    def wrapped(*args, **kwargs):
        with condition:
            return func(*args, **kwargs)
    return wrapped


class RxMessageBroker(object):
    def __init__(self, owner):
        self.owner = owner

        # Ensure that we are not reading and writing at the same time.
        self.cond = Condition()

        # Structured as outputs[address][node_name] = {rx=Subject, node_name=node_name, source=RxOutput(...), etc..}
        self.rx_connectable = dict()

        # Structured as node_io[node_name][type][address] = {rx=Subject, disposable=rx_disposable, etc..}
        self.node_io = dict()
        self.disconnected = dict()
        self.connected_ros = dict()
        self.connected_rx = dict()

    # Every method is wrapped in a 'with Condition' block in order to be threadsafe
    def __getattribute__(self, name):
        attr = super(RxMessageBroker, self).__getattribute__(name)
        if type(attr)==types.MethodType:
            attr = thread_safe_wrapper(attr, self.cond)
        return attr

    def add_rx_objects(self, node_name, node=None, inputs=tuple(), outputs=tuple(), feedthrough=tuple(),
                       state_inputs=tuple(), state_outputs=tuple(), node_inputs=tuple(), node_outputs=tuple(),
                       reactive_proxy=tuple()):
        # Only add outputs that we would like to link with rx (i.e., skipping ROS (de)serialization)
        for i in outputs:
            assert i['address'] not in self.rx_connectable, 'Non-unique output (%s). All output names must be unique.' % i['address']
            self.rx_connectable[i['address']] = dict(rx=i['msg'], source=i, node_name=node_name, rate=i['rate'])

        # Register all I/O of node
        # todo: add other 'node_outputs' (e.g. '/rx/resettable/sim').
        if node_name not in self.node_io:
            assert node is not None, 'No reference to Node "%s" was provided, during the first attempt to register it.'
            # Prepare io dictionaries
            self.node_io[node_name] = dict(node=node, inputs={}, outputs={}, feedthrough={}, state_inputs={}, state_outputs={}, node_inputs={}, node_outputs={}, reactive_proxy={})
            self.disconnected[node_name] = dict(inputs={}, feedthrough={}, state_inputs={}, node_inputs={})
            self.connected_ros[node_name] = dict(inputs={}, feedthrough={}, state_inputs={}, node_inputs={})
            self.connected_rx[node_name] = dict(inputs={}, feedthrough={}, state_inputs={}, node_inputs={})
        n = dict(inputs={}, outputs={}, feedthrough={}, state_inputs={}, state_outputs={}, node_inputs={}, node_outputs={}, reactive_proxy={})
        for i in inputs:
            address = i['address']
            assert address not in self.node_io[node_name]['inputs'], 'Cannot re-register the same address (%s) twice as "%s".' % (address, 'inputs')
            n['inputs'][address] = {'rx': i['msg'], 'disposable': None, 'source': i, 'msg_type': i['msg_type'], 'converter': i['converter'], 'repeat': i['repeat'], 'status': 'disconnected'}
            n['inputs'][address + '/reset'] = {'rx': i['reset'], 'disposable': None, 'source': i, 'msg_type': UInt64, 'status': 'disconnected'}
        for i in outputs:
            address = i['address']
            assert address not in self.node_io[node_name]['outputs'], 'Cannot re-register the same address (%s) twice as "%s".' % (address, 'outputs')
            n['outputs'][address] = {'rx': i['msg'], 'disposable': None, 'source': i, 'msg_type': i['msg_type'], 'rate': i['rate'], 'converter': i['converter'], 'status': ''}
            n['outputs'][address + '/reset'] = {'rx': i['reset'], 'disposable': None, 'source': i, 'msg_type': UInt64, 'status': ''}
        for i in feedthrough:
            address = i['address']
            assert address not in self.node_io[node_name]['feedthrough'], 'Cannot re-register the same address (%s) twice as "%s".' % (address, 'feedthrough')
            n['feedthrough'][address] = {'rx': i['msg'], 'disposable': None, 'source': i, 'msg_type': i['msg_type'], 'converter': i['converter'], 'repeat': i['repeat'], 'status': 'disconnected'}
            n['feedthrough'][address + '/reset'] = {'rx': i['reset'], 'disposable': None, 'source': i, 'msg_type': UInt64, 'status': 'disconnected'}
        for i in state_outputs:
            address = i['address']
            assert address not in self.node_io[node_name]['state_outputs'], 'Cannot re-register the same address (%s) twice as "%s".' % (address, 'state_outputs')
            n['state_outputs'][address] = {'rx': i['msg'], 'disposable': None, 'source': i, 'msg_type': i['msg_type'], 'status': ''}
        for i in state_inputs:
            address = i['address']
            assert address not in self.node_io[node_name]['state_inputs'], 'Cannot re-register the same address (%s) twice as "%s".' % (address, 'state_inputs')
            n['state_inputs'][address + '/set'] = {'rx': i['msg'], 'disposable': None, 'source': i, 'msg_type': i['msg_type'], 'converter': i['converter'], 'status': 'disconnected'}
            if address + '/done' not in n['state_outputs'].keys():
                n['state_inputs'][address + '/done'] = {'rx': i['done'], 'disposable': None, 'source': i, 'msg_type': UInt64, 'status': 'disconnected'}
        for i in node_inputs:
            address = i['address']
            assert address not in self.node_io[node_name]['node_inputs'], 'Cannot re-register the same address (%s) twice as "%s".' % (address, 'node_inputs')
            n['node_inputs'][address] = {'rx': i['msg'], 'disposable': None, 'source': i, 'msg_type': i['msg_type'], 'status': 'disconnected'}
        for i in node_outputs:
            address = i['address']
            assert address not in self.node_io[node_name]['node_outputs'], 'Cannot re-register the same address (%s) twice as "%s".' % (address, 'node_outputs')
            n['node_outputs'][address] = {'rx': i['msg'], 'disposable': None, 'source': i, 'msg_type': i['msg_type'], 'status': ''}
        for i in reactive_proxy:
            address = i['address']
            assert address not in self.node_io[node_name]['reactive_proxy'], 'Cannot re-register the same address (%s) twice as "%s".' % (address, 'reactive_proxy')
            n['reactive_proxy'][address + '/reset'] = {'rx': i['reset'], 'disposable': None, 'source': i,  'msg_type': UInt64, 'rate': i['rate'], 'status': ''}

        # Add new addresses to already registered I/Os
        for key in n.keys():
            self.node_io[node_name][key].update(n[key])

        # Add new addresses to disconnected
        for key in ('inputs', 'feedthrough', 'state_inputs', 'node_inputs'):
            self.disconnected[node_name][key] = n[key].copy()

    def print_io_status(self, node_names=None):
        # Only print status for specific node
        if node_names is None:
            node_names = self.node_io.keys()
        else:
            if not isinstance(node_names, list):
                node_names = list(node_names)

        # Print status
        for node_name in node_names:
            cprint(('OWNER "%s"' % self.owner).ljust(15, ' ') + ('| OVERVIEW NODE "%s" ' % node_name).ljust(140, " "), attrs=['bold', 'underline'])
            for key in ('inputs', 'feedthrough', 'state_inputs', 'node_inputs', 'outputs', 'state_outputs', 'node_outputs', 'reactive_proxy'):
                if len(self.node_io[node_name][key]) == 0:
                    continue
                for address in self.node_io[node_name][key].keys():
                    color = None
                    if key in ('outputs', 'node_outputs', 'state_outputs', 'reactive_proxy'):
                        color = 'cyan'
                    else:
                        if address in self.disconnected[node_name][key]:
                            color = 'red'
                        if address in self.connected_rx[node_name][key]:
                            assert color is None, 'Duplicate connection status for address (%s).' % address
                            color = 'green'
                        if address in self.connected_ros[node_name][key]:
                            assert color is None, 'Duplicate connection status for address (%s).' % address
                            color = 'blue'
                    status = self.node_io[node_name][key][address]['status']

                    # Print status
                    entry = self.node_io[node_name][key][address]
                    key_str = ('%s' % key).ljust(15, ' ')
                    address_str = ('| %s ' % address).ljust(40, ' ')
                    msg_type_str = ('| %s ' % entry['msg_type'].__name__).ljust(10, ' ')
                    if 'converter' in entry:
                        converter_str = ('| %s ' % entry['converter'].__name__).ljust(17, ' ')
                    else:
                        converter_str = ('| %s ' % '').ljust(17, ' ')
                    if 'repeat' in entry:
                        repeat_str = ('| %s ' % entry['repeat']).ljust(8, ' ')
                    else:
                        repeat_str = ('| %s ' % '').ljust(8, ' ')
                    if 'rate' in entry:
                        rate_str = '|' + ('%s' % entry['rate']).center(3, ' ')
                    else:
                        rate_str = '|' + ''.center(3, ' ')
                    status_str = ('| %s' % status).ljust(60, ' ')

                    log_msg = key_str + rate_str + address_str + msg_type_str + converter_str + repeat_str + status_str
                    cprint(log_msg, color)
            print(' '.center(140, " "))

    def connect_io(self, print_status=True):
        for node_name, node in self.disconnected.items():
            # Skip if no disconnected addresses
            num_disconnected = 0
            for key, addresses in node.items():
                num_disconnected += len(addresses)
            if num_disconnected == 0:
                continue

            # Else, initialize connection
            print_status and cprint(('OWNER "%s"' % self.owner).ljust(15, ' ') + ('| CONNECTING NODE "%s" ' % node_name).ljust(140, " "), attrs=['bold', 'underline'])
            for key, addresses in node.items():
                for address in list(addresses.keys()):
                    entry = addresses[address]
                    assert address not in self.connected_rx[node_name][key], 'Address (%s) of this node (%s) already connected via rx.' % (address, node_name)
                    assert address not in self.connected_ros[node_name][key], 'Address (%s) of this node (%s) already connected via ROS.' % (address, node_name)
                    if address in self.rx_connectable.keys():
                        color = 'green'
                        status = 'Rx'.ljust(4, ' ')
                        rate_str = '|' + ('%s' % self.rx_connectable[address]['rate']).center(3, ' ')
                        node_str = ('| %s' % self.rx_connectable[address]['node_name']).ljust(12, ' ')
                        msg_type_str = ('| %s' % self.rx_connectable[address]['source']['msg_type'].__name__).ljust(12, ' ')
                        converter_str = ('| %s' % self.rx_connectable[address]['source']['converter'].__name__).ljust(12, ' ')
                        status += node_str + msg_type_str + converter_str
                        self.connected_rx[node_name][key][address] = entry
                        O = self.rx_connectable[address]['rx']
                    else:
                        color = 'blue'
                        status = 'ROS |'.ljust(5, ' ')
                        rate_str = '|' + ''.center(3, ' ')
                        msg_type = entry['msg_type']
                        self.connected_ros[node_name][key][address] = entry
                        O = from_topic(msg_type, address)

                    # Subscribe and change status
                    entry['disposable'] = O.subscribe(entry['rx'])
                    entry['status'] = status

                    # Print status
                    key_str = ('%s' % key).ljust(15, ' ')
                    address_str = ('| %s ' % address).ljust(40, ' ')
                    msg_type_str = ('| %s ' % entry['msg_type'].__name__).ljust(10, ' ')
                    status_str = ('| Connected via %s' % status).ljust(60, ' ')

                    if 'converter' in entry:
                        converter_str = ('| %s ' % entry['converter'].__name__).ljust(17, ' ')
                    else:
                        converter_str = ('| %s ' % '').ljust(17, ' ')
                    if 'repeat' in entry:
                        repeat_str = ('| %s ' % entry['repeat']).ljust(8, ' ')
                    else:
                        repeat_str = ('| %s ' % '').ljust(8, ' ')

                    log_msg = key_str + rate_str + address_str + msg_type_str + converter_str + repeat_str + status_str
                    print_status and cprint(log_msg, color)

                    # Remove address from disconnected
                    addresses.pop(address)

            print_status and print(''.center(140, " "))
