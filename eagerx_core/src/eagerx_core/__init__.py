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
from rx.scheduler import ThreadPoolScheduler, EventLoopScheduler, ImmediateScheduler
from eagerx_core.utils.utils import get_attribute_from_module, launch_node, wait_for_node_initialization, get_param_with_blocking

selected_nodes = ['/rx/bridge',
                  '/rx/N1',
                  '/rx/N2',
                  '/rx/N3',
                  '/rx/N4',
                  '/rx/N5',
                  '/rx/obj/nodes/sensors/N6',
                  '/rx/obj/nodes/actuators/N7',
                  '/rx/obj/nodes/states/N8',
                  '/rx/obj/nodes/states/N9',
                  ]

node_color = {'/rx/bridge': 'magenta',
              '/rx/N1': 'white',
              '/rx/N2': 'white',
              '/rx/N3': 'blue',
              '/rx/N4': 'white',
              '/rx/N5': 'white',
              '/rx/obj/nodes/sensors/N6': 'cyan',
              '/rx/obj/nodes/actuators/N7': 'green',
              '/rx/obj/nodes/states/N9': 'yellow',
              '/rx/obj/nodes/states/N8': 'yellow',
              }


def identity(msg):
    return msg


def UInt64_to_int(msg):
    return int(msg.data)


def cb_ft(cb_input, name='P3'):
    # Fill output msg with number of node ticks
    output_msgs = dict()
    for key, item in cb_input.items():
        if key not in ['node_tick', 't_n']:
            output_msgs[key] = item['msg'][0]
    return output_msgs


def print_info(node_name, color, id=None, trace_type=None, value=None, date=None):
    if date:
        cprint('[' + str(date)[:40].ljust(20) + ']', color, end='')
    cprint('[' + str(getpid())[:5].ljust(5) + ']', color, end='')
    cprint('[' + current_thread().name.split('/')[-1][:15].ljust(15) + ']', color, end='')
    cprint('[' + node_name.split('/')[-1][:12].ljust(12) + ']', color, end='', attrs=['bold'])
    if id:
        cprint('[' + id.split('/')[-1][:12].ljust(12) + ']', color, end='')
    cprint((' %s: %s' % (trace_type, value))[:160].ljust(160), color)


def spy(id='', node_name='', only_node=None, color=None):
    if only_node is None:
        only_node = selected_nodes
    # else:
    #     print(id)
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
                    if node_name in only_node:
                        print_info(node_name, color, id, trace_type='', value=str(value))
                observer.on_next(value)

            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler)

        return rx.create(subscribe)
    return _spy


def trace_observable(id, node_name, trace_next=False, trace_next_payload=False, trace_subscribe=False, date=None):
    def _trace(source):
        color_err = 'red'
        color_info = 'white'

        def on_subscribe(observer, scheduler):
            def on_next(value):
                if trace_next is True:
                    if trace_next_payload is True:
                        print_info(node_name, color_info, id, 'on_next', value, date=date or datetime.datetime.now())
                    else:
                        print_info(node_name, color_info, id, 'on_next', '', date=date or datetime.datetime.now())
                observer.on_next(value)

            def on_completed():
                value = ''
                print_info(node_name, color_info, id, 'on_completed', value, date=date or datetime.datetime.now())
                observer.on_completed()

            def on_error(error):
                if isinstance(error, Exception):
                    error_traceback = '%s, %s' % (error, traceback.print_tb(error.__traceback__))
                    print_info(node_name, color_err, id, 'on_error', error_traceback, date=date or datetime.datetime.now())
                else:
                    print_info(node_name, color_info, id, 'on_error', error, date=date or datetime.datetime.now())
                observer.on_error(error)

            def dispose():
                if trace_subscribe is True:
                    value = ''
                    print_info(node_name, color_info, id, 'dispose', value, date=date or datetime.datetime.now())
                disposable.dispose()

            if trace_subscribe is True:
                value = ''
                print_info(node_name, color_info, id, 'on_subscribe', value, date=date or datetime.datetime.now())
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


def regroup_inputs(node_name, dt_n=None):
    def _regroup_inputs(source):
        def subscribe(observer, scheduler=None):
            def on_next(value):
                # if node_name == '/rx/bridge':
                #     print(value)
                #     print('')

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
                        print_info(node_name, 'red', 'regroup_inputs', trace_type='', value='Not all node_ticks are the same: %s' % str(value))
                # assert len(node_ticks) == 0 or len(set(node_ticks)) == 1, 'Not all node_ticks are the same: %s' % str(value)
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
                          ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
                          # ops.share(),  # todo: add? perhaps efficient and worked kind-off with create_channel_bridge
                          # spy('data [%s]' % name, node_name, only_node=['/rx/bridge']),
                          )

    # Get rate from rosparam server
    try:
        dt_i = 1 / rospy.get_param(inpt['address'] + '/rate')
    except Exception as e:
        print('Probably cannot find key "%s" on ros param server.' % inpt['name'] + '/rate')
        print(e)

    # Create input channel
    num_msgs = Nc.pipe(ops.observe_on(scheduler),
                       ops.start_with(0),  # todo: ADDED FOR DYNAMIC BRIDGE PIPELINE
                       ops.map(lambda i: {'node_tick': i, 'num_msgs': expected_inputs(i, dt_i, dt_n)}))  # todo: expected_inputs(i-1, dt_i, dt_n)})
    msg = num_msgs.pipe(gen_msg(Ir))
    channel = num_msgs.pipe(ops.filter(lambda x: x['num_msgs'] == 0),
                            ops.with_latest_from(msg),
                            ops.share(),  # todo: really needed with share at the end here?
                            ops.map(get_repeat_fn(inpt['repeat'], 'msg')),
                            ops.merge(msg),
                            regroup_msgs(name, dt_i=dt_i, dt_n=dt_n),
                            ops.share())

    # Create reset flag
    flag = Ir.pipe(# ops.observe_on(scheduler),
                   ops.map(lambda val: val[0] + 1),
                   ops.start_with(0),
                   # spy('Ir_%s' % name, node_name, only_node='/rx/obj/nodes/actuators/N7'),
                   ops.combine_latest(Is.pipe(ops.map(lambda msg: msg.data))),  # Depends on ROS reset msg type
                   # spy('Ir_Is_%s' % name, node_name, only_node='/rx/bridge'),
                   # spy('Ir_Is_%s' % name, node_name, only_node='/rx/obj/nodes/actuators/N7'),
                   ops.filter(lambda value: value[0] == value[1]),
                   ops.map(lambda x: {name: x[0]})
                   )  # .subscribe(flag, scheduler=scheduler)  # todo: scheduler change
    return channel, flag


def create_channel_bridge(ns, Nc, dt_n, inpt, scheduler, is_feedthrough, node_name=''):
    if is_feedthrough:
        name = inpt['feedthrough_to']
    else:
        name = inpt['name']

    # Readable format
    Is = inpt['reset']
    Ir = inpt['msg'].pipe(ops.observe_on(scheduler),
                          ops.map(inpt['converter']), ops.share(),
                          ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
                          ops.share()
                          )

    # Get rate from rosparam server
    try:
        dt_i = 1 / rospy.get_param(inpt['address'] + '/rate')
    except Exception as e:
        print('Probably cannot find key "%s" on ros param server.' % inpt['name'] + '/rate')
        print(e)

    # Create input channel
    num_msgs = Nc.pipe(ops.observe_on(scheduler),
                       ops.start_with(0),
                       ops.map(lambda i: {'node_tick': i, 'num_msgs': expected_inputs(i, dt_i, dt_n)}))  # todo: expected_inputs(i-1, dt_i, dt_n)})
    msg = num_msgs.pipe(gen_msg(Ir))
    channel = num_msgs.pipe(ops.filter(lambda x: x['num_msgs'] == 0),
                            ops.with_latest_from(msg),
                            ops.share(),  # todo: really needed with share at the end here?
                            ops.map(get_repeat_fn(inpt['repeat'], 'msg')),
                            ops.merge(msg),
                            regroup_msgs(name, dt_i=dt_i, dt_n=dt_n),
                            ops.share())

    # Create reset flag
    flag = Ir.pipe(# ops.observe_on(scheduler),
                   ops.map(lambda val: val[0] + 1),
                   ops.start_with(0),
                   # spy('Ir_%s' % name, node_name, only_node='/rx/bridge'),
                   ops.combine_latest(Is.pipe(ops.map(lambda msg: msg.data))),  # Depends on ROS reset msg type
                   spy('Ir_Is_%s' % name, node_name, only_node='/rx/bridge'),
                   ops.filter(lambda value: value[0] == value[1]),
                   ops.map(lambda x: {name: x[0]})
                   )  # .subscribe(flag, scheduler=scheduler)  # todo: scheduler change
    return channel, flag


def init_channels(ns, Nc, dt_n, inputs, scheduler, is_feedthrough=False, node_name=''):
    # Create channels
    channels = []
    flags = []
    for i in inputs:
        channel, flag = create_channel(ns, Nc, dt_n, i, scheduler, is_feedthrough=is_feedthrough, node_name=node_name)
        channels.append(channel)
        if node_name in selected_nodes:
            if is_feedthrough:
                name = i['address']
            else:
                name = i['name']
            flag = flag.pipe(spy('flag [%s]' % name.split('/')[-1][:12].ljust(4), node_name))
        flags.append(flag)
    zipped_flags = rx.zip(*flags).pipe(ops.map(lambda x: merge_dicts({}, x)))
    zipped_channels = rx.zip(*channels).pipe(regroup_inputs(dt_n=dt_n, node_name=node_name), ops.share())
    return zipped_channels, zipped_flags


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
        zipped_channels, zipped_flags = init_channels(ns, Nc, dt_n, feedthrough, scheduler, is_feedthrough=True, node_name=node_name)

        # Create switch subject
        RR_ho = BehaviorSubject(zipped_channels)
        d_RR_ho = RR.pipe(ops.map(lambda x: Nc.pipe(ops.map(lambda x: None), ops.start_with(None)))).subscribe(RR_ho)
        rr_channel = RR_ho.pipe(ops.switch_latest())

        # Add disposables
        dispose += [RR_ho, d_RR_ho]
    else:
        # Create switch Subject
        zipped_flags = rx.never().pipe(ops.start_with({}))
        rr_channel = Nc.pipe(ops.map(lambda x: None), ops.start_with(None)) # todo: ADDED start_with FOR DYNAMIC BRIDGE PIPELINE

    return rr_channel, zipped_flags, dispose


def init_state_channel(states, scheduler, node_name=''):
    channels = []
    for s in states:
        c = s['msg'].pipe(ops.map(s['converter']), ops.share(),
                          ops.scan(lambda acc, x: (acc[0] + 1, x), (-1, None)),
                          ops.map(lambda val: dict(msg=val)),
                          regroup_msgs(s['name']),)
        channels.append(c)
    return rx.zip(*channels).pipe(regroup_inputs(node_name=node_name))


def merge_dicts(dict_1, dict_2):
    if isinstance(dict_2, dict):
        dict_2 = (dict_2,)
    for d in dict_2:
        dict_1.update(d)
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
                          ops.start_with(dict(name=s['name'], msg=None)),
                          ops.combine_latest(d),
                          ops.filter(lambda x: x[0]['msg'] is not None or x[1]),
                          ops.map(lambda x: merge_dicts(x[0].copy(), {'done': x[1]})))
        channels.append(c)

    return rx.zip(*channels).pipe(regroup_inputs(node_name=node_name), ops.merge(rx.never()))


def init_callback_pipeline(ns, cb_tick, cb_ft, stream, real_reset, state_inputs, state_outputs, outputs, scheduler, node_name=''):
    d_msg = []
    if real_reset:
        state_stream = init_state_channel(state_inputs, scheduler, node_name=node_name)

        # Split stream into feedthrough (ft) and reset stream
        reset_stream, ft_stream = stream.pipe(ops.partition(lambda x: x[1][1] is None))

        # Either feedthrough action or run callback
        ft_stream = ft_stream.pipe(ops.map(lambda x: x[1][1]),
                                   spy('CB_FEED', node_name),
                                   ops.map(lambda val: cb_ft(val)), ops.share())
        reset_stream = reset_stream.pipe(ops.map(lambda x: x[1][0]),
                                         ops.combine_latest(state_stream),
                                         spy('CB_REAL', node_name),
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
                                    spy('CB_TICK', node_name),
                                    # spy('CB_TICK', node_name, only_node=['/rx/bridge']),
                                    ops.map(lambda val: cb_tick(val)),
                                    ops.share())

    # Publish output msg as ROS topic and to subjects if single process
    for o in outputs:
        d = output_stream.pipe(ops.pluck(o['name']),
                               ops.map(o['converter']), ops.share(),
                               publisher_to_topic(o['msg_pub']),
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

    # Get parameters from ROS param server
    params = []
    for node_name in obj_params['node_names']:
        node_params = get_param_with_blocking(node_name)
        params.append(node_params)
    return params


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
            new_input['repeat'] = 'all'
            new_input['name'] = new_input['address']
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
                new_out = dict()
                new_out.update(i)
                new_out.pop('repeat')
                reactive_proxy.append(new_out)

    return dict(inputs=inputs, reactive_proxy=reactive_proxy, sp_nodes=sp_nodes, launch_nodes=launch_nodes)


def initialize_reactive_proxy_reset(dt_n, RM, reactive_proxy):
    for rx in reactive_proxy:
        if 'disposable' in rx:
            continue
        dt_i = 1 / rx['rate']
        rx['disposable'] = RM.pipe(ops.map(lambda msg: msg.data),
                                   ops.map(lambda idx_n: 1 + int((idx_n - 1) * dt_n / dt_i)),  # We subtract -1 from msg to account for the initial tick.
                                   ops.map(lambda i: UInt64(data=i)),
                                   publisher_to_topic(rx['reset_pub'])).subscribe(rx['reset'])
    return None


def switch_with_check_pipeline(init_ho=None):
    if init_ho is None:
        stream_ho = Subject()
    else:
        stream_ho = BehaviorSubject(init_ho)
    check, stream = stream_ho.pipe(ops.switch_latest(), ops.partition(lambda event: event is None))
    return check, stream, stream_ho


###########################################################################
# RxNode ##################################################################
###########################################################################


def init_node_pipeline(ns, dt_n, cb_tick, inputs, outputs, F, SS_ho, SS_CL_ho, R, RR, real_reset, feedthrough, state_inputs, state_outputs, cb_ft, node_name='', event_scheduler=None, thread_pool_scheduler=None):
    # Node ticks
    Rn = ReplaySubject()  # Reset flag for the node (Nc=Ns and r_signal)
    # Nc = BehaviorSubject(0)  # Number completed callbacks (i.e. send Topics): initialized at zero to kick of chain reaction
    Nc = Subject()  # Number completed callbacks (i.e. send Topics): initialized at zero to kick of chain reaction # todo: CHANGED TO Subject() FOR DYNAMIC BRIDGE PIPELINE
    Ns = BehaviorSubject(0)  # Number of started callbacks (i.e. number of planned Topics).

    # Create input channels
    zipped_inputs, zipped_input_flags = init_channels(ns, Nc, dt_n, inputs, node_name=node_name, scheduler=thread_pool_scheduler)

    # Create action channels
    zipped_feedthrough, zipped_action_flags, d_rr = init_real_reset(ns, Nc, dt_n, RR, real_reset, feedthrough, thread_pool_scheduler, node_name=node_name)

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
                       ops.take(1),
                       ops.merge(rx.never()),
                       ).subscribe(Rn)

    # Create reset flags for the set_states (only concerns StateNode)
    if len(state_inputs) > 0 and not real_reset:
        ss_flags = init_state_inputs_channel(ns, state_inputs, event_scheduler, node_name=node_name)
        SS_ho.on_next(ss_flags.pipe(ops.take(1), ops.start_with(None)))
        SS_CL_ho.on_next(ss_flags.pipe(ops.start_with(None)))
    else:
        SS_ho.on_next(rx.never().pipe(ops.start_with('n/a'), ops.start_with(None)))
        SS_CL_ho.on_next(rx.never().pipe(ops.start_with('n/a'), ops.start_with(None)))

    # Combine flags and subscribe F to all the flags zipped together
    d_flag = rx.zip(zipped_input_flags, zipped_action_flags).pipe(ops.map(lambda x: merge_dicts(x[0], x[1]))).subscribe(F, scheduler=thread_pool_scheduler)  # todo: CHECK!

    # Dispose
    dispose = [Rn, Nc, Ns, d_Rn, d_Nc, d_Ns, d_flag] + d_msg + d_rr
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
    if real_reset:
        RRS = Subject()
        resettable_real = dict()
        resettable_real['address'] = ns + '/resettable/real'
        resettable_real['msg'] = RRS
        resettable_real['msg_type'] = String
        resettable_real['msg_pub'] = rospy.Publisher(resettable_real['address'], resettable_real['msg_type'], queue_size=0)
        node_outputs.append(resettable_real)

    # Prepare input topics
    flags = []
    for i in inputs:
        # Subscribe to input topic
        Ir = Subject()
        i['msg'] = Ir

        # Subscribe to input reset topic
        Is = Subject()
        i['reset'] = Is

        # Prepare initial flag
        flag = i['reset'].pipe(flag_dict(i['name']), ops.first(), ops.merge(rx.never()))
        flags.append(flag)

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

        # Prepare initial flag
        flag = i['reset'].pipe(flag_dict(i['address']), ops.first(), ops.merge(rx.never()))
        flags.append(flag)

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

            BehaviorSubject(String(i['address'])).pipe(ops.zip(RR),
                                                       ops.first(),
                                                       ops.map(lambda x: x[0]),
                                                       publisher_to_topic(resettable_real['msg_pub'])).subscribe(resettable_real['msg'])

    # Prepare initial flags
    F_init = rx.zip(*flags).pipe(spy('zip', node_name), ops.map(lambda x: merge_dicts({}, x)))

    # Reset flags
    F = Subject()
    f = rx.merge(F, F_init)
    check_SS, SS, SS_ho = switch_with_check_pipeline(init_ho=BehaviorSubject('n/a'))
    check_SS_CL, SS_CL, SS_CL_ho = switch_with_check_pipeline(init_ho=BehaviorSubject('n/a'))

    # Node ticks
    Rn_ho = BehaviorSubject(BehaviorSubject((0, 0, True)))
    Rn = Rn_ho.pipe(ops.switch_latest(), ops.map(lambda x: x[0]))

    # Create reset switch_latest
    Rr = R.pipe(ops.map(lambda x: True))  # ops.observe_on(event_scheduler),  # seemed to make ROS variant crash

    # Reset node pipeline
    Nc_reset = rx.zip(Rr.pipe(spy('Rr', node_name)),
                      Rn.pipe(spy('Rn', node_name)),
                      f.pipe(spy('F', node_name)),
                      SS.pipe(spy('SS', node_name))).pipe(ops.with_latest_from(SS_CL),
                                                          ops.map(lambda x: x[0][:-1] + (x[1],)),
                                                          spy('flags_zipped', node_name),
                                                          ops.map(lambda x: (x[1], x[3])), ops.share())  # x[0][0]=Nc
    reset_obs = Nc_reset.pipe(ops.map(lambda x: init_node_pipeline(ns, dt_n, cb_tick, inputs, outputs, F, SS_ho, SS_CL_ho, R, RR, real_reset, feedthrough, state_inputs, state_outputs, cb_ft, node_name=node_name, event_scheduler=event_scheduler, thread_pool_scheduler=thread_pool_scheduler)),
                              trace_observable('init_node_pipeline', node_name),
                              ops.share())
    reset_obs.pipe(ops.pluck('Rn')).subscribe(Rn_ho)

    # Dispose old pipeline, run reset callback & send reset topic
    reset_msg = reset_obs.pipe(ops.pluck('dispose'),
                               ops.buffer_with_count(2, skip=1),
                               ops.map(lambda x: [d.dispose() for d in x[0]]),
                               ops.start_with(None),
                               # zipped with Rn_ho so that Rn has switched before sending "reset topic"
                               ops.zip(Nc_reset, Rn_ho.pipe(ops.skip(1)), check_SS, check_SS_CL),
                               ops.map(lambda x: x[1]),  # x[1]=Nc
                               ops.share(),
                               spy('RESET', node_name),
                               ops.map(lambda x: (x[0], cb_reset(x))),
                               trace_observable('cb_reset', node_name),
                               ops.map(lambda x: UInt64(data=x[0])),
                               ops.share())

    [reset_msg.pipe(publisher_to_topic(o['reset_pub'])).subscribe(o['reset']) for o in outputs]

    node_inputs = (reset_input, real_reset_input)
    rx_objects = dict(inputs=inputs, outputs=outputs, feedthrough=feedthrough, state_inputs=state_inputs, state_outputs=state_outputs, node_inputs=node_inputs, node_outputs=node_outputs)
    return rx_objects


###########################################################################
# RxBridge#################################################################
###########################################################################


def init_bridge_pipeline(ns, dt_n, cb_tick, zipped_channels, outputs, Nc_ho, DF, RRn_ho, node_name='', event_scheduler=None, thread_pool_scheduler=None):
    # Node ticks
    RRn = Subject()
    RRn_ho.on_next(RRn)
    Nc = Subject()  # Number completed callbacks (i.e. send Topics): initialized at zero to kick of chain reaction
    Ns = BehaviorSubject(0)  # Number of started callbacks (i.e. number of planned Topics).
    Nc_ho.on_next(Nc)

    # Create a tuple with None, to be consistent with feedthrough pipeline of init_node_pipeline
    zipped_channels = zipped_channels.pipe(ops.map(lambda i: (i, None)))

    # New routine with merge
    PR = DF.pipe(ops.filter(lambda x: all([df for df in x])),
                 spy('DF filter', node_name),
                 ops.observe_on(event_scheduler),
                 ops.map(lambda x: True),
                 ops.merge(zipped_channels),
                 switch_to_reset(),
                 ops.share())

    # Create reset signal
    RRr, P = PR.pipe(ops.partition(lambda value: isinstance(value, bool)))

    # Create accumulator: (acc)
    d_Ns = P.pipe(ops.scan(lambda acc, x: acc + 1, 0)).subscribe(Ns)

    # Create callback stream
    input_stream = Ns.pipe(ops.skip(1), ops.zip(P), ops.share())
    d_msg = init_callback_pipeline(ns, cb_tick, cb_ft, input_stream, False, tuple(), tuple(), outputs, event_scheduler, node_name=node_name)

    # After outputs have been send, increase the completed callback counter
    Nc_obs = rx.zip(*[o['msg'] for o in outputs]).pipe(ops.scan(lambda acc, x: acc + 1, 0))

    # Increase ticks
    d_Nc = Nc_obs.subscribe(Nc, scheduler=thread_pool_scheduler)
    d_RRn = Nc_obs.pipe(spy('Nc_obs', node_name),
                        ops.start_with(0),  # added to simulated first zero from BS(0) of Nc
                        ops.combine_latest(Ns, RRr),
                        spy('RRn pre-filter', node_name),
                        ops.filter(lambda value: value[0] == value[1]),
                        ops.take(1),
                        ops.merge(rx.never()),
                        ).subscribe(RRn)

    # Dispose
    dispose = [RRn, Nc, Ns, d_RRn, d_Nc, d_Ns] + d_msg
    return {'dispose': dispose}


def init_bridge(ns, dt_n, cb_tick, cb_pre_reset, cb_post_reset, cb_register_object, inputs_init, outputs, message_broker, node_name='', scheduler=''):
    ###########################################################################
    # Initialization ##########################################################
    ###########################################################################
    # Prepare scheduler
    if scheduler is not None:
        event_scheduler = scheduler
        thread_pool_scheduler = event_scheduler
    else:
        event_scheduler = EventLoopScheduler()
        thread_pool_scheduler = event_scheduler

    # Prepare output topics
    for i in outputs:
        # Prepare output topic
        i['msg'] = Subject()
        i['msg_pub'] = rospy.Publisher(i['address'], i['msg_type'], queue_size=0)

        # Initialize reset topic
        i['reset'] = Subject()
        i['reset_pub'] = rospy.Publisher(i['address'] + '/reset', UInt64, queue_size=0)

    # Prepare input topics
    for i in inputs_init:
        # Subscribe to input topic
        Ir = Subject()
        i['msg'] = Ir

        # Subscribe to input reset topic
        Is = Subject()
        i['reset'] = Is

    # Track node I/O
    node_inputs = []
    node_outputs = []

    ###########################################################################
    # Registry: real reset states #############################################
    ###########################################################################
    # Resettable real states (to dynamically add state/done flags to DF for RR)
    RRS = Subject()
    RR = Subject()
    RRr = RR.pipe(ops.map(lambda x: True))
    resettable_real = dict(address=ns + '/resettable/real', msg=RRS, msg_type=String)
    node_inputs.append(resettable_real)

    # Switch to latest state reset done flags
    DF_ho = ReplaySubject()  # todo: replay subject?
    DF = DF_ho.pipe(ops.switch_latest())
    RRS.pipe(spy('state register', node_name),
             ops.map(lambda s: dict(address=s.data, done=Subject())),
             ops.map(lambda i: (i, message_broker.add_rx_objects(node_name, state_inputs=[i]))),
             ops.map(lambda i: i[0]['done'].pipe(ops.map(lambda msg: bool(msg.data)))),
             ops.start_with(RRr),
             ops.scan(lambda acc, x: acc + [x], []),
             ops.map(lambda i: rx.combine_latest(*i)),
             trace_observable('DF callback', node_name)).subscribe(DF_ho)

    ###########################################################################
    # Registry: objects #######################################################
    ###########################################################################
    # Object register (to dynamically add input reset flags to F for reset)
    OR = Subject()
    object_registry = dict(address=ns + '/register', msg=OR, msg_type=String)
    node_inputs.append(object_registry)

    # Object registry pipeline
    params_nodes = OR.pipe(ops.map(get_object_params),
                           ops.filter(lambda params: params is not None),
                           ops.map(lambda params: (params,) + cb_register_object(params)), ops.share())
    rx_objects = params_nodes.pipe(ops.map(lambda i: extract_topics_in_reactive_proxy(*i)),
                                   ops.map(lambda i: (i, message_broker.add_rx_objects(node_name, inputs=i['inputs'], reactive_proxy=i['reactive_proxy']))),
                                   ops.map(lambda i: i[0]),
                                   ops.scan(combine_dict, dict(inputs=[], reactive_proxy=[], sp_nodes=[], launch_nodes=[])), ops.share())

    ###########################################################################
    # Start reset #############################################################
    ###########################################################################
    # Prepare start_reset input
    SR = Subject()
    start_reset_input = dict(address=ns + '/start_reset', msg=SR, msg_type=UInt64)
    node_inputs.append(start_reset_input)

    # Latch on '/rx/start_reset' event
    rx_objects = SR.pipe(ops.with_latest_from(rx_objects), ops.map(lambda i: i[1]), ops.share())
    inputs = rx_objects.pipe(ops.pluck('inputs'))
    reactive_proxy = rx_objects.pipe(ops.pluck('reactive_proxy'))

    # Prepare output for reactive proxy
    # todo: test reactive proxy
    RM = Subject()
    check_reactive_proxy = reactive_proxy.pipe(ops.map(lambda rx: initialize_reactive_proxy_reset(dt_n, RM, rx)))

    # Zip initial input flags
    check_F_init, F_init, F_init_ho = switch_with_check_pipeline()
    F_init = F_init.pipe(ops.first(), ops.merge(rx.never()))
    # todo: blocks: flags received, but not zipped
    inputs.pipe(ops.map(lambda inputs: rx.zip(*[i['reset'].pipe(flag_dict(i['name'])) for i in inputs]).pipe(ops.map(lambda x: merge_dicts({}, x)), ops.start_with(None)))).subscribe(F_init_ho)
    F = Subject()
    f = rx.merge(F.pipe(spy('F', node_name)), F_init)

    ###########################################################################
    # Real reset ##############################################################
    ###########################################################################
    # Prepare real_reset output
    real_reset_output = dict(address=ns + '/real_reset', msg=RR, msg_type=UInt64)
    real_reset_output['msg_pub'] = rospy.Publisher(real_reset_output['address'], real_reset_output['msg_type'], queue_size=0)
    node_outputs.append(real_reset_output)

    # Zip switch checks to indicate end of '/rx/start_reset' procedure, and start of '/rx/real_reset'
    rx.zip(check_F_init, check_reactive_proxy).pipe(ops.map(lambda i: message_broker.connect_io()),
                                                    publisher_to_topic(real_reset_output['msg_pub'])).subscribe(RR)

    # Real reset routine. Cuts-off tick_callback when RRr is received, instead of Rr
    check_RRn, RRn, RRn_ho = switch_with_check_pipeline(init_ho=BehaviorSubject((0, 0, True)))
    rx.zip(RRn.pipe(spy('RRn', node_name)),
           RRr.pipe(spy('RRr', node_name))).pipe(ops.map(lambda x: x[0][0]),  # x[0][0]=Nc
                                                 ops.map(lambda x: (x, cb_pre_reset(x))),  # Run pre-reset callback
                                                 spy('PRE-RESET', node_name),
                                                 trace_observable('cb_pre_reset', node_name),
                                                 ops.map(lambda x: UInt64(data=x[0] + 1)),  # todo: Add + 1 for initial tick. Really? Also during initial episode?
                                                 ops.share()).subscribe(RM)

    ###########################################################################
    # Reset ###################################################################
    ###########################################################################
    # Prepare reset output
    R = Subject()
    reset_output = dict(address=ns + '/reset', msg=R, msg_type=UInt64)
    reset_output['msg_pub'] = rospy.Publisher(reset_output['address'], reset_output['msg_type'], queue_size=0)
    node_outputs.append(reset_output)

    # Send reset message
    RM.pipe(publisher_to_topic(reset_output['msg_pub'])).subscribe(R)
    Rr = R.pipe(ops.map(lambda x: True))
    reset_trigger = rx.zip(f.pipe(spy('f', node_name)),
                           Rr.pipe(spy('Rr', node_name)))

    # Send reset messages for all outputs (Only '/rx/bridge/tick')
    [RM.pipe(publisher_to_topic(o['reset_pub'])).subscribe(o['reset']) for o in outputs]

    ###########################################################################
    # Reset: initialize episode pipeline ######################################
    ###########################################################################
    # Dynamically initialize new input pipeline
    check_Nc, Nc, Nc_ho = switch_with_check_pipeline()
    inputs_flags = inputs.pipe(ops.zip(reset_trigger),
                               ops.map(lambda i: i[0]),
                               ops.map(lambda inputs: init_channels(ns, Nc, dt_n, inputs, node_name=node_name, scheduler=thread_pool_scheduler)),
                               # spy('inputs_flags', node_name),
                               ops.share())

    # Switch to latest zipped inputs pipeline
    check_z_inputs, z_inputs, z_inputs_ho = switch_with_check_pipeline()
    inputs_flags.pipe(ops.map(lambda i: i[0].pipe(ops.start_with(None)))).subscribe(z_inputs_ho, scheduler=thread_pool_scheduler)

    # Switch to latest zipped flags pipeline
    check_z_flags, z_flags, z_flags_ho = switch_with_check_pipeline()
    inputs_flags.pipe(ops.map(lambda i: i[1].pipe(ops.start_with(None)))).subscribe(z_flags_ho, scheduler=thread_pool_scheduler)
    z_flags.subscribe(F)

    # Initialize rest of episode pipeline
    pipeline_trigger = rx.zip(check_z_flags, check_z_inputs)
    reset_obs = pipeline_trigger.pipe(ops.map(lambda x: init_bridge_pipeline(ns, dt_n, cb_tick, z_inputs, outputs, Nc_ho, DF, RRn_ho, node_name=node_name, event_scheduler=event_scheduler, thread_pool_scheduler=thread_pool_scheduler)),
                                      trace_observable('init_bridge_pipeline', node_name), ops.share())

    ###########################################################################
    # End reset ###############################################################
    ###########################################################################
    # Prepare end_reset output
    end_reset = dict(address=ns + '/end_reset', msg=Subject(), msg_type=UInt64)
    end_reset['msg_pub'] = rospy.Publisher(end_reset['address'], end_reset['msg_type'], queue_size=0)
    node_outputs.append(end_reset)

    # Send initial tick after reset has finished
    reset_obs.pipe(ops.pluck('dispose'),
                   ops.buffer_with_count(2, skip=1),
                   ops.map(lambda x: [d.dispose() for d in x[0]]),
                   ops.start_with(None),
                   ops.zip(reset_obs),
                   ops.map(lambda x: cb_post_reset()),
                   spy('POST-RESET', node_name),
                   trace_observable('cb_post_reset', node_name),
                   ops.map(lambda x: UInt64(data=0)),
                   ops.share(),
                   publisher_to_topic(end_reset['msg_pub'])).subscribe(end_reset['msg'])

    rx_objects = dict(inputs=inputs_init, outputs=outputs, node_inputs=node_inputs, node_outputs=node_outputs)
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
            if i['address'] == '/rx/bridge/tick': continue
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
            if 'msg' in i:  # Only true if real reset or sim state node (i.e. **not** for bridge done flags)
                n['state_inputs'][address + '/set'] = {'rx': i['msg'], 'disposable': None, 'source': i, 'msg_type': i['msg_type'], 'converter': i['converter'], 'status': 'disconnected'}
            if address + '/done' not in n['state_outputs'].keys():  # Only true if **not** a real reset node (i.e., sim state & bridge done flag)
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
            cprint(('OWNER "%s"' % self.owner).ljust(15, ' ') + ('| OVERVIEW NODE "%s" ' % node_name).ljust(180, " "), attrs=['bold', 'underline'])
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
                    address_str = ('| %s ' % address).ljust(50, ' ')
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
            print_status and cprint(('OWNER "%s"' % self.owner).ljust(15, ' ') + ('| CONNECTING NODE "%s" ' % node_name).ljust(180, " "), attrs=['bold', 'underline'])
            for key, addresses in node.items():
                for address in list(addresses.keys()):
                    entry = addresses[address]
                    assert address not in self.connected_rx[node_name][key], 'Address (%s) of this node (%s) already connected via rx.' % (address, node_name)
                    assert address not in self.connected_ros[node_name][key], 'Address (%s) of this node (%s) already connected via ROS.' % (address, node_name)
                    if address in self.rx_connectable.keys():
                        color = 'green'
                        status = 'Rx'.ljust(4, ' ')
                        entry['rate'] = self.rx_connectable[address]['rate']
                        rate_str = '|' + ('%s' % entry['rate']).center(3, ' ')
                        node_str = ('| %s' % self.rx_connectable[address]['node_name']).ljust(40, ' ')
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
                    address_str = ('| %s ' % address).ljust(50, ' ')
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
