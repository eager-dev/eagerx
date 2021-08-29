from typing import List, Dict
from eagerx_core.utils.utils import load_yaml, get_attribute_from_module

class Params(object):
    def __init__(self, **kwargs):
        # Iterates over provided arguments and sets the provided arguments as class properties
        for key, value in kwargs.items():
            setattr(self, key, value)


class RxInput(Params):
    def __init__(self,
                 name: str,
                 address: str,
                 msg_type: str,
                 repeat: str = 'all',
                 msg_module: str = 'std_msgs.msg',
                 converter: str = 'identity',
                 converter_module: str = 'eagerx_core',
                 is_reactive: bool = True,
                 ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        super(RxInput, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def get_params(self, ns=''):
        params = self.__dict__.copy()
        params['address'] = ns + '/' + params['address']
        return params


class RxOutput(Params):
    def __init__(self,
                 name: str,
                 address: str,
                 msg_type: str,
                 rate: int,
                 msg_module: str = 'std_msgs.msg',
                 converter: str = 'identity',
                 converter_module: str = 'eagerx_core',
                 ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        super(RxOutput, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def get_params(self, ns=''):
        params = self.__dict__.copy()
        params['address'] = ns + '/' + params['address']
        return params


class RxFeedthrough(Params):
    def __init__(self,
                 # name: str,
                 address: str,
                 msg_type: str,
                 feedthrough_to: str,
                 repeat: str = 'all',
                 msg_module: str = 'std_msgs.msg',
                 converter: str = 'identity',
                 converter_module: str = 'eagerx_core',
                 is_reactive: bool = True,
                 ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        super(RxFeedthrough, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def get_params(self, ns=''):
        params = self.__dict__.copy()
        # params['feedthrough_to'] = ns + '/' + params['feedthrough_to']
        params['address'] = ns + '/' + params['address']
        return params


class RxState(Params):
    def __init__(self,
                 name: str,
                 address: str,
                 msg_type: str,
                 msg_module: str = 'std_msgs.msg',
                 converter: str = 'identity',
                 converter_module: str = 'eagerx_core'
                 ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        super(RxState, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def get_params(self, ns=''):
        params = self.__dict__.copy()
        params['address'] = ns + '/' + params['address']
        return params


class RxNodeParams(Params):
    def __init__(self,
                 name: str,
                 node_type: str,
                 module: str,
                 topics_out: List[RxOutput],
                 topics_in: List[RxInput],
                 feedthrough_in: List[RxFeedthrough] = [],
                 states_in: List[RxState] = [],
                 # set_states_in: List[RxState] = [],
                 launch_locally: bool = True,
                 single_process: bool = True,
                 **node_args):
        # Only define variables (locally) you wish to store on the parameter server (done in baseclass constructor).
        launch_file = '$(find eagerx_core)/launch/rxnode.launch'

        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        super(RxNodeParams, self).__init__(**kwargs)

        # Check that node has at least one input.
        if len(topics_in) == 0:
            raise ValueError('Nodes must have at least one input. Node "%s" does not have inputs' % name)

        # Check that output rates are the same
        rates = []
        for o in topics_out:
            rates.append(o.rate)
        if len(set(rates)) > 1:
            raise ValueError('Nodes can only have outputs with the same rate. Check the output rates of node "%s".' % name)

        # Calculate other parameters based on previously defined attributes.
        self.rate = topics_out[0].rate

    def get_params(self, ns=''):
        params = self.__dict__.copy()

        # Calculate properties
        params['topics_out'] = [i.get_params(ns=ns) for i in params['topics_out']]
        params['topics_in'] = [i.get_params(ns=ns) for i in params['topics_in']]
        params['feedthrough_in'] = [i.get_params(ns=ns) for i in params['feedthrough_in']]
        params['states_in'] = [i.get_params(ns=ns) for i in params['states_in']]
        # params['set_states_in'] = [i.get_params(ns=ns) for i in params['set_states_in']]

        # Get rate dictionary
        chars_ns = len(ns)+1
        rate_dict = dict()
        for i in params['topics_out']:
            address = i['address'][chars_ns:]
            rate_dict[address] = {'rate': i['rate']}

        # Put parameters in node namespace
        # TODO: WATCH OUT, ORDER OF DICT KEY MATTERS!
        params = {self.name: params}
        if self.name in rate_dict.keys():
            raise ValueError('Cannot use the same output topic name for a node (%s).' % self.name)

        # Add output rates in separate namespace
        params.update(rate_dict)
        return params


class RxBridgeParams(Params):
    def __init__(self,
                 rate: float = 10,
                 num_substeps: int = 10,
                 launch_locally: bool = True,
                 single_process: bool = True,
                 ):
        # Only define variables (locally) you wish to store on the parameter server (done in baseclass constructor).
        name = 'bridge'
        node_type = 'BridgeNode'
        module = 'eagerx_core.bridge'
        launch_file = '$(find eagerx_core)/launch/rxbridge.launch'

        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        super(RxBridgeParams, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.
        self.dt = 1 / self.rate
        self.sub_dt = self.dt / self.num_substeps

        # Error check the parameters here.

    def get_params(self, ns=''):
        bridge_params = self.__dict__.copy()

        bridge = RxNodeParams(name=self.name, node_type=self.node_type, module=self.module,
                              topics_in=[RxInput('tick', 'bridge/tick', 'UInt64')],
                              topics_out=[RxOutput('tick', 'bridge/tick', 'UInt64', rate=self.rate)])

        params = bridge.get_params(ns=ns)
        params[self.name].update(bridge_params)
        # TODO: WATCH OUT, ORDER OF DICT KEY MATTERS!
        return params


class RxObjectParams(Params):
    def __init__(self,
                 name: str,
                 params: Dict,
                 ):

        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        super(RxObjectParams, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def get_params(self, ns, bridge):
        params = self.params.copy()
        name = self.name

        # Check if the bridge name is not contained in the config's (.yaml) default parameters
        if bridge in params['default']:
            raise ValueError('Cannot have a bridge (%s) as a default object parameter int the config (.yaml) of object (%s).' % (bridge, self.name))
        if 'node_names' in params['default']:
            raise ValueError('Cannot have a keyword (%s) as a default object parameter int the config (.yaml) of object (%s).' % ('node_names', self.name))
        if bridge not in params:
            raise ValueError('The config (.yaml) of object (%s) does not provide support for the selected bridge (%s).' % (self.name, bridge))

        # Create sensors
        component = 'sensors'
        sensors = []
        for i in params['default'][component]:
            if i not in params[bridge][component]:
                raise ValueError('%s not defined for bridge %s and component "%s".' % (i, bridge, component))
            p_env = params[component][i]
            p_bridge = params[bridge][component][i]

            # Prepare node arguments
            args = p_bridge.copy()
            del args['node_type'], args['module']
            args['outputs'], args['outputs_address'] = ['%s' % i], ['%s/%s/%s' % (name, component, i)]
            args['name'], args['rate'] = '%s/nodes/%s/%s' % (name, component, i), p_env['rate']

            # Get node param class
            node_cls = get_attribute_from_module(p_bridge['module'], p_bridge['node_type'])
            node = node_cls(**args)
            sensors.append(node)

        # Create actuators
        component = 'actuators'
        actuators = []
        for i in params['default'][component]:
            if i not in params[bridge][component]:
                raise ValueError('%s not defined for bridge %s and component "%s".' % (i, bridge, component))
            p_env = params[component][i]
            p_bridge = params[bridge][component][i]

            # Prepare node arguments
            args = p_bridge.copy()
            del args['node_type'], args['module']
            args['outputs'], args['outputs_address'] = ['%s' % i], ['%s/%s/%s/applied' % (name, component, i)]
            args['name'], args['rate'] = '%s/nodes/%s/%s' % (name, component, i), p_env['rate']

            # Get node param class
            node_cls = get_attribute_from_module(p_bridge['module'], p_bridge['node_type'])
            node = node_cls(**args)
            actuators.append(node)

        # Create states
        component = 'states'
        states = []
        for i in params['default'][component]:
            if i not in params[bridge][component]:
                raise ValueError('%s not defined for bridge %s and component "%s".' % (i, bridge, component))
            p_env = params[component][i]
            p_bridge = params[bridge][component][i]

            # Prepare node arguments
            args = p_bridge.copy()
            del args['node_type'], args['module']
            args['outputs'], args['outputs_address'] = ['%s' % i], ['%s/%s/%s' % (name, component, i)]
            args['name'], args['rate'] = '%s/nodes/%s/%s' % (name, component, i), p_env['rate']

            # Get node param class
            node_cls = get_attribute_from_module(p_bridge['module'], p_bridge['node_type'])
            node = node_cls(**args)
            states.append(node)

        nodes = sensors + actuators + states

        # Create obj parameters
        obj_params = dict()
        obj_params.update(params['default'])

        # Gather node names
        sens_names = ['%s/%s/nodes/sensors/%s' % (ns, name, i) for i in obj_params['sensors']]
        act_names = ['%s/%s/nodes/actuators/%s' % (ns, name, i) for i in obj_params['actuators']]
        state_names = ['%s/%s/nodes/states/%s' % (ns, name, i) for i in obj_params['states']]
        obj_params['node_names'] = sens_names + act_names + state_names

        # Remove clutter from parameters
        obj_params[bridge] = params[bridge]
        for c in ('sensors', 'actuators', 'states'):
            if c in obj_params[bridge]:
                del obj_params[bridge][c]
            if c in obj_params:
                del obj_params[c]

        return {self.name: obj_params}, nodes

    @classmethod
    def create(cls,
               name: str,
               package_name: str,
               object_name: str,
               **kwargs):

        # Load yaml from config file
        params = load_yaml(package_name, object_name)

        # Replace default arguments
        for key, item in kwargs.items():
            if key in params['default'].keys():
                params['default'][key] = item
        return cls(name, params)


######## NODE IMPLEMENTATIONS ########


class ProcessNode(RxNodeParams):
    def __init__(self,
                 name: str,
                 inputs: List,
                 inputs_address: List,
                 outputs: List,
                 outputs_address: List,
                 rate: int,
                 launch_locally: bool = True,
                 single_process: bool = True,
                 inputs_converter: List = None,
                 inputs_converter_module: List = None,
                 outputs_converter: List = None,
                 outputs_converter_module: List = None,
                 ):
        node_type = 'ProcessNode'
        module = 'eagerx_core.node'

        # Create topics_in
        topics_in = []
        for i in range(len(inputs)):
            args = dict()
            args['name'], args['address'] = inputs[i], inputs_address[i]
            args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
            args['repeat'], args['is_reactive'] = 'all', 'True'
            if inputs_converter:
                args['converter'], args['converter_module'] = inputs_converter[i], inputs_converter_module[i]
            n = RxInput(**args)
            topics_in.append(n)

        # Create topics_in
        topics_out = []
        for i in range(len(outputs)):
            args = dict()
            args['name'], args['address'] = outputs[i], outputs_address[i]
            args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
            args['rate'] = rate
            if outputs_converter:
                args['converter'], args['converter_module'] = outputs_converter[i], outputs_converter_module[i]
            n = RxOutput(**args)
            topics_out.append(n)

        super(ProcessNode, self).__init__(name=name, node_type=node_type, module=module,
                                          launch_locally=launch_locally, single_process=single_process,
                                          topics_in=topics_in, topics_out=topics_out)


class RealResetNode(RxNodeParams):
    def __init__(self,
                 name: str,
                 inputs: List,
                 inputs_address: List,
                 outputs: List,
                 outputs_address: List,
                 states: List,
                 states_address: List,
                 feedthrough_address: List,
                 feedthrough_to: List,
                 rate: int,
                 launch_locally: bool = True,
                 single_process: bool = True,
                 inputs_converter: List = None,
                 inputs_converter_module: List = None,
                 outputs_converter: List = None,
                 outputs_converter_module: List = None,
                 states_converter: List = None,
                 states_converter_module: List = None,
                 feedthrough_converter: List = None,
                 feedthrough_converter_module: List = None,
                 **kwargs):
        node_type = 'RealResetNode'
        module = 'eagerx_core.node'

        # Create topics_in
        topics_in = []
        for i in range(len(inputs)):
            args = dict()
            args['name'], args['address'] = inputs[i], inputs_address[i]
            args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
            args['repeat'], args['is_reactive'] = 'all', 'True'
            if inputs_converter:
                args['converter'], args['converter_module'] = inputs_converter[i], inputs_converter_module[i]
            n = RxInput(**args)
            topics_in.append(n)

        # Create topics_in
        topics_out = []
        for i in range(len(outputs)):
            args = dict()
            args['name'], args['address'] = outputs[i], outputs_address[i]
            args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
            args['rate'] = rate
            if outputs_converter:
                args['converter'], args['converter_module'] = outputs_converter[i], outputs_converter_module[i]
            n = RxOutput(**args)
            topics_out.append(n)

        # Create feedthrough_in
        feedthrough_in = []
        for i in range(len(feedthrough_address)):
            args = dict()
            if feedthrough_to[i] not in outputs[i]:
                raise ValueError('The output (%s) we feedthrough to address (%s) is not defined.' % (feedthrough_to[i], feedthrough_address[i]))
            args['address'], args['feedthrough_to'] = feedthrough_address[i], feedthrough_to[i]
            args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
            args['is_reactive'] = True
            if feedthrough_converter:
                args['converter'], args['converter_module'] = feedthrough_converter[i], feedthrough_converter_module[i]
            n = RxFeedthrough(**args)
            feedthrough_in.append(n)

        # Create states_in
        states_in = []
        for i in range(len(states)):
            args = dict()
            args['name'], args['address'] = states[i], states_address[i]
            args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
            if states_converter:
                args['converter'], args['converter_module'] = states_converter[i], states_converter_module[i]
            n = RxState(**args)
            states_in.append(n)

        super(RealResetNode, self).__init__(name=name, node_type=node_type, module=module,
                                            launch_locally=launch_locally, single_process=single_process,
                                            topics_in=topics_in, topics_out=topics_out, feedthrough_in=feedthrough_in,
                                            states_in=states_in, **kwargs)


class SimActuatorNode(RxNodeParams):
    def __init__(self,
                 name: str,
                 outputs: List,
                 outputs_address: List,
                 rate: int,
                 inputs: List = None,
                 inputs_address: List = None,
                 inputs_converter: List = None,
                 inputs_converter_module: List = None,
                 outputs_converter: List = None,
                 outputs_converter_module: List = None,
                 ):
        node_type = 'ProcessNode'
        module = 'eagerx_core.node'
        launch_locally = True
        single_process = True

        # Create topics_in
        topics_in = [RxInput('tick', 'bridge/tick', 'UInt64')]
        for i in range(len(inputs)):
            args = dict()
            args['name'], args['address'] = inputs[i], inputs_address[i]
            args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
            args['repeat'], args['is_reactive'] = 'all', 'True'
            if inputs_converter:
                args['converter'], args['converter_module'] = inputs_converter[i], inputs_converter_module[i]
            n = RxInput(**args)
            topics_in.append(n)

        # Create topics_in
        topics_out = []
        for i in range(len(outputs)):
            args = dict()
            args['name'], args['address'] = outputs[i], outputs_address[i]
            args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
            args['rate'] = rate
            if outputs_converter:
                args['converter'], args['converter_module'] = outputs_converter[i], outputs_converter_module[i]
            n = RxOutput(**args)
            topics_out.append(n)

        super(SimActuatorNode, self).__init__(name=name, node_type=node_type, module=module,
                                              launch_locally=launch_locally, single_process=single_process,
                                              topics_in=topics_in, topics_out=topics_out)


class SimSensorNode(RxNodeParams):
    def __init__(self,
                 name: str,
                 outputs: List,
                 outputs_address: List,
                 rate: int,
                 inputs: List = None,
                 inputs_address: List = None,
                 inputs_converter: List = None,
                 inputs_converter_module: List = None,
                 outputs_converter: List = None,
                 outputs_converter_module: List = None,
                 ):
        node_type = 'ProcessNode'
        module = 'eagerx_core.node'
        launch_locally = True
        single_process = True

        # Create topics_in
        topics_in = [RxInput('tick', 'bridge/tick', 'UInt64')]
        if inputs:
            for i in range(len(inputs)):
                args = dict()
                args['name'], args['address'] = inputs[i], inputs_address[i]
                args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
                args['repeat'], args['is_reactive'] = 'all', 'True'
                if inputs_converter:
                    args['converter'], args['converter_module'] = inputs_converter[i], inputs_converter_module[i]
                n = RxInput(**args)
                topics_in.append(n)

        # Create topics_out
        topics_out = []
        for i in range(len(outputs)):
            args = dict()
            args['name'], args['address'] = outputs[i], outputs_address[i]
            args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
            args['rate'] = rate
            if outputs_converter:
                args['converter'], args['converter_module'] = outputs_converter[i], outputs_converter_module[i]
            n = RxOutput(**args)
            topics_out.append(n)

        super(SimSensorNode, self).__init__(name=name, node_type=node_type, module=module,
                                            launch_locally=launch_locally, single_process=single_process,
                                            topics_in=topics_in, topics_out=topics_out)


class SimStateNode(RxNodeParams):
    def __init__(self,
                 name: str,
                 outputs: List,
                 outputs_address: List,
                 rate: int,
                 inputs: List = None,
                 inputs_address: List = None,
                 inputs_converter: List = None,
                 inputs_converter_module: List = None,
                 outputs_converter: List = None,
                 outputs_converter_module: List = None,
                 states_converter: List = None,
                 states_converter_module: List = None,
                 ):
        node_type = 'StateNode'
        module = 'eagerx_core.node'
        launch_locally = True
        single_process = True

        # Create topics_in
        topics_in = [RxInput('tick', 'bridge/tick', 'UInt64')]
        if inputs:
            for i in range(len(inputs)):
                args = dict()
                args['name'], args['address'] = inputs[i], inputs_address[i]
                args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
                args['repeat'], args['is_reactive'] = 'all', 'True'
                if inputs_converter:
                    args['converter'], args['converter_module'] = inputs_converter[i], inputs_converter_module[i]
                n = RxInput(**args)
                topics_in.append(n)

        # Create topics_out
        topics_out = []
        for i in range(len(outputs)):
            args = dict()
            args['name'], args['address'] = outputs[i], outputs_address[i]
            args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
            args['rate'] = rate
            if outputs_converter:
                args['converter'], args['converter_module'] = outputs_converter[i], outputs_converter_module[i]
            n = RxOutput(**args)
            topics_out.append(n)

        # Create set_states_in
        states_in = []
        for i in range(len(outputs)):
            args = dict()
            args['name'], args['address'] = outputs[i], outputs_address[i]
            args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
            if states_converter:
                args['converter'], args['converter_module'] = states_converter[i], states_converter_module[i]
            n = RxState(**args)
            states_in.append(n)

        super(SimStateNode, self).__init__(name=name, node_type=node_type, module=module,
                                           launch_locally=launch_locally, single_process=single_process,
                                           topics_in=topics_in, topics_out=topics_out, states_in=states_in)
