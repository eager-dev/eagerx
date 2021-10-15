from typing import List, Dict, Tuple
from eagerx_core.utils.utils import load_yaml, get_attribute_from_module


class Params(object):
    def __init__(self, **kwargs):
        # Iterates over provided arguments and sets the provided arguments as class properties
        for key, value in kwargs.items():
            if key == '__class__': continue   # Skip if __class__ type
            setattr(self, key, value)


class RxInput(Params):
    def __init__(self,
                 name: str,
                 address: str,
                 msg_type: str,
                 repeat: str = 'all',
                 msg_module: str = 'std_msgs.msg',
                 converter: Dict = None,
                 is_reactive: bool = True,
                 rate: int = None,
                 gym_space: Dict = None,
                 ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        if not converter:
            del converter

        # If gym_space undefined, remove it
        if not gym_space:
            del gym_space

        if is_reactive:
            del rate
        else:
            assert rate is not None, 'Input (%s) is not reactive, so the rate must be defined.' % address

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
                 converter: Dict = None,
                 gym_space: Dict = None,
                 ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        # If gym_space undefined, remove it
        if not converter:
            del converter

        if not gym_space:
            del gym_space
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
                 converter: Dict = None,
                 is_reactive: bool = True,
                 gym_space: Dict = None,
                 ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        # If gym_space undefined, remove it
        if not converter:
            del converter

        if not gym_space:
            del gym_space
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
                 converter: Dict = None,
                 gym_space: Dict = None,
                 ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        # If gym_space undefined, remove it
        if not converter:
            del converter

        if not gym_space:
            del gym_space
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
                 params: Dict,
                 ):
        # Only define variables (locally) you wish to store on the parameter server (done in baseclass constructor).

        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        kwargs = locals().copy()
        kwargs.pop('self')
        super(RxNodeParams, self).__init__(**kwargs)

        # Check that node has at least one input.
        assert len(params['inputs']) > 0, 'Nodes must have at least one input. Node "%s" does not have inputs.' % name

    @classmethod
    def create(cls, name: str, package_name: str, config_name: str, **kwargs):
        # default arguments, not specified in node_name.yaml
        nonlisted_yaml_args = ['input_converters', 'output_converters', 'state_converters', 'feedthrough_converters']
        ignored_yaml_args = ['inputs', 'states', 'feedthroughs']

        # Load yaml from config file
        params = load_yaml(package_name, config_name)

        # Replace default arguments
        for key, item in kwargs.items():
            if key in nonlisted_yaml_args and key.split('_')[0] + 's' in params:
                params['default'][key] = item
                continue
            assert key in params['default'], 'Received unknown argument "%s". Check under "default" in "%s.yaml" inside ROS package "%s/config" for all possible arguments.' % (key, config_name, package_name)
            params['default'][key] = item

        # Check if all arguments are specified
        for key, value in params['default'].items():
            if key in ignored_yaml_args: continue
            assert value is not None, 'Missing argument "%s". Check under "default" in "%s.yaml" inside ROS package "%s/config" for all required arguments.' % (key, config_name, package_name)

        # Check reserved keywords
        assert 'name' not in params['default'], 'Argument "name" is reserved. Modify name in "%s.yaml" inside ROS package "%s/config" for all required arguments.' % (config_name, package_name)
        assert 'package_name' not in params['default'], 'Argument "package_name" is reserved. Modify name in "%s.yaml" inside ROS package "%s/config" for all required arguments.' % (config_name, package_name)
        assert 'config_name' not in params['default'], 'Argument "config_name" is reserved. Modify name in "%s.yaml" inside ROS package "%s/config" for all required arguments.' % (config_name, package_name)
        params['default']['name'] = name
        params['default']['package_name'] = package_name
        params['default']['config_name'] = config_name
        return cls(name, params)

    def get_component(self, component, cname=None, **kwargs):
        name = self.name

        # If no cname is provided, add all components
        if cname is None:
            cname = list(self.params['default'][component].keys())

        # Verify that cname is a list
        if not isinstance(cname, list):
            cname = [cname]

        # Verify that all components are contained in defined components
        for i in cname:
            assert i in self.params['default'][component], '"%s" not included in the %s of object "%s".' % (i, component, name)

        # Loop over components
        cdict = dict()
        cdict[name] = dict()
        for i in cname:
            params = self.params[component][i].copy()
            address = self.params['default'][component][i]

            # Create converter
            gym_space = params['gym_space']
            args = params['gym_space'].copy()
            del args['converter_type']
            converter_cls = get_attribute_from_module(*gym_space['converter_type'].split('/'))
            # todo: Check if converter is string, then do not reload converter_cls when initializing rxpipeline
            converter = converter_cls(**args)

            # Create rxobject
            # todo: add feedthroughs
            if component in ['inputs', 'feedthroughs']:
                args = dict()
                args['name'], args['address'] = ('%s' % i, address)
                args['msg_module'], args['msg_type'] = params['msg_type'].split('/')
                args['rate'] = None
                args['converter'] = converter
                rxobject = RxOutput(**args)
            # todo: check if an output converter is used
            elif component in ['outputs']:
                # assert 'repeat' in kwargs, 'Missing argument"%s". Must be defined when converting sensors and/or states to observations.' % 'repeat'
                args = dict()
                args['name'], args['address'] = ('%s' % i, address)
                args['msg_module'], args['msg_type'] = params['msg_type'].split('/')
                args['repeat'] = kwargs['repeat'] if 'repeat' in kwargs else 'all'  # todo: change to last?
                args['converter'] = converter
                rxobject = RxInput(**args)

            # Define component
            cdict[name][i] = (None, rxobject, converter)

        return cdict

    def get_action(self, cname=None, **kwargs):
        return self.get_component('inputs', cname, **kwargs)

    def get_observation(self, cname=None, **kwargs):
        return self.get_component('outputs', cname, **kwargs)

    def get_feedthrough(self, cname=None, **kwargs):
        return self.get_component('feedthroughs', cname, **kwargs)

    def get_params(self, ns=''):
        params = self.params.copy()
        default = params['default'].copy()
        name = self.name
        package_name = default['package_name']
        config_name = default['config_name']
        default['module'], default['node_type'] = params['node_type'].split('/')

        # Process inputs
        topics_in = []
        for key, value in default['inputs'].items():
            assert key in params['inputs'], 'Received unknown %s "%s". Check under "%s" in "%s.yaml" inside ROS package "%s/config".' % ('input', key, 'inputs', config_name, package_name)
            p_in = params['inputs'][key]
            args = dict()
            args['name'], args['address'] = key, value
            args['msg_module'], args['msg_type'] = p_in['msg_type'].split('/')
            args['repeat'] = p_in['repeat']
            if 'rate' in p_in:
                args['rate'], args['is_reactive'] = p_in['rate'], False
            else:
                args['is_reactive'] = True
            if 'input_converters' in default and key in default['input_converters']:
                args['converter'] = default['input_converters'][key]
            # if 'converter_type' in p_in:
            #     args['converter_module'], args['converter'] = p_in['converter_type'].split('/')
            if 'gym_space' in p_in:
                args['gym_space'] = p_in['gym_space']
            n = RxInput(**args)
            topics_in.append(n)
        del default['inputs']

        # Process outputs
        topics_out = []
        if 'outputs' in default:
            for key, value in default['outputs'].items():
                assert key in params['outputs'], ('Received unknown %s "%s". Check under "%s" in "%s.yaml" inside ROS package "%s/config".' % ('output', key, 'outputs', config_name, package_name))
                p_in = params['outputs'][key]
                args = dict()
                args['name'], args['address'] = key, value
                args['msg_module'], args['msg_type'] = p_in['msg_type'].split('/')
                args['rate'] = default['rate']
                if 'output_converters' in default and key in default['output_converters']:
                    args['converter'] = default['output_converters'][key]
                if 'gym_space' in p_in:
                    args['gym_space'] = p_in['gym_space']
                n = RxOutput(**args)
                topics_out.append(n)
            del default['outputs']

        feedthrough_in = []
        if 'feedthroughs' in default:
            for key, value in default['feedthroughs'].items():
                assert key in params['feedthroughs'], 'Received unknown %s "%s". Check under "%s" in "%s.yaml" inside ROS package "%s/config".' % ('feedthrough', key, 'feedthroughs', config_name, package_name)
                p_in = params['feedthroughs'][key]
                args = dict()
                args['feedthrough_to'], args['address'] = key, value
                args['msg_module'], args['msg_type'] = p_in['msg_type'].split('/')
                args['is_reactive'] = True
                if 'feedthrough_converters' in default and key in default['feedthrough_converters']:
                    args['converter'] = default['feedthrough_converters'][key]
                n = RxFeedthrough(**args)
                feedthrough_in.append(n)
            del default['feedthroughs']

        states_in = []
        if 'states' in default:
            for key, value in default['states'].items():
                assert key in params['states'], 'Received unknown %s "%s". Check under "%s" in "%s.yaml" inside ROS package "%s/config".' % ('state', key, 'states', config_name, package_name)
                p_in = params['states'][key]
                args = dict()
                args['name'], args['address'] = key, value
                args['msg_module'], args['msg_type'] = p_in['msg_type'].split('/')
                if 'state_converters' in default and key in default['state_converters']:
                    args['converter'] = default['state_converters'][key]
                n = RxState(**args)
                states_in.append(n)
            del default['states']

        # Calculate properties
        default['topics_out'] = [i.get_params(ns=ns) for i in topics_out]
        default['topics_in'] = [i.get_params(ns=ns) for i in topics_in]
        default['feedthrough_in'] = [i.get_params(ns=ns) for i in feedthrough_in]
        default['states_in'] = [i.get_params(ns=ns) for i in states_in]

        # Create rate dictionary with outputs
        chars_ns = len(ns)+1
        rate_dict = dict()
        for i in default['topics_out']:
            address = i['address'][chars_ns:]
            rate_dict[address] = {'rate': i['rate']}
        assert name not in rate_dict, 'Cannot use the same output topic name for a node (%s).' % name

        # Put parameters in node namespace
        # TODO: WATCH OUT, ORDER OF DICT KEY MATTERS!
        node_params = {name: default}

        # Add output rates in separate namespace
        node_params.update(rate_dict)
        return node_params


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

    @classmethod
    def create(cls, name: str, package_name: str, object_name: str, **kwargs):

        # Load yaml from config file
        params = load_yaml(package_name, object_name)

        # Replace default arguments
        for key, item in kwargs.items():
            if key in params['default'].keys():
                params['default'][key] = item
        return cls(name, params)

    def get_component(self, component, cname=None, **kwargs):
        name = self.name

        # If no cname is provided, add all components
        if cname is None:
            cname = self.params['default'][component]

        # Verify that cname is a list
        if not isinstance(cname, list):
            cname = [cname]

        # Verify that all components are contained in defined components
        for i in cname:
            assert i in self.params['default'][component], '"%s" not included in the %s of object "%s".' % (i, component, name)

        # Loop over components
        cdict = dict()
        cdict[name] = dict()
        for i in cname:
            params = self.params[component][i].copy()

            # Create converter
            gym_space = params['gym_space']
            args = params['gym_space'].copy()
            del args['converter_type']
            converter_cls = get_attribute_from_module(*gym_space['converter_type'].split('/'))
            # todo: Check if converter is string, then do not reload converter_cls when initializing rxpipeline
            converter = converter_cls(**args)

            # Create rxobject
            if component in ['actuators']:
                args = dict()
                args['name'], args['address'] = ('%s' % i, '%s/%s/%s' % (name, component, i))
                args['msg_module'], args['msg_type'] = params['msg_type'].split('/')
                args['rate'] = None  # todo: overwrite with environment rate
                args['converter'] = converter
                rxobject = RxOutput(**args)
            elif component in ['sensors', 'states']:
                # assert 'repeat' in kwargs, 'Missing argument"%s". Must be defined when converting sensors and/or states to observations.' % 'repeat'
                args = dict()
                args['name'], args['address'] = ('%s' % i, '%s/%s/%s' % (name, component, i))
                args['msg_module'], args['msg_type'] = params['msg_type'].split('/')
                args['repeat'] = kwargs['repeat'] if 'repeat' in kwargs else 'all'  # todo: change to last?
                args['converter'] = converter
                rxobject = RxInput(**args)

            # Define component
            cdict[name][i] = (None, rxobject, converter)

        return cdict

    def get_action(self, cname=None, **kwargs):
        return self.get_component('actuators', cname, **kwargs)

    def get_observation(self, cname=None, **kwargs):
        return self.get_component('sensors', cname, **kwargs)

    def get_state(self, cname=None, **kwargs):
        return self.get_component('states', cname, **kwargs)

    def get_params(self, ns, bridge):
        params = self.params.copy()
        name = self.name

        # Check if the bridge name is not contained in the config's (.yaml) default parameters
        assert bridge not in params['default'], 'Cannot have a bridge (%s) as a default object parameter int the config (.yaml) of object (%s).' % (bridge, self.name)
        assert 'node_names' not in params['default'], 'Cannot have a keyword (%s) as a default object parameter int the config (.yaml) of object (%s).' % ('node_names', self.name)
        assert bridge in params, 'The config (.yaml) of object (%s) does not provide support for the selected bridge (%s).' % (self.name, bridge)

        # Create sensors
        component = 'sensors'
        sensors = []
        for i in params['default'][component]:
            assert i in params[bridge][component], '%s not defined for bridge %s and component "%s".' % (i, bridge, component)
            p_env = params[component][i]
            p_bridge = params[bridge][component][i]

            # Prepare node arguments
            node_name = '%s/nodes/%s/%s' % (name, component, i)
            package, config_name = p_bridge['node_config'].split('/')
            args = p_bridge.copy()
            args.pop('node_config')
            args['rate'] = p_env['rate']

            # Define node outputs mapping
            for key, val in args['outputs'].items():
                args['outputs'][key] = '%s/%s' % (name, val)

            # Define node inputs mapping
            args['inputs'] = {'tick': 'bridge/tick'}
            if 'inputs' in p_bridge:
                # First, prepend node name to inputs
                for key, val in p_bridge['inputs'].items():
                    p_bridge['inputs'][key] = '%s/%s' % (name, val)
                args['inputs'].update(p_bridge['inputs'])

            # Create node
            node = RxNodeParams.create(node_name, package, config_name, **args)
            sensors.append(node)

        # Create actuators
        component = 'actuators'
        actuators = []
        for i in params['default'][component]:
            assert i in params[bridge][component], '%s not defined for bridge %s and component "%s".' % (i, bridge, component)
            p_env = params[component][i]
            p_bridge = params[bridge][component][i]

            # Prepare node arguments
            node_name = '%s/nodes/%s/%s' % (name, component, i)
            package, config_name = p_bridge['node_config'].split('/')
            args = p_bridge.copy()
            args.pop('node_config')
            args['rate'] = p_env['rate']

            # Define node outputs mapping
            args['outputs'] = dict()
            for key, val in args['inputs'].items():
                args['outputs'][key] = '%s/%s/applied' % (name, val)

            # Define node inputs mapping
            args['inputs'] = {'tick': 'bridge/tick'}
            if 'inputs' in p_bridge:
                # First, prepend node name to inputs
                for key, val in p_bridge['inputs'].items():
                    p_bridge['inputs'][key] = '%s/%s' % (name, val)
                args['inputs'].update(p_bridge['inputs'])

            # Create node
            node = RxNodeParams.create(node_name, package, config_name, **args)
            actuators.append(node)

        # Create sensors
        component = 'states'
        states = []
        for i in params['default'][component]:
            assert i in params[bridge][component], '%s not defined for bridge %s and component "%s".' % (i, bridge, component)
            p_env = params[component][i]
            p_bridge = params[bridge][component][i]

            # Prepare node arguments
            node_name = '%s/nodes/%s/%s' % (name, component, i)
            package, config_name = p_bridge['node_config'].split('/')
            args = p_bridge.copy()
            args.pop('node_config')
            args['rate'] = p_env['rate']

            # Define node outputs mapping
            for key, val in args['outputs'].items():
                args['outputs'][key] = '%s/%s' % (name, val)

            # Define node inputs mapping
            args['inputs'] = {'tick': 'bridge/tick'}
            if 'inputs' in p_bridge:
                # First, prepend node name to inputs
                for key, val in p_bridge['inputs'].items():
                    p_bridge['inputs'][key] = '%s/%s' % (name, val)
                args['inputs'].update(p_bridge['inputs'])

            # Create node
            node = RxNodeParams.create(node_name, package, config_name, **args)
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


######## NODE IMPLEMENTATIONS ########

class RxBridgeParams(Params):
    def __init__(self,
                 rate: float = 10,
                 num_substeps: int = 10,
                 launch_locally: bool = True,
                 single_process: bool = False,
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

        # IMPORTANT! Ensure that first output topic is 'tick'.
        bridge = RxNodeParamsOld(name=self.name, node_type=self.node_type, module=self.module,
                                 topics_in=[RxInput('tick', 'bridge/tick', 'UInt64')],
                                 topics_out=[RxOutput('tick', 'bridge/tick', 'UInt64', rate=self.rate)])

        params = bridge.get_params(ns=ns)
        params[self.name].update(bridge_params)
        # TODO: WATCH OUT, ORDER OF DICT KEY MATTERS!
        return params


class RxNodeParamsOld(Params):
    def __init__(self,
                 name: str,
                 node_type: str,
                 module: str,
                 topics_out: List[RxOutput],
                 topics_in: List[RxInput],
                 feedthrough_in: List[RxFeedthrough] = [],
                 states_in: List[RxState] = [],
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
        super(RxNodeParamsOld, self).__init__(**kwargs)

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


# class ProcessNode(RxNodeParamsOld):
#     def __init__(self,
#                  name: str,
#                  inputs: List,
#                  outputs: List,
#                  rate: int,
#                  launch_locally: bool = True,
#                  single_process: bool = False,
#                  inputs_converter: List = None,
#                  outputs_converter: List = None,
#                  ):
#         node_type = 'ProcessNode'
#         module = 'eagerx_core.node'
#
#         # Create topics_in
#         topics_in = []
#         for i in range(len(inputs)):
#             args = dict()
#             args['name'], args['address'] = inputs[i].split('/')[-1], inputs[i]
#             args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
#             args['repeat'], args['is_reactive'] = 'all', 'True'
#             if inputs_converter:
#                 args['converter_module'], args['converter'] = inputs_converter[i].split('/')
#             n = RxInput(**args)
#             topics_in.append(n)
#
#         # Create topics_in
#         topics_out = []
#         for i in range(len(outputs)):
#             args = dict()
#             args['name'], args['address'] = outputs[i].split('/')[-1], outputs[i]
#             args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
#             args['rate'] = rate
#             if outputs_converter:
#                 args['converter_module'], args['converter'] = outputs_converter[i].split('/')
#             n = RxOutput(**args)
#             topics_out.append(n)
#
#         super(ProcessNode, self).__init__(name=name, node_type=node_type, module=module,
#                                           launch_locally=launch_locally, single_process=single_process,
#                                           topics_in=topics_in, topics_out=topics_out)
#
#     def get_action(self, cname=None):
#         component = 'input'
#         params = self.__dict__.copy()
#         name = self.name
#
#         # Collect all input names & addresses
#         cnames = []
#         addresses = []
#         for i in params['topics_in']:
#             cnames.append(i.name)
#             addresses.append(i.address)
#
#         # If no cname is provided, add all components
#         if cname is None:
#             cname = cnames
#         else:
#             if not isinstance(cname, list):
#                 assert cname in cnames, '"%s" not included as %s of node "%s".' % (cname, component, name)
#                 cname = [cname]
#             else:
#                 for i in cname:
#                     assert i in cnames, '"%s" not included as %s of node "%s".' % (i, component, name)
#
#         # Loop over components
#         cdict = dict()
#         cdict[name] = dict()
#         # for id, i in enumerate(cname):
#         #     # Create converter
#         #     gym_space = params['gym_space']
#         #     args = params['gym_space'].copy()
#         #     del args['converter_type']
#         #     converter_cls = get_attribute_from_module(*gym_space['converter_type'].split('/'))
#         #     converter = converter_cls(**args)
#
#
# class RealResetNode(RxNodeParamsOld):
#     def __init__(self,
#                  name: str,
#                  inputs: List[Tuple],
#                  outputs: List[Tuple],
#                  states: List[Tuple],
#                  feedthrough: List[Tuple],
#                  rate: int,
#                  launch_locally: bool = True,
#                  single_process: bool = False,
#                  inputs_converter: List = None,
#                  outputs_converter: List = None,
#                  states_converter: List = None,
#                  feedthrough_converter: List = None,
#                  **kwargs):
#         node_type = 'RealResetNode'
#         module = 'eagerx_core.node'
#
#         # Create topics_in
#         topics_in = []
#         for i in range(len(inputs)):
#             args = dict()
#             args['name'], args['address'] = inputs[i]
#             args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
#             args['repeat'], args['is_reactive'] = 'all', 'True'
#             if inputs_converter:
#                 args['converter_module'], args['converter'] = inputs_converter[i].split('/')
#             n = RxInput(**args)
#             topics_in.append(n)
#
#         # Create topics_in
#         topics_out = []
#         for i in range(len(outputs)):
#             args = dict()
#             args['name'], args['address'] = outputs[i]
#             args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
#             args['rate'] = rate
#             if outputs_converter:
#                 args['converter_module'], args['converter'] = outputs_converter[i].split('/')
#             n = RxOutput(**args)
#             topics_out.append(n)
#
#         # Create feedthrough_in
#         feedthrough_in = []
#         for i in range(len(feedthrough)):
#             args = dict()
#             args['address'], args['feedthrough_to'] = feedthrough[i]
#             assert args['feedthrough_to'] in outputs[i], 'The output (%s) we feedthrough to address (%s) is not defined.' % (args['feedthrough_to'], args['address'])
#             args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
#             args['is_reactive'] = True
#             if feedthrough_converter:
#                 args['converter_module'], args['converter'] = feedthrough_converter[i].split('/')
#             n = RxFeedthrough(**args)
#             feedthrough_in.append(n)
#
#         # Create states_in
#         states_in = []
#         for i in range(len(states)):
#             args = dict()
#             args['name'], args['address'] = states[i]
#             args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
#             if states_converter:
#                 args['converter_module'], args['converter'] = states_converter[i].split('/')
#             n = RxState(**args)
#             states_in.append(n)
#
#         super(RealResetNode, self).__init__(name=name, node_type=node_type, module=module,
#                                             launch_locally=launch_locally, single_process=single_process,
#                                             topics_in=topics_in, topics_out=topics_out, feedthrough_in=feedthrough_in,
#                                             states_in=states_in, **kwargs)
#
#
# class SimActuatorNode(RxNodeParamsOld):
#     def __init__(self,
#                  name: str,
#                  outputs: List[Tuple],
#                  rate: int,
#                  inputs: List[Tuple] = None,
#                  inputs_converter: List = None,
#                  outputs_converter: List = None,
#                  ):
#         node_type = 'ProcessNode'
#         module = 'eagerx_core.node'
#         launch_locally = True
#         single_process = True
#
#         # Create topics_in
#         topics_in = [RxInput('tick', 'bridge/tick', 'UInt64')]
#         for i in range(len(inputs)):
#             args = dict()
#             args['name'], args['address'] = inputs[i]
#             args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
#             args['repeat'], args['is_reactive'] = 'all', 'True'
#             if inputs_converter:
#                 args['converter_module'], args['converter'] = inputs_converter[i].split('/')
#             n = RxInput(**args)
#             topics_in.append(n)
#
#         # Create topics_in
#         topics_out = []
#         for i in range(len(outputs)):
#             args = dict()
#             args['name'], args['address'] = outputs[i]
#             args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
#             args['rate'] = rate
#             if outputs_converter:
#                 args['converter_module'], args['converter'] = outputs_converter[i].split('/')
#             n = RxOutput(**args)
#             topics_out.append(n)
#
#         super(SimActuatorNode, self).__init__(name=name, node_type=node_type, module=module,
#                                               launch_locally=launch_locally, single_process=single_process,
#                                               topics_in=topics_in, topics_out=topics_out)
#
#
# class SimSensorNode(RxNodeParamsOld):
#     def __init__(self,
#                  name: str,
#                  outputs: List,
#                  rate: int,
#                  inputs: List = None,
#                  inputs_converter: List = None,
#                  outputs_converter: List = None,
#                  ):
#         node_type = 'ProcessNode'
#         module = 'eagerx_core.node'
#         launch_locally = True
#         single_process = True
#
#         # Create topics_in
#         topics_in = [RxInput('tick', 'bridge/tick', 'UInt64')]
#         if inputs:
#             for i in range(len(inputs)):
#                 args = dict()
#                 args['name'], args['address'] = inputs[i]
#                 args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
#                 args['repeat'], args['is_reactive'] = 'all', 'True'
#                 if inputs_converter:
#                     args['converter_module'], args['converter'] = inputs_converter[i].split('/')
#                 n = RxInput(**args)
#                 topics_in.append(n)
#
#         # Create topics_out
#         topics_out = []
#         for i in range(len(outputs)):
#             args = dict()
#             args['name'], args['address'] = outputs[i]
#             args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
#             args['rate'] = rate
#             if outputs_converter:
#                 args['converter_module'], args['converter'] = outputs_converter[i].split('/')
#             n = RxOutput(**args)
#             topics_out.append(n)
#
#         super(SimSensorNode, self).__init__(name=name, node_type=node_type, module=module,
#                                             launch_locally=launch_locally, single_process=single_process,
#                                             topics_in=topics_in, topics_out=topics_out)
#
#
# class SimStateNode(RxNodeParamsOld):
#     def __init__(self,
#                  name: str,
#                  outputs: List,
#                  rate: int,
#                  inputs: List = None,
#                  inputs_converter: List = None,
#                  outputs_converter: List = None,
#                  states_converter: List = None,
#                  ):
#         node_type = 'StateNode'
#         module = 'eagerx_core.node'
#         launch_locally = True
#         single_process = True
#
#         # Create topics_in
#         topics_in = [RxInput('tick', 'bridge/tick', 'UInt64')]
#         if inputs:
#             for i in range(len(inputs)):
#                 args = dict()
#                 args['name'], args['address'] = inputs[i]
#                 args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
#                 args['repeat'], args['is_reactive'] = 'all', 'True'
#                 if inputs_converter:
#                     args['converter_module'], args['converter'] = inputs_converter[i].split('/')
#                 n = RxInput(**args)
#                 topics_in.append(n)
#
#         # Create topics_out
#         topics_out = []
#         for i in range(len(outputs)):
#             args = dict()
#             args['name'], args['address'] = outputs[i]
#             args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
#             args['rate'] = rate
#             if outputs_converter:
#                 args['converter_module'], args['converter'] = outputs_converter[i].split('/')
#             n = RxOutput(**args)
#             topics_out.append(n)
#
#         # Create set_states_in
#         states_in = []
#         for i in range(len(outputs)):
#             args = dict()
#             args['name'], args['address'] = outputs[i]
#             args['msg_type'], args['msg_module'] = 'UInt64', 'std_msgs.msg'
#             if states_converter:
#                 args['converter_module'], args['converter'] = states_converter[i].split('/')
#             n = RxState(**args)
#             states_in.append(n)
#
#         super(SimStateNode, self).__init__(name=name, node_type=node_type, module=module,
#                                            launch_locally=launch_locally, single_process=single_process,
#                                            topics_in=topics_in, topics_out=topics_out, states_in=states_in)
