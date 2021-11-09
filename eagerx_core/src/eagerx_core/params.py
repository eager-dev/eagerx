from typing import Dict
from eagerx_core.utils.utils import load_yaml


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
                 space_converter: Dict = None,
                 ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        if not converter:
            del converter

        # If space_converter undefined, remove it
        if not space_converter:
            del space_converter

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
                 space_converter: Dict = None,
                 ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        # If space_converter undefined, remove it
        if not converter:
            del converter

        if not space_converter:
            del space_converter
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
                 space_converter: Dict = None,
                 ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        # If space_converter undefined, remove it
        if not converter:
            del converter

        if not space_converter:
            del space_converter
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
                 space_converter: Dict = None,
                 ):
        # Store parameters as properties in baseclass
        # IMPORTANT! Do not define variables locally you do **not** want to store
        # on the parameter server anywhere before calling the baseclass' constructor.
        # If space_converter undefined, remove it
        if not converter:
            del converter

        if not space_converter:
            del space_converter
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

        # Check that node has at least one input. # todo: bridge initially does not have any inputs...
        # assert len(params['inputs']) > 0, 'Nodes must have at least one input. Node "%s" does not have inputs.' % name

    @classmethod
    def create(cls, name: str, package_name: str, config_name: str, **kwargs):
        # default arguments, not specified in node_name.yaml
        nonlisted_yaml_args = ['input_converters', 'output_converters', 'state_converters', 'feedthrough_converters', 'target_converters']
        ignored_yaml_args = ['inputs', 'states', 'feedthroughs', 'targets']

        # Load yaml from config file
        params = load_yaml(package_name, config_name)

        # Add output details (msg_type, space_converter, etc...) to feedthroughs
        if 'feedthroughs' in params:
            for key, value in params['feedthroughs'].items():
                assert key in params['outputs'], 'Feedthrough "%s" must directly correspond to an output. Check under "outputs" in "%s.yaml" inside ROS package "%s/config" for all outputs.' % (key, config_name, package_name)
                params['feedthroughs'][key] = {'msg_type': params['outputs'][key]['msg_type']}
                if 'space_converter' in params['outputs'][key]:
                    params['feedthroughs'][key]['space_converter'] = params['outputs'][key]['space_converter']

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

    def get_params(self, ns=''):
        params = self.params.copy()
        default = params['default'].copy()
        name = self.name
        package_name = default['package_name']
        config_name = default['config_name']
        default['module'], default['node_type'] = params['node_type'].split('/')

        # Process inputs
        inputs = []
        if 'inputs' in default:
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
                if 'space_converter' in p_in:
                    args['space_converter'] = p_in['space_converter']
                n = RxInput(**args)
                inputs.append(n)
            del default['inputs']

        # Process outputs
        outputs = []
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
                if 'space_converter' in p_in:
                    args['space_converter'] = p_in['space_converter']
                n = RxOutput(**args)
                outputs.append(n)
            del default['outputs']

        states = []
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
                states.append(n)
            del default['states']

        targets = []
        if 'targets' in default:
            for key, value in default['targets'].items():
                assert key in params['targets'], 'Received unknown %s "%s". Check under "%s" in "%s.yaml" inside ROS package "%s/config".' % ('target', key, 'targets', config_name, package_name)
                p_in = params['targets'][key]
                args = dict()
                args['name'], args['address'] = key, value
                args['msg_module'], args['msg_type'] = p_in['msg_type'].split('/')
                if 'target_converters' in default and key in default['target_converters']:
                    args['converter'] = default['target_converters'][key]
                n = RxState(**args)
                targets.append(n)
            del default['targets']

        feedthroughs = []
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
                feedthroughs.append(n)
            del default['feedthroughs']

        # Calculate properties
        default['outputs'] = [i.get_params(ns=ns) for i in outputs]
        default['inputs'] = [i.get_params(ns=ns) for i in inputs]
        default['states'] = [i.get_params(ns=ns) for i in states]
        default['targets'] = [i.get_params(ns=ns) for i in targets]
        default['feedthroughs'] = [i.get_params(ns=ns) for i in feedthroughs]

        # Create rate dictionary with outputs
        chars_ns = len(ns)+1
        rate_dict = dict()
        for i in default['outputs']:
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

            # Get node yaml
            node_yaml = load_yaml(package, config_name)

            # Define node outputs mapping
            args['outputs'] = {}
            for key, val in node_yaml['outputs'].items():
                args['outputs'][key] = '%s/%s/%s' % (name, component, i)

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
            actuator_input = p_bridge['actuator_input']
            args = p_bridge.copy()
            args.pop('node_config')
            args.pop('actuator_input')  # Key 'actuator_input' only relevant for object, so remove from node params
            args['rate'] = p_env['rate']

            # Get node yaml
            # node_yaml = load_yaml(package, config_name)

            # Define node outputs mapping
            assert actuator_input in p_bridge['inputs'], 'The address for actuator "%s" of object "%s" has not been specified!'
            args['outputs'] = dict()
            args['outputs'][actuator_input] = '%s/actuators/%s/applied' % (name, i)

            # Define node inputs mapping
            args['inputs'] = {'tick': 'bridge/tick'}
            if 'inputs' in p_bridge:
                # First, prepend node name to inputs (if not actuator input)
                for key, val in p_bridge['inputs'].items():
                    if key == actuator_input: continue
                    p_bridge['inputs'][key] = '%s/%s' % (name, val)
                args['inputs'].update(p_bridge['inputs'])

            # Create node
            node = RxNodeParams.create(node_name, package, config_name, **args)
            actuators.append(node)

        # Create states
        component = 'states'
        states = []
        for i in params['default'][component]:
            assert i in params[bridge][component], '%s not defined for bridge %s and component "%s".' % (i, bridge, component)
            p_env = params[component][i]
            p_bridge = params[bridge][component][i]

            # Prepare node arguments
            node_name = '%s/nodes/%s/%s' % (name, component, i)
            package, config_name = p_bridge['node_config'].split('/')
            state_input = p_bridge['state_input']
            args = p_bridge.copy()
            args.pop('node_config')
            args.pop('state_input')  # Key 'state_input' only relevant for object, so remove from node params
            args['rate'] = p_env['rate']

            # Get node yaml
            node_yaml = load_yaml(package, config_name)

            # Define node outputs mapping
            args['outputs'] = {}
            for key, val in node_yaml['outputs'].items():
                args['outputs'][key] = '%s/%s/%s' % (name, component, i)

            # Define node inputs mapping
            args['inputs'] = {'tick': 'bridge/tick'}
            if 'inputs' in p_bridge:
                # First, prepend node name to inputs
                for key, val in p_bridge['inputs'].items():
                    p_bridge['inputs'][key] = '%s/%s' % (name, val)
                args['inputs'].update(p_bridge['inputs'])

            # Define node states mapping
            args['states'] = {}
            for key, val in node_yaml['states'].items():
                args['states'][key] = p_bridge['states'][key]
                # args['states'][key] = '%s/%s/%s' % (name, component, i)

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
        obj_params[bridge] = params[bridge].copy()
        for c in ('sensors', 'actuators', 'states'):
            if c in obj_params[bridge]:
                del obj_params[bridge][c]
            if c in obj_params:
                del obj_params[c]

        return {self.name: obj_params}, nodes
