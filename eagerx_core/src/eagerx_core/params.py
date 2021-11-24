from typing import Dict
from copy import deepcopy
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
                 rate: float = None,
                 space_converter: Dict = None,
                 delay: float = 0.0,
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
        params['address'] = '/'.join(filter(None, [ns, params['address']]))
        return params


class RxOutput(Params):
    def __init__(self,
                 name: str,
                 address: str,
                 msg_type: str,
                 rate: float,
                 msg_module: str = 'std_msgs.msg',
                 converter: Dict = None,
                 space_converter: Dict = None,
                 start_with_msg: bool = False
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
        params['address'] = '/'.join(filter(None, [ns, params['address']]))
        return params


class RxFeedthrough(Params):
    def __init__(self,
                 address: str,
                 msg_type: str,
                 feedthrough_to: str,
                 repeat: str = 'all',
                 msg_module: str = 'std_msgs.msg',
                 converter: Dict = None,
                 is_reactive: bool = True,
                 space_converter: Dict = None,
                 delay: float = 0.0
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
        params['address'] = '/'.join(filter(None, [ns, params['address']]))
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
        params['address'] = '/'.join(filter(None, [ns, params['address']]))
        return params


class RxSimState(Params):
    def __init__(self,
                 name: str,
                 address: str,
                 state: Dict,
                 msg_type: str,
                 msg_module: str = 'std_msgs.msg',
                 converter: Dict = None,
                 space_converter: Dict = None
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
        super(RxSimState, self).__init__(**kwargs)

        # Calculate other parameters based on previously defined attributes.

        # Error check the parameters here.

    def get_params(self, ns=''):
        params = self.__dict__.copy()
        params['address'] = '/'.join(filter(None, [ns, params['address']]))
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

    @classmethod
    def create(cls, name: str, package_name: str, config_name: str, **kwargs):
        # default arguments, not specified in node_name.yaml
        ignored_yaml_args = []

        # Load yaml from config file
        params = load_yaml(package_name, config_name)

        # Add feedthrough entries for each output if node is a reset node (i.e. when it has a target)
        if 'targets' in params:
            params['feedthroughs'] = dict()
            for cname in params['outputs']:
                params['feedthroughs'][cname] = dict()

        # Re-direct dict entries (converters, addresses, delays)
        # Will remove re-directed dicts.
        keys_to_pop = []
        for key, entry in kwargs.items():
            if isinstance(entry, dict):
                if key in ['inputs', 'outputs', 'states', 'targets', 'feedthroughs']:
                    component = key
                    clist = []
                    for cname, address in entry.items():
                        params[component][cname]['address'] = address
                        clist.append(cname)
                    kwargs.update({component: clist})
                if key in ['input_converters', 'output_converters', 'state_converters', 'feedthrough_converters', 'target_converters']:
                    component = key.split('_')[0] + 's'
                    for cname, converter in entry.items():
                        params[component][cname]['converter'] = converter
                    keys_to_pop.append(key)
                if key in ['delays']:
                    component = 'inputs'
                    for cname, delay in entry.items():
                        params[component][cname]['delay'] = delay
                    keys_to_pop.append(key)
        [kwargs.pop(key) for key in keys_to_pop]

        # Replace default arguments
        for key, entry in kwargs.items():
            if key in ['color', 'print_mode', 'log_level']:
                params['default'][key] = entry
            assert key in params['default'], 'Received unknown argument "%s". Check under "default" in "%s.yaml" inside ROS package "%s/config" for all possible arguments.' % (key, config_name, package_name)
            params['default'][key] = entry

        # Check if all arguments are specified
        for key, value in params['default'].items():
            if key in ignored_yaml_args: continue
            assert value is not None, 'Missing argument "%s". Check under "default" in "%s.yaml" inside ROS package "%s/config" for all required arguments.' % (key, config_name, package_name)

        # Check reserved keywords
        assert [n not in name.split('/') for n in ['env', 'rate', 'bridge']], 'Node name "%s" not allowed. Names containing "%s" are reserved.' % (name, ['env', 'rate', 'bridge'])
        assert 'name' not in params['default'], 'Argument "name" is reserved. Modify name in "%s.yaml" inside ROS package "%s/config" for all required arguments.' % (config_name, package_name)
        assert 'package_name' not in params['default'], 'Argument "package_name" is reserved. Modify name in "%s.yaml" inside ROS package "%s/config" for all required arguments.' % (config_name, package_name)
        assert 'config_name' not in params['default'], 'Argument "config_name" is reserved. Modify name in "%s.yaml" inside ROS package "%s/config" for all required arguments.' % (config_name, package_name)
        params['default']['name'] = name
        params['default']['package_name'] = package_name
        params['default']['config_name'] = config_name
        return cls(name, params)

    def get_params(self, ns='', in_object=False):
        params = deepcopy(self.params)
        default = deepcopy(params['default'])
        name = self.name
        package_name = default['package_name']
        config_name = default['config_name']
        default['module'], default['node_type'] = params['node_type'].split('/')

        # Process inputs
        inputs = []
        if 'inputs' in default:
            for cname in default['inputs']:
                assert cname in params['inputs'], 'Received unknown %s "%s". Check under "%s" in "%s.yaml" inside ROS package "%s/config".' % ('input', cname, 'inputs', config_name, package_name)
                params['inputs'][cname]['msg_module'], params['inputs'][cname]['msg_type'] = params['inputs'][cname]['msg_type'].split('/')
                if 'rate' in params['inputs'][cname]:
                    is_reactive = False
                else:
                    is_reactive = True
                n = RxInput(name=cname, is_reactive=is_reactive, **params['inputs'][cname])
                inputs.append(n)

        # Process outputs
        outputs = []
        if 'outputs' in default:
            for cname in default['outputs']:
                assert cname in params['outputs'], ('Received unknown %s "%s". Check under "%s" in "%s.yaml" inside ROS package "%s/config".' % ('output', cname, 'outputs', config_name, package_name))
                params['outputs'][cname]['msg_module'], params['outputs'][cname]['msg_type'] = params['outputs'][cname]['msg_type'].split('/')
                if in_object and 'sensors' in name.split('/'):  # If node is part of object, only use name of node (e.g. obj/sensors/out_1)
                    address = name
                else:
                    address = '%s/outputs/%s' % (name, cname)
                n = RxOutput(name=cname, rate=default['rate'], address=address, **params['outputs'][cname])
                outputs.append(n)

        states = []
        if 'states' in default:
            for cname in default['states']:
                assert cname in params['states'], 'Received unknown %s "%s". Check under "%s" in "%s.yaml" inside ROS package "%s/config".' % ('state', cname, 'states', config_name, package_name)
                params['states'][cname]['msg_module'], params['states'][cname]['msg_type'] = params['states'][cname]['msg_type'].split('/')
                if 'address' in params['states'][cname]: # if 'env/supervisor', the state address is pre-defined (like an input)
                    assert not in_object, 'Cannot pre-specify a state address for state "%s" when you use the node inside an object.' % cname
                    n = RxState(name=cname, **params['states'][cname])
                else:
                    address = '%s/states/%s' % (name, cname)
                    n = RxState(name=cname, address=address, **params['states'][cname])
                states.append(n)

        targets = []
        if 'targets' in default:
            for cname in default['targets']:
                assert cname in params['targets'], 'Received unknown %s "%s". Check under "%s" in "%s.yaml" inside ROS package "%s/config".' % ('target', cname, 'targets', config_name, package_name)
                params['targets'][cname]['msg_module'], params['targets'][cname]['msg_type'] = params['targets'][cname]['msg_type'].split('/')
                n = RxState(name=cname, **params['targets'][cname])
                targets.append(n)

        feedthroughs = []
        if 'feedthroughs' in params:
            for cname in default['outputs']:
                # Add output details (msg_type, space_converter) to feedthroughs
                assert cname in params['feedthroughs'], 'Feedthrough "%s" must directly correspond to a selected output. Check under "outputs" in "%s.yaml" inside ROS package "%s/config" for all outputs.' % (cname, config_name, package_name)
                msg_module, msg_type = params['outputs'][cname]['msg_module'], params['outputs'][cname]['msg_type']
                if 'space_converter' in params['outputs'][cname]:
                    space_converter = params['outputs'][cname]['space_converter']
                else:
                    space_converter = None
                n = RxFeedthrough(feedthrough_to=cname, msg_type=msg_type, msg_module=msg_module, space_converter=space_converter, **params['feedthroughs'][cname])
                feedthroughs.append(n)

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
            rate_dict[address] = i['rate']  # {'rate': i['rate']}
        # assert name not in rate_dict, 'Cannot use the same output topic name for a node (%s).' % name

        # Put parameters in node namespace
        # TODO: WATCH OUT, ORDER OF DICT KEY MATTERS!
        node_params = {name: default, 'rate': rate_dict}
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
    def create(cls, name: str, package_name: str, config_name: str, **kwargs):
        # default arguments, not specified in node_name.yaml
        ignored_yaml_args = []

        # Load yaml from config file
        params = load_yaml(package_name, config_name)

        # Replace default arguments
        for key, item in kwargs.items():
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

    def get_params(self, ns, bridge):
        params = deepcopy(self.params)
        name = self.name

        # Check if the bridge name is not contained in the config's (.yaml) default parameters
        assert bridge not in params['default'], 'Cannot have a bridge (%s) as a default object parameter int the config (.yaml) of object (%s).' % (bridge, self.name)
        assert 'node_names' not in params['default'], 'Cannot have a keyword (%s) as a default object parameter int the config (.yaml) of object (%s).' % ('node_names', self.name)
        assert bridge in params, 'The config (.yaml) of object (%s) does not provide support for the selected bridge (%s).' % (self.name, bridge)

        # Create sensors
        component = 'sensors'
        sensors = []
        for cname in params['default'][component]:
            assert cname in params[bridge][component], 'Entry "%s" not defined for bridge "%s" and component "%s".' % (cname, bridge, component)
            p_env = params[component][cname]
            p_bridge = params[bridge][component][cname]

            # Prepare node arguments
            node_name = '%s/%s/%s' % (name, component, cname)
            package, config_name = p_bridge['node_config'].split('/')
            p_bridge.pop('node_config')
            p_bridge['rate'] = p_env['rate']

            # Define node inputs mapping
            inputs_dict = {'tick': 'bridge/outputs/tick'}
            if 'inputs' in p_bridge:
                # First, prepend node name to inputs
                for key, val in p_bridge['inputs'].items():
                    p_bridge['inputs'][key] = '%s/%s' % (name, val)
                inputs_dict.update(p_bridge['inputs'])
            p_bridge['inputs'] = inputs_dict

            # Create node
            node = RxNodeParams.create(node_name, package, config_name, **p_bridge)
            sensors.append(node)

        # Create actuators
        component = 'actuators'
        actuators = []
        for cname in params['default'][component]:
            assert cname in params[bridge][component], 'Entry "%s" not defined for bridge "%s" and component "%s".' % (cname, bridge, component)
            p_env = params[component][cname]
            p_bridge = params[bridge][component][cname]

            # Prepare node arguments
            node_name = '%s/%s/%s' % (name, component, cname)
            package, config_name = p_bridge['node_config'].split('/')
            p_bridge.pop('node_config')
            p_bridge['rate'] = p_env['rate']

            # Define node inputs mapping
            inputs_dict = {'tick': 'bridge/outputs/tick'}
            check_None_trigger = False
            for act_cname, address in p_bridge['inputs'].items():
                if address is None:
                    assert not check_None_trigger, 'Can only have a single (actuator) input (identified with an empty value). Modify "%s.yaml" inside ROS package "%s/config" for all required arguments.' % (params['default']['config_name'], params['default']['package_name'])
                    check_None_trigger = True
                    p_bridge['inputs'][act_cname] = p_env['address']
                else:  # prepend node name to inputs (which have to originate from inside the object)
                    p_bridge['inputs'][act_cname] = '%s/%s' % (name, address)
            assert check_None_trigger, 'Actuator must have at least one (actuator) input (identified with a None value). Modify "%s.yaml" inside ROS package "%s/config" for all required arguments.' % (params['default']['config_name'], params['default']['package_name'])
            inputs_dict.update(p_bridge['inputs'])
            p_bridge['inputs'] = inputs_dict

            # Create node
            node = RxNodeParams.create(node_name, package, config_name, **p_bridge)
            actuators.append(node)

        # Create states
        component = 'states'
        states = []
        for cname in params['default'][component]:
            assert cname in params[bridge][component], 'Entry "%s" not defined for bridge "%s" and component "%s".' % (cname, bridge, component)
            p_env = params[component][cname]
            p_bridge = params[bridge][component][cname]

            # Prepare state arguments
            state = dict()
            for key, value in p_bridge.items():
                if key in ['address', 'converter', 'space_converter']: continue
                state[key] = value
            args = dict(state=state)
            args['name'], args['address'] = '%s/%s/%s' % (name, component, cname), '%s/%s/%s' % (name, component, cname)
            if 'space_converter' in p_env:
                args['space_converter'] = p_env['space_converter']
            args['msg_module'], args['msg_type'] = p_env['msg_type'].split('/')
            s = RxSimState(**args)
            states.append(s)

        nodes = sensors + actuators

        # Create obj parameters
        obj_params = dict()
        obj_params.update(params['default'])

        # Gather node names
        sens_names = ['%s/%s/sensors/%s' % (ns, name, cname) for cname in obj_params['sensors']]
        act_names = ['%s/%s/actuators/%s' % (ns, name, cname) for cname in obj_params['actuators']]
        state_names = ['%s/%s/states/%s' % (ns, name, cname) for cname in obj_params['states']]
        obj_params['node_names'] = sens_names + act_names
        obj_params['state_names'] = state_names
        obj_params['states'] = [s.get_params(ns) for s in states]

        # Remove clutter from parameters
        obj_params[bridge] = params[bridge].copy()
        for c in ('sensors', 'actuators', 'states'):
            if c in obj_params[bridge]:
                del obj_params[bridge][c]
            if c in obj_params:
                if c == 'states': continue
                del obj_params[c]

        return {self.name: obj_params}, nodes


class RxBridgeParams(RxNodeParams):
    @classmethod
    def create(cls, package_name: str, config_name: str, **kwargs):
        return RxNodeParams.create('bridge', package_name, config_name, **kwargs)
