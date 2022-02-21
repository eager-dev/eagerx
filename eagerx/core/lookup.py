from yaml import dump
from eagerx.utils.utils import is_supported_type


supported_types = (str, int, list, float, bool, dict)


def keys_exists(element, *keys):
    """
    Check if *keys (nested) exists in `element` (dict).
    """
    if not isinstance(element, dict):
        raise AttributeError("keys_exists() expects dict as first argument.")
    if len(keys) == 0:
        raise AttributeError("keys_exists() expects at least two arguments, one given.")

    _element = element
    for key in keys:
        try:
            _element = _element[key]
        except KeyError:
            return False
    return True


def get_dict(params, depth):
    for k in depth:
        params = params[k]
    return params


def _is_supported_type(param):
    try:
        is_supported_type(param, supported_types, none_support=True)
    except TypeError:
        if isinstance(param, Lookup):
            param = param.to_dict()
            is_supported_type(param, supported_types, none_support=True)
        else:
            from eagerx.core.specs import EntitySpec

            if isinstance(param, EntitySpec):
                param = param.params
                is_supported_type(param, supported_types, none_support=True)
            else:
                raise


def _convert_type(param):
    if type(param) in supported_types or param is None:
        return param
    else:
        if isinstance(param, Lookup):
            return param.to_dict()
        else:
            from eagerx.core.specs import EntitySpec

            if isinstance(param, EntitySpec):
                return param.params
            else:
                message = f'Type "{type(param)}" is not supported. Only types {supported_types} are supported.'
                raise TypeError(message)


class LookupIterator(object):
    # todo: deal with entity spec.
    def __init__(self, lookup, items=False):
        self._lookup = lookup
        self._items = items
        self._lookup_keys = list(lookup.keys()).copy()
        self._index = 0

    def __iter__(self):
        """Returns the Iterator object"""
        return self

    def __next__(self):
        if not len(self._lookup_keys) == len(self._lookup.keys()):
            raise ValueError("Iterator length not constant while iterating.")
        if self._index < len(self._lookup_keys):
            key = self._lookup_keys[self._index]
            self._index += 1
            if self._items:
                return key, self._lookup[key]
            else:
                return key
        raise StopIteration


class Lookup(object):
    def __init__(self, spec, depth, name=None):
        super(Lookup, self).__setattr__("_spec", spec)
        super(Lookup, self).__setattr__("_depth", depth)
        super(Lookup, self).__setattr__("_name", name)
        super(Lookup, self).__setattr__("_unlocked", False)

    def __setattr__(self, name, value):
        # Check if type is supported
        _is_supported_type(value)
        value = _convert_type(value)  # Convert Lookup & EntitySpec to dicts

        d = get_dict(self._spec._params, self._depth)
        if name in d:
            d[name] = value
        elif self._unlocked:
            d[name] = value
        else:
            message = f"Cannot set new attribute '{name}' when locked. \n"
            message += "The lock is there because new attributes are likely not accepted during initialization. "
            message += "If you are sure it will not break functionality, you can use '.unlock() before setting."
            raise AttributeError(message)

    def __getattr__(self, name):
        if keys_exists(self._spec._params, *self._depth, name):
            new_depth = self._depth.copy() + [name]
            d = get_dict(self._spec._params, new_depth)
            if isinstance(d, dict):
                return Lookup(self._spec, new_depth, self._name)
            else:
                return d
        else:
            try:
                d = get_dict(self._spec._params, self._depth)
                return getattr(d, name)
            except AttributeError as e:
                d = get_dict(self._spec._params, self._depth)
                if self._name:
                    depth_str = f"{self._name}"
                else:
                    depth_str = "params"
                for i in self._depth:
                    depth_str += f".{i}"
                attr_available = ["." + n for n in list(d.keys())]
                message = f"Attribute '{name}' not found. Available attributes in {depth_str}={attr_available}."
                raise AttributeError(message) from e

    def __getitem__(self, name):
        try:
            return self.__getattr__(name)
        except AttributeError:
            raise KeyError(name)

    def __setitem__(self, name, value):
        if name.startswith("__"):
            raise AttributeError("Cannot set magic attribute '{}'".format(name))
        self.__setattr__(name, value)

    def __str__(self):
        d = get_dict(self._spec._params, self._depth)
        repr = f"Params for {self.entry()}: \n\n"
        repr += dump(d)
        return repr

    def __repr__(self):
        d = get_dict(self._spec._params, self._depth)
        repr = f"Params for {self.entry()}: \n\n"
        repr += dump(d)
        return repr

    def __contains__(self, item):
        d = get_dict(self._spec._params, self._depth)
        return item in d

    def __iter__(self):
        return LookupIterator(self, items=False)

    def __enter__(self):
        self._unlock()
        return self

    def __exit__(self, exc_type, exc_value, exc_tb):
        self._lock()

    @property
    def unlocked(self):
        """
        Use as with .unlockes as s: now set attributes to 's' with 's.<attribute>' within the context
        """
        return self

    def _lock(self):
        super(Lookup, self).__setattr__("_unlocked", False)

    def _unlock(self):
        super(Lookup, self).__setattr__("_unlocked", True)

    def update(self, mapping):
        for key, value in mapping.items():
            setattr(self, key, value)
        return self

    def items(self):
        return LookupIterator(self, items=True)

    def entry(self):
        if self._name:
            return tuple([self._name] + self._depth)
        else:
            return tuple(self._depth)

    def to_dict(self):
        return get_dict(self._spec._params, self._depth)
