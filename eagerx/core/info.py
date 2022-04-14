import inspect
from textwrap import indent
from eagerx.core import register
from eagerx.utils.utils import load


def get_info(entity, id, methods=None, no_cls=False, return_msg=False):
    """Get information on the entity's registered function"""
    REGISTRY = register.REGISTRY
    TYPE_REGISTER = register.TYPE_REGISTER

    # Get the registered cls
    try:
        cls = load(REGISTRY[entity][id]["cls"])
    except KeyError:
        if entity not in REGISTRY:
            msg = f"No entity {entity.__qualname__} was registered."
        else:
            msg = f"No entity '{entity.__qualname__}' with entity_id='{id}' was registered."
        raise KeyError(msg)

    # Determine methods to print info on
    if methods is None:
        methods = list(entity.INFO.keys())
    elif isinstance(methods, str):
        methods = [methods]
    assert isinstance(
        methods, list
    ), "Incorrect specification. Please provide the method as type(methods) =`List[str]`or `str`."

    # Create base info message
    tab = "   "
    msg = f"Registered entity_id=`{id}`:\n"
    msg += indent(f"entity_type: `{entity.__qualname__}`\n", tab)
    msg += indent(f"module: `{cls.__module__}`\n", tab)
    msg += indent(f"file: `{inspect.getfile(cls)}`\n", tab)
    msg += "\n"

    # Objects: Add supported bridges
    if entity.__qualname__ == "Object":
        if len(REGISTRY[entity][id].keys()) == 2:
            bridge_msg = "Supported bridges: <Nothing registered>\n"
        else:
            bridge_msg = "Supported bridges:\n"
            for bridge_id in REGISTRY[entity][id].keys():
                if bridge_id in ["spec", "cls"]:
                    continue
                bridge_msg += indent(f"{bridge_id}\n", tab[2:] + "- ")

        msg += bridge_msg + "\n"

    # Converters: Add conversions
    if entity.__qualname__ in ["Converter", "SpaceConverter", "Processor"]:
        conv_msg = "Supported message types:\n"
        for i in ("MSG_TYPE_A", "MSG_TYPE_B", "MSG_TYPE"):
            try:
                conv_msg += indent(f"{i}: {getattr(cls, i)}\n", tab[2:] + "- ")
            except AttributeError:
                pass
        msg += conv_msg + "\n"

    # Make spec message
    method_fn = cls.spec
    sig = inspect.signature(method_fn)
    arg_entity_id = inspect.Parameter("entity_id", inspect.Parameter.POSITIONAL_OR_KEYWORD, annotation=str)
    sig = sig.replace(parameters=(arg_entity_id,) + tuple(sig.parameters.values())[1:])
    msg += f'Make this spec with (use `entity_id: str = "{id}"`):\n'
    make_msg = f"spec = {entity.__qualname__}.make{sig}\n"
    msg += indent(make_msg + "\n", tab)

    # Generate class
    if not no_cls:
        msg += f"class {cls.__qualname__}({entity.__qualname__}):\n"
        msg += indent(cls.__doc__ + "\n\n", tab) if isinstance(cls.__doc__, str) else ""

        # Expand info message
        for method, register_fns in entity.INFO.items():
            if method not in methods:
                continue
            method_fn = getattr(cls, method)
            sig = inspect.signature(method_fn)
            method_msg = f"{method}{sig}:\n"

            for r in register_fns:
                try:
                    args = TYPE_REGISTER[id][r]
                    if len(args) == 0:
                        registered_msg = f"{r}: <Nothing registered>\n"
                    else:
                        registered_msg = f"{r}:\n"
                    for name, value in args.items():
                        registered_msg += indent(f"{name}: {value}\n", tab[2:] + "- ")
                except KeyError:
                    registered_msg = f"{r}: <Nothing registered>\n"
                method_msg += indent(registered_msg, tab)

            if isinstance(method_fn.__doc__, str):
                docs_msg = "docs:\n"
                docs_msg += indent(method_fn.__doc__ + "\n", tab)
            else:
                docs_msg = "docs: <Not available>\n"
            method_msg += indent(docs_msg, tab)

            msg += indent(method_msg + "\n", tab)

    # Print message
    if return_msg:
        return msg
    else:
        print(msg)
