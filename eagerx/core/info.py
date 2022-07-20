import inspect
import functools
from textwrap import indent
from eagerx.core import register
from eagerx.core.entities import Object


def get_info(cls, methods=None, no_cls=False, return_msg=False):
    """Get information on the entity's registered function"""
    REGISTRY = register.REGISTRY
    TYPE_REGISTER = register.TYPE_REGISTER

    id = cls.__module__ + "/" + cls.__qualname__

    # Determine methods to print info on
    if methods is None:
        methods = list(cls.INFO.keys())
    elif isinstance(methods, str):
        methods = [methods]
    assert isinstance(
        methods, list
    ), "Incorrect specification. Please provide the method as type(methods) =`List[str]`or `str`."

    # Create base info message
    tab = "   "
    msg = indent(f"entity_type: `{cls.__qualname__}`\n", tab)
    msg += indent(f"module: `{cls.__module__}`\n", tab)
    msg += indent(f"file: `{inspect.getfile(cls)}`\n", tab)
    msg += "\n"

    # Objects: Add supported engines
    if issubclass(cls, Object) and id in REGISTRY:
        if len(REGISTRY[id].keys()) == 0:
            engine_msg = "Supported engines: <Nothing registered>\n"
        else:
            engine_msg = "Supported engines:\n"
            for engine_id in REGISTRY[id].keys():
                if engine_id in ["make", "cls"]:
                    continue
                engine_msg += indent(f"{engine_id}\n", tab[2:] + "- ")

        msg += engine_msg + "\n"

    # Make spec message
    method_fn = cls.make
    sig = inspect.signature(method_fn)
    msg += "Make this spec with:\n"
    make_msg = f"spec = {method_fn.__qualname__}{sig}\n"
    msg += indent(make_msg + "\n", tab)

    # Generate class
    if not no_cls:
        msg += f"class {cls.__qualname__}:\n"
        msg += indent(cls.__doc__ + "\n\n", tab) if isinstance(cls.__doc__, str) else ""

        # Expand info message
        for method, register_fns in cls.INFO.items():
            if method not in methods:
                continue
            method_fn = getattr(cls, method)
            sig = inspect.signature(method_fn)
            method_msg = f"{method}{sig}:\n"

            # Replace if wrapped
            if hasattr(method_fn, "__wrapped__") and isinstance(method_fn.__wrapped__, functools.partial):
                method_fn = method_fn.__wrapped__.func

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
