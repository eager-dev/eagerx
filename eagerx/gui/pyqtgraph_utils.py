import yaml
import inspect
import ast
from functools import wraps, partial
from copy import deepcopy

from pyqtgraph import ComboBox, SpinBox
from pyqtgraph.Qt import QtGui

from eagerx.core import constants
from eagerx.utils.utils import (
    get_attribute_from_module,
    get_module_type_string,
    get_opposite_msg_cls,
)
from eagerx.core.register import REVERSE_REGISTRY


def tryeval(val):
    try:
        val = ast.literal_eval(val)
    except Exception as e:
        if isinstance(e, ValueError):
            pass
        elif isinstance(e, SyntaxError):
            pass
        else:
            raise
    return val


def exception_handler(function_to_decorate):
    wraps(function_to_decorate)

    def exception_handler_wrapper(*args, graph_backup=None, dialog_title="Invalid", **kwargs):
        assert graph_backup is not None, "Graph should be defined for recovering state."
        state_copy = deepcopy(graph_backup._state)
        try:
            return function_to_decorate(*args, **kwargs)
        except Exception as e:
            graph_backup._state = state_copy
            graph_backup.load_state(clear=True)
            error_window = QtGui.QDialog(graph_backup.widget().cwWin)
            error_window.setWindowTitle(dialog_title)
            layout = QtGui.QGridLayout()
            label = QtGui.QLabel(str(e))
            layout.addWidget(label)
            error_window.setLayout(layout)
            error_window.exec_()
            raise

    return exception_handler_wrapper


class NodeCreationDialog(QtGui.QDialog):
    def __init__(self, name, node_type, parent):
        super().__init__(parent)
        self.setWindowTitle("Create {}".format(name))
        self.mapping = {}
        self.node_type = node_type

        signature = node_type["entity_cls"].get_spec(node_type["id"], verbose=False)
        parameters = signature.parameters

        self.layout = QtGui.QGridLayout()
        self.labels = []
        self.widgets = []
        row = 0

        required_args = {}
        optional_args = {}

        for key in parameters.keys():
            if parameters[key].default is inspect._empty:
                required_args[key] = parameters[key]
            else:
                optional_args[key] = parameters[key]

        required_args_label = QtGui.QLabel("Required Arguments")
        self.layout.addWidget(required_args_label, row, 0)
        self.labels.append(required_args_label)
        row += 1

        for key, parameter in required_args.items():
            if key == "name":
                value = name
            elif key == "rate":
                value = 1.0
            else:
                value = None
            self.add_widget(key, value, parameter, row)
            row += 1

        optional_args_label = QtGui.QLabel("Optional Arguments")
        self.layout.addWidget(optional_args_label, row, 0)
        self.labels.append(optional_args_label)
        row += 1

        for key, parameter in optional_args.items():
            value = parameter.default
            self.add_widget(key, value, parameter, row)
            row += 1

        self.setLayout(self.layout)

    def open(self):
        self.exec_()
        return self.mapping

    def add_widget(self, key, value, parameter, row):
        label = QtGui.QLabel(str(parameter).split("=")[0].strip())
        if parameter.annotation is bool:
            value = value if value is not None else True
            items = ["True", "False"]
            widget = ComboBox(items=items, default=str(value))
            widget.activated.connect(partial(self.combo_box_value_changed, key=key, items=items))
        elif parameter.annotation is int:
            widget = SpinBox(value=value, int=True, dec=True)
            widget.sigValueChanged.connect(partial(self.value_changed, key=key))
        elif parameter.annotation is float:
            value = value if value is not None else 0.0
            widget = SpinBox(value=value, dec=True)
            widget.sigValueChanged.connect(partial(self.value_changed, key=key))
        elif parameter.annotation is str:
            widget = QtGui.QLineEdit(str(value))
            widget.textChanged.connect(partial(self.text_changed, key=key))
        elif isinstance(value, bool):
            items = ["True", "False"]
            widget = ComboBox(items=items, default=str(value))
            widget.activated.connect(partial(self.combo_box_value_changed, key=key, items=items))
        elif isinstance(value, int):
            widget = SpinBox(value=value, int=True, dec=True)
            widget.sigValueChanged.connect(partial(self.value_changed, key=key))
        elif isinstance(value, float):
            widget = SpinBox(value=value, dec=True)
            widget.sigValueChanged.connect(partial(self.value_changed, key=key))
        else:
            widget = QtGui.QLineEdit(str(value))
            widget.textChanged.connect(partial(self.text_changed, key=key))
        self.mapping[key] = value
        for grid_object in [label, widget]:
            font = grid_object.font()
            font.setPointSize(12)
            grid_object.setFont(font)
        for grid_object in [label, widget]:
            font = grid_object.font()
            font.setPointSize(12)
            grid_object.setFont(font)
        self.layout.addWidget(label, row, 0)
        self.layout.addWidget(widget, row, 1)
        self.labels.append(label)
        self.widgets.append(widget)

    def combo_box_value_changed(self, int, items, key):
        value = items[int]
        self.mapping[key] = value

    def value_changed(self, widget, key):
        self.mapping[key] = widget.value()

    def text_changed(self, text, key):
        self.mapping[key] = tryeval(text)


class ParamWindow(QtGui.QDialog):
    def __init__(self, node, term=None):
        self.parent = node.graph.widget().cwWin
        super().__init__(self.parent)
        self.node = node
        self.node_type = node.node_type
        self.library = node.graph.library
        self.is_term = term is not None
        self.entity = term if self.is_term else node
        self.entity_type = "term" if self.is_term else "node"

        name = term.terminal_name if self.is_term else node.name
        self.setWindowTitle("Parameters {}".format(name))

        self.layout = QtGui.QGridLayout()
        self.labels = []
        self.widgets = []
        self.params_changed = {}
        row = 0
        for key, value in self.entity.params().items():
            if key in constants.GUI_WIDGETS[self.entity_type]["hide"]["all"]:
                continue
            elif (
                self.node_type in constants.GUI_WIDGETS[self.entity_type]["hide"]
                and key in constants.GUI_WIDGETS[self.entity_type]["hide"][self.node_type]
            ):
                continue
            elif (
                self.is_term
                and self.entity.terminal_type in constants.GUI_WIDGETS[self.entity_type]["hide"]
                and key in constants.GUI_WIDGETS[self.entity_type]["hide"][self.entity.terminal_type]
            ):
                continue
            else:
                self.add_widget(key, value, row)
                row += 1
        if row == 0:
            label = QtGui.QLabel("No parameters to show.")
            self.layout.addWidget(label, row, 0)
        self.setLayout(self.layout)

    def open(self):
        self.exec_()
        valid = False
        while not valid:
            try:
                self.set_params(graph_backup=self.node.graph)
                self.node.graph.load_state(clear=True)
                valid = True
            except Exception:
                self.exec_()

    @exception_handler
    def set_params(self):
        for key, value in self.params_changed.items():
            value = yaml.safe_load(str(value))
            self.entity.set_param(key, value)

    def add_widget(self, key, value, row):
        label = QtGui.QLabel(key)
        if self.is_term and key in "converter":
            button_string = value["converter_type"].split("/")[-1] if "converter_type" in value else "converter"
            widget = QtGui.QPushButton("Edit {}".format(button_string))
            widget.pressed.connect(partial(self.open_converter_dialog, button=widget))
        elif self.is_term and key == "space_converter":
            if self.entity.is_state:
                button_string = (
                    value["converter_type"].split("/")[-1] if "converter_type" in value else "space_converter"
                )
                widget = QtGui.QPushButton("Edit {}".format(button_string))
                widget.pressed.connect(
                    partial(
                        self.open_converter_dialog,
                        button=widget,
                        is_space_converter=True,
                    )
                )
            else:
                widget = QtGui.QLineEdit(str(value))
                widget.setEnabled(False)
        elif key in constants.GUI_WIDGETS[self.entity_type]["items"]:
            items = constants.GUI_WIDGETS[self.entity_type]["items"][key]
            if isinstance(items, dict):
                item_converter = items
                index, items = list(items.values()).index(value), list(items.keys())
                value = items[index]
            else:
                item_converter = None
            widget = ComboBox(items=items, default=str(value))
            widget.activated.connect(
                partial(
                    self.combo_box_value_changed,
                    key=key,
                    items=items,
                    item_converter=item_converter,
                )
            )
        elif isinstance(value, bool):
            items = ["True", "False"]
            widget = ComboBox(items=items, default=str(value))
            widget.activated.connect(partial(self.combo_box_value_changed, key=key, items=items))
        elif isinstance(value, int):
            widget = SpinBox(value=value, int=True, dec=True)
            widget.sigValueChanged.connect(partial(self.value_changed, key=key))
        elif isinstance(value, float):
            widget = SpinBox(value=value, dec=True)
            widget.sigValueChanged.connect(partial(self.value_changed, key=key))
        else:
            widget = QtGui.QLineEdit(str(value))
            widget.textChanged.connect(partial(self.text_changed, key=key))
        if key in constants.GUI_WIDGETS[self.entity_type]["constant"]["all"] or (
            self.node_type in constants.GUI_WIDGETS[self.entity_type]["constant"]
            and key in constants.GUI_WIDGETS[self.entity_type]["constant"][self.node_type]
        ):
            widget.setEnabled(False)
        for grid_object in [label, widget]:
            font = grid_object.font()
            font.setPointSize(12)
            grid_object.setFont(font)
        for grid_object in [label, widget]:
            font = grid_object.font()
            font.setPointSize(12)
            grid_object.setFont(font)
        self.layout.addWidget(label, row, 0)
        self.layout.addWidget(widget, row, 1)
        self.labels.append(label)
        self.widgets.append(widget)

    def combo_box_value_changed(self, int, items, key, item_converter=None):
        value = items[int]
        if item_converter is not None:
            value = item_converter[value]
        self.params_changed[key] = value

    def value_changed(self, widget, key):
        self.params_changed[key] = widget.value()

    def text_changed(self, text, key):
        self.params_changed[key] = tryeval(text)

    def open_converter_dialog(self, button, is_space_converter=False):
        key = "space_converter" if is_space_converter else "converter"
        library = self.node.graph.library
        if "converter" in self.entity.params() and self.entity.params()["converter"] is not None:
            msg_type = get_opposite_msg_cls(self.entity.params()["msg_type"], self.entity.params()["converter"])
        else:
            msg_type = get_attribute_from_module(self.entity.params()["msg_type"])
        msg_type_in, msg_type_out = (msg_type, None) if self.entity.is_input else (None, msg_type)
        is_space_converter = is_space_converter or self.node.node_type in [
            "actions",
            "observations",
        ]
        converter_dialog = ConverterDialog(
            converter=self.entity.params()[key],
            parent=self.parent,
            library=library,
            msg_type_in=msg_type_in,
            msg_type_out=msg_type_out,
            is_space_converter=is_space_converter,
        )
        converter = converter_dialog.open()
        converter_dialog.close()
        self.params_changed[key] = converter
        button_string = converter["converter_type"].split("/")[-1] if "converter_type" in converter else key
        button.setText("Edit {}".format(button_string))


class ConverterDialog(QtGui.QDialog):
    def __init__(
        self,
        converter,
        parent,
        library,
        msg_type_in,
        msg_type_out=None,
        is_space_converter=False,
    ):
        super().__init__(parent)
        self.parent = parent
        self.msg_type_in = msg_type_in
        self.msg_type_out = msg_type_out
        self.is_space_converter = is_space_converter
        self.converter = converter
        self.library = library
        self.setWindowTitle("Converter Parameters")
        self.layout = QtGui.QGridLayout()
        self.labels = []
        self.widgets = []

        converter_class = get_attribute_from_module(self.converter["converter_type"])
        converter_id = REVERSE_REGISTRY[converter_class.spec]
        (
            converter_id,
            available_converters,
            required_args,
            optional_args,
        ) = self.get_parameters(converter_id)

        self.add_widget(
            key="Converter Class",
            value=converter_id,
            parameter=None,
            row=0,
            items=available_converters,
        )
        self.add_argument_widgets(required_args, optional_args)
        self.setLayout(self.layout)

    def open(self):
        self.exec_()
        valid = False
        while not valid:
            try:
                for key, value in self.converter.items():
                    self.converter[key] = yaml.safe_load(str(value))
                get_attribute_from_module(self.converter["converter_type"])
                valid = True
            except Exception as e:
                error_window = QtGui.QDialog(self.parent)
                error_window.setWindowTitle("Invalid Converter")
                layout = QtGui.QGridLayout()
                label = QtGui.QLabel(str(e))
                layout.addWidget(label)
                error_window.setLayout(layout)
                error_window.exec_()
                self.exec_()
        return self.converter

    def add_argument_widgets(self, required_args, optional_args):
        row = 1
        required_args_label = QtGui.QLabel("Required Converter Arguments")
        self.layout.addWidget(required_args_label, row, 0)
        self.labels.append(required_args_label)
        row += 1

        for key, parameter in required_args.items():
            if key in self.converter:
                value = self.converter[key]
            else:
                value = None
            self.add_widget(key, value, parameter, row)
            row += 1

        optional_args_label = QtGui.QLabel("Optional Converter Arguments")
        self.layout.addWidget(optional_args_label, row, 0)
        self.labels.append(optional_args_label)
        row += 1

        for key, parameter in optional_args.items():
            if key in self.converter:
                value = self.converter[key]
            else:
                value = parameter.default
            self.add_widget(key, value, parameter, row)
            row += 1

    def get_parameters(self, converter_id):
        converter = deepcopy(self.converter)

        available_converters = {}
        required_args = {}
        optional_args = {}

        if self.is_space_converter:
            cnvrtr_types = ["SpaceConverter"]
        elif None in [self.msg_type_in, self.msg_type_out]:
            cnvrtr_types = ["BaseConverter", "Processor", "Converter"]
        elif self.msg_type_in == self.msg_type_out:
            cnvrtr_types = ["BaseConverter", "Processor"]
        else:
            cnvrtr_types = ["Converter"]

        for cnvrtr_type in cnvrtr_types:
            if cnvrtr_type not in self.library:
                continue
            for cnvrtr in self.library[cnvrtr_type]:
                cnvrtr_cls = cnvrtr["cls"]
                if cnvrtr_type == "Processor":
                    if not cnvrtr_cls.MSG_TYPE in [self.msg_type_in, self.msg_type_out]:
                        continue
                elif cnvrtr_type in ["Converter", "SpaceConverter"]:
                    if self.msg_type_in is not None and not self.msg_type_in in [
                        cnvrtr_cls.MSG_TYPE_A,
                        cnvrtr_cls.MSG_TYPE_B,
                    ]:
                        continue
                    elif self.msg_type_out is not None and not self.msg_type_out in [
                        cnvrtr_cls.MSG_TYPE_A,
                        cnvrtr_cls.MSG_TYPE_B,
                    ]:
                        continue
                available_converters[cnvrtr["id"]] = {
                    "spec": cnvrtr["entity_cls"].get_spec(cnvrtr["id"], verbose=False),
                    "cls": cnvrtr_cls,
                }

        available_converters_list = list(available_converters.keys())

        if converter_id not in available_converters.keys():
            if "Identity" in available_converters:
                converter_id = "Identity"
            elif len(available_converters_list) > 0:
                converter_id = available_converters_list[0]
            else:
                converter_id = None

        if converter_id is not None:
            parameters = available_converters[converter_id]["spec"].parameters
            for key in parameters.keys():
                if parameters[key].default is inspect._empty:
                    required_args[key] = parameters[key]
                else:
                    optional_args[key] = parameters[key]
            invalid_arguments = []
            for key in converter.keys():
                if key not in parameters.keys():
                    invalid_arguments.append(key)
            for key in invalid_arguments:
                converter.pop(key)

        if converter_id is not None:
            converter["converter_type"] = get_module_type_string(available_converters[converter_id]["cls"])
        else:
            converter = None
        self.converter = converter
        return converter_id, available_converters_list, required_args, optional_args

    def add_widget(self, key, value, parameter, row, items=None):
        if parameter is not None:
            label = QtGui.QLabel(str(parameter).split("=")[0].strip())
        else:
            label = QtGui.QLabel(str(key))
        if parameter is None:
            widget = ComboBox(items=items, default=value)
            widget.activated.connect(partial(self.class_changed, items=items))
        elif parameter.annotation is bool:
            value = value if value is not None else True
            items = ["True", "False"]
            self.converter[key] = value
            widget = ComboBox(items=items, default=str(value))
            widget.activated.connect(partial(self.combo_box_value_changed, key=key, items=items))
        elif parameter.annotation is int:
            value = value if value is not None else 0
            self.converter[key] = value
            widget = SpinBox(value=value, int=True, dec=True)
            widget.sigValueChanged.connect(partial(self.value_changed, key=key))
        elif parameter.annotation is float:
            value = value if value is not None else 0.0
            self.converter[key] = value
            widget = SpinBox(value=value, dec=True)
            widget.sigValueChanged.connect(partial(self.value_changed, key=key))
        elif parameter.annotation is str:
            widget = QtGui.QLineEdit(str(value))
            widget.textChanged.connect(partial(self.text_changed, key=key))
        else:
            widget = QtGui.QLineEdit(str(value))
            widget.textChanged.connect(partial(self.argument_changed, key=key))

        self.layout.addWidget(label, row, 0)
        self.layout.addWidget(widget, row, 1)
        self.labels.append(label)
        self.widgets.append(widget)

    def class_changed(self, int, items):
        converter_id = items[int]

        for label in self.labels[1:]:
            self.layout.removeWidget(label)
            label.close()

        for widget in self.widgets[1:]:
            self.layout.removeWidget(widget)
            widget.close()

        self.labels = self.labels[:2]
        self.widgets = self.widgets[:2]

        _, _, required_args, optional_args = self.get_parameters(converter_id)

        self.add_argument_widgets(required_args, optional_args)

    def argument_changed(self, text, key):
        self.converter[key] = text

    def text_changed(self, text, key):
        self.params[key] = tryeval(text)


class ConnectionDialog(QtGui.QDialog):
    def __init__(self, input_term, output_term, **kwargs):
        self.parent = input_term.node.graph.widget().cwWin
        super().__init__(self.parent)
        self.setWindowTitle("Connection Parameters")
        self.library = input_term.node.graph.library
        self.input_term = input_term
        self.output_term = output_term
        self.layout = QtGui.QGridLayout()
        self.params = {}
        self.labels = []
        self.widgets = []
        row = 0
        params = deepcopy(input_term.params())
        for key, value in kwargs.items():
            if value is not None:
                params[key] = value
        for key, value in params.items():
            if key in inspect.getfullargspec(input_term.node.graph.connect).args:
                self.params[key] = value
                self.add_widget(key, value, row)
                row += 1
        self.setLayout(self.layout)

    def open(self):
        self.exec_()
        for widget in self.widgets:
            widget.setEnabled(False)
        return self.params

    def add_widget(self, key, value, row):
        label = QtGui.QLabel(key)
        if key == "converter":
            button_string = value["converter_type"].split("/")[-1] if "converter_type" in value else "converter"
            widget = QtGui.QPushButton("Edit {}".format(button_string))
            widget.pressed.connect(partial(self.open_converter_dialog, button=widget))
        elif isinstance(value, bool):
            items = ["True", "False"]
            widget = ComboBox(items=items, default=str(value))
            widget.activated.connect(partial(self.combo_box_value_changed, key=key, items=items))
        elif isinstance(value, int):
            widget = SpinBox(value=value, int=True, dec=True)
            widget.sigValueChanged.connect(partial(self.value_changed, key=key))
        elif isinstance(value, float):
            widget = SpinBox(value=value)
            widget.sigValueChanged.connect(partial(self.value_changed, key=key))
        else:
            widget = QtGui.QLineEdit(str(value))
            widget.textChanged.connect(partial(self.text_changed, key=key))
        for grid_object in [label, widget]:
            font = grid_object.font()
            font.setPointSize(12)
            grid_object.setFont(font)
        self.layout.addWidget(label, row, 0)
        self.layout.addWidget(widget, row, 1)
        self.labels.append(label)
        self.widgets.append(widget)

    def open_converter_dialog(self, button):
        is_space_converter = self.input_term.node.node_type == "observations"
        library = self.input_term.node.graph.library
        if self.input_term.node.node_type == "observations":
            msg_type_in = None
            if "converter" in self.output_term.params() and self.output_term.params()["converter"] is not None:
                msg_type_out = get_opposite_msg_cls(
                    self.output_term.params()["msg_type"],
                    self.output_term.params()["converter"],
                )
            else:
                msg_type_out = get_attribute_from_module(self.input_term.params()["msg_type"])
        elif self.output_term.node.node_type == "actions" and "msg_type" not in self.output_term.params():
            if "converter" in self.input_term.params() and self.input_term.params()["converter"] is not None:
                msg_type_in = get_opposite_msg_cls(
                    self.input_term.params()["msg_type"],
                    self.input_term.params()["converter"],
                )
            else:
                msg_type_in = get_attribute_from_module(self.input_term.params()["msg_type"])
            msg_type_out = msg_type_in
        else:
            if "converter" in self.input_term.params() and self.input_term.params()["converter"] is not None:
                msg_type_in = get_opposite_msg_cls(
                    self.input_term.params()["msg_type"],
                    self.input_term.params()["converter"],
                )
            else:
                msg_type_in = get_attribute_from_module(self.input_term.params()["msg_type"])
            if "converter" in self.output_term.params() and self.output_term.params()["converter"] is not None:
                msg_type_out = get_opposite_msg_cls(
                    self.output_term.params()["msg_type"],
                    self.output_term.params()["converter"],
                )
            else:
                msg_type_out = get_attribute_from_module(self.input_term.params()["msg_type"])
        converter_dialog = ConverterDialog(
            converter=self.params["converter"],
            parent=self.parent,
            library=library,
            msg_type_in=msg_type_in,
            msg_type_out=msg_type_out,
            is_space_converter=is_space_converter,
        )
        converter = converter_dialog.open()
        self.params["converter"] = converter
        converter_dialog.close()
        button_string = converter["converter_type"].split("/")[-1] if "converter_type" in converter else "converter"
        button.setText("Edit {}".format(button_string))

    def combo_box_value_changed(self, int, items, key):
        self.params[key] = items[int]

    def value_changed(self, widget, key):
        self.params[key] = widget.value()

    def text_changed(self, text, key):
        self.params[key] = tryeval(text)
