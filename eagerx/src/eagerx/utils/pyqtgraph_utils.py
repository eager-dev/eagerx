import yaml
import inspect
import importlib
from functools import wraps, partial
from copy import deepcopy

from pyqtgraph import ComboBox, SpinBox
from pyqtgraph.Qt import QtGui

from eagerx.core import constants
from eagerx.utils.utils import get_attribute_from_module
from eagerx.core.entities import BaseConverter


def exception_handler(function_to_decorate):
    wraps(function_to_decorate)

    def exception_handler_wrapper(*args, graph_backup=None, dialog_title='Invalid', **kwargs):
        assert graph_backup is not None, 'Graph should be defined for recovering state.'
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


class ParamWindow(QtGui.QDialog):
    def __init__(self, node, term=None):
        super().__init__(node.graph.widget().cwWin)
        self.node = node
        self.node_type = node.node_type
        self.is_term = term is not None
        self.entity = term if self.is_term else node
        self.entity_type = 'term' if self.is_term else 'node'

        name = term.terminal_name if self.is_term else node.name
        self.setWindowTitle('Parameters {}'.format(name))

        self.layout = QtGui.QGridLayout()
        self.labels = []
        self.widgets = []
        self.params_changed = {}
        row = 0
        for key, value in self.entity.params().items():
            if key in constants.GUI_WIDGETS[self.entity_type]['hide']['all']:
                continue
            elif self.node_type in constants.GUI_WIDGETS[self.entity_type]['hide'] and \
                    key in constants.GUI_WIDGETS[self.entity_type]['hide'][self.node_type]:
                continue
            elif self.is_term and self.entity.terminal_type in constants.GUI_WIDGETS[self.entity_type]['hide'] and \
                    key in constants.GUI_WIDGETS[self.entity_type]['hide'][self.entity.terminal_type]:
                continue
            else:
                self.add_widget(key, value, row)
                row += 1
        if row == 0:
            label = QtGui.QLabel('No parameters to show.')
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
        if self.is_term and key in 'converter':
            button_string = value['converter_type'].split('/')[-1] if 'converter_type' in value else 'converter'
            widget = QtGui.QPushButton('Edit {}'.format(button_string))
            widget.pressed.connect(partial(self.open_converter_dialog, button=widget))
        elif self.is_term and key == 'space_converter':
            if self.entity.is_state:
                button_string = value['converter_type'].split('/')[-1] if 'converter_type' in value else \
                    'space_converter'
                widget = QtGui.QPushButton('Edit {}'.format(button_string))
                widget.pressed.connect(partial(self.open_converter_dialog, button=widget, is_space_converter=True))
            else:
                widget = QtGui.QLineEdit(str(value))
                widget.setEnabled(False)
        elif key in constants.GUI_WIDGETS[self.entity_type]['items']:
            items = constants.GUI_WIDGETS[self.entity_type]['items'][key]
            if isinstance(items, dict):
                item_converter = items
                index, items = list(items.values()).index(value), list(items.keys())
                value = items[index]
            else:
                item_converter = None
            widget = ComboBox(items=items, default=str(value))
            widget.activated.connect(partial(self.combo_box_value_changed, key=key, items=items,
                                             item_converter=item_converter))
        elif isinstance(value, bool):
            items = ['True', 'False']
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
        if key in constants.GUI_WIDGETS[self.entity_type]['constant']['all'] or \
                (self.node_type in constants.GUI_WIDGETS[self.entity_type]['constant'] and
                 key in constants.GUI_WIDGETS[self.entity_type]['constant'][self.node_type]):
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
        self.params_changed[key] = text

    def open_converter_dialog(self, button, is_space_converter=False):
        key = 'space_converter' if is_space_converter else 'converter'
        converter_dialog = ConverterDialog(converter=self.entity.params()[key], parent=self.parent())
        converter = converter_dialog.open()
        converter_dialog.close()
        self.params_changed[key] = converter
        button_string = converter['converter_type'].split('/')[-1] if 'converter_type' in converter else key
        button.setText('Edit {}'.format(button_string))

class ConverterDialog(QtGui.QDialog):
    def __init__(self, converter, parent):
        super().__init__(parent)
        self.parent = parent
        self.converter = converter

        self.setWindowTitle('Converter Parameters')
        self.layout = QtGui.QGridLayout()
        self.labels = []
        self.widgets = []

        module_name, class_name, available_converters, required_args, optional_args = self.get_parameters()

        self.add_widget(key='Converter Module', value=module_name, row=0)
        self.add_widget(key='Converter Class', value=class_name, row=1, items=available_converters)

        self.add_argument_widgets(required_args, optional_args)

        self.setLayout(self.layout)

    def open(self):
        self.exec_()
        valid = False
        while not valid:
            try:
                for key, value in self.converter.items():
                    self.converter[key] = yaml.safe_load(str(value))
                get_attribute_from_module(self.converter['converter_type'])
                valid = True
            except Exception as e:
                error_window = QtGui.QDialog(self.parent)
                error_window.setWindowTitle('Invalid Converter')
                layout = QtGui.QGridLayout()
                label = QtGui.QLabel(str(e))
                layout.addWidget(label)
                error_window.setLayout(layout)
                error_window.exec_()
                self.exec_()
        return self.converter

    def add_argument_widgets(self, required_args, optional_args):
        row = 2
        required_args_label = QtGui.QLabel('Required Converter Arguments')
        self.layout.addWidget(required_args_label, row, 0)
        self.labels.append(required_args_label)
        row += 1

        for key, value in required_args.items():
            self.add_widget(key=key, value=str(value), row=row)
            row += 1

        optional_args_label = QtGui.QLabel('Optional Converter Arguments')
        self.layout.addWidget(optional_args_label, row, 0)
        self.labels.append(optional_args_label)

        row += 1
        for key, value in optional_args.items():
            self.add_widget(key=key, value=str(value), row=row)
            row += 1

    def get_parameters(self):
        converter = deepcopy(self.converter)
        converter_type = converter['converter_type'] if 'converter_type' in converter else ''
        module_name = converter_type.split('/')[0]
        class_name = converter_type.split('/')[1] if len(converter_type.split('/')) == 2 else None

        available_converters = []
        required_args = {}
        optional_args = {}
        module = None

        module_exists = importlib.util.find_spec(module_name) is not None
        if module_exists:
            module = importlib.import_module(module_name)
            for name, member in inspect.getmembers(module):
                if inspect.isclass(member):
                    if BaseConverter in member.__mro__:
                        available_converters.append(name)

        if class_name in available_converters:
            class_name = class_name
        elif len(available_converters) > 0:
            class_name = available_converters[0]
        else:
            class_name = None

        if class_name is not None:
            argspec = inspect.getfullargspec(getattr(module, class_name).__init__)
            default_values = [] if argspec.defaults is None else argspec.defaults
            required_keys = argspec.args if len(default_values) == 0 else argspec.args[:-len(default_values)]
            if 'self' in required_keys:
                required_keys.remove('self')
            optional_keys = argspec.args[-len(default_values):]
            optional_args = dict(zip(optional_keys, default_values))
            required_args = dict(zip(required_keys, [''] * len(required_keys)))

        invalid_arguments = []
        for key, value in converter.items():
            if key in required_args:
                required_args[key] = value
            elif key in optional_args:
                optional_args[key] = value
            else:
                invalid_arguments.append(key)
        for key in invalid_arguments:
            converter.pop(key)

        converter['converter_type'] = '/'.join([module_name, class_name]) if class_name is not None else \
            module_name
        self.converter = converter
        return module_name, class_name, available_converters, required_args, optional_args

    def add_widget(self, key, value, row, items=None):
        label = QtGui.QLabel(key)
        if items is not None:
            widget = ComboBox(items=items, default=value)
            widget.activated.connect(partial(self.class_changed, items=items))
        else:
            widget = QtGui.QLineEdit(value)
            if key == 'Converter Module':
                widget.textChanged.connect(self.module_changed)
            else:
                widget.textChanged.connect(partial(self.argument_changed, key=key))

        self.layout.addWidget(label, row, 0)
        self.layout.addWidget(widget, row, 1)
        self.labels.append(label)
        self.widgets.append(widget)

    def module_changed(self, text):
        old_converter = self.converter['converter_type']
        converter_class = old_converter.split('/')[1] if len(old_converter.split('/')) == 2 else ''
        self.converter['converter_type'] = text + '/' + converter_class

        for label in self.labels[1:]:
            self.layout.removeWidget(label)
            label.close()

        for widget in self.widgets[1:]:
            self.layout.removeWidget(widget)
            widget.close()

        self.labels = [self.labels[1]]
        self.widgets = [self.widgets[1]]

        module_name, class_name, available_converters, required_args, optional_args = self.get_parameters()

        self.add_widget(key='Converter Class', value=class_name, row=1, items=available_converters)

        self.add_argument_widgets(required_args, optional_args)

    def class_changed(self, int, items):
        converter_class = items[int]
        converter_module = self.converter['converter_type'].split('/')[0]
        converter_type = '/'.join([converter_module, converter_class])
        self.converter['converter_type'] = converter_type

        for label in self.labels[2:]:
            self.layout.removeWidget(label)
            label.close()

        for widget in self.widgets[2:]:
            self.layout.removeWidget(widget)
            widget.close()

        self.labels = self.labels[:3]
        self.widgets = self.widgets[:3]

        _, _, _, required_args, optional_args = self.get_parameters()

        self.add_argument_widgets(required_args, optional_args)

    def argument_changed(self, text, key):
        self.converter[key] = text

class ConnectionDialog(QtGui.QDialog):
    def __init__(self, input_term, **kwargs):
        super().__init__(input_term.node.graph.widget().cwWin)
        self.setWindowTitle('Connection Parameters')
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
        if key == 'converter':
            button_string = value['converter_type'].split('/')[-1] if 'converter_type' in value else 'converter'
            widget = QtGui.QPushButton('Edit {}'.format(button_string))
            widget.pressed.connect(partial(self.open_converter_dialog, button=widget))
        elif isinstance(value, bool):
            items = ['True', 'False']
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
        converter_dialog = ConverterDialog(converter=self.params['converter'], parent=self.parent())
        converter = converter_dialog.open()
        self.params['converter'] = converter
        converter_dialog.close()
        button_string = converter['converter_type'].split('/')[-1] if 'converter_type' in converter else 'converter'
        button.setText('Edit {}'.format(button_string))

    def combo_box_value_changed(self, int, items, key):
        self.params[key] = items[int]

    def value_changed(self, widget, key):
        self.params[key] = widget.value()

    def text_changed(self, text, key):
        try:
            self.params[key] = yaml.safe_load(str(text))
        except Exception:
            pass