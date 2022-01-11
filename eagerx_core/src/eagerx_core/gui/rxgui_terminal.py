# -*- coding: utf-8 -*-
import yaml
import inspect
import importlib
from functools import partial
from copy import deepcopy

from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.graphicsItems.GraphicsObject import GraphicsObject
from pyqtgraph import functions as fn
from pyqtgraph import ComboBox, SpinBox
from pyqtgraph.Point import Point

from eagerx_core import constants
from eagerx_core.baseconverter import BaseConverter, IdentityConverter
from eagerx_core.utils.pyqtgraph_utils import exception_handler
from eagerx_core.utils.utils import get_attribute_from_module


class RxGuiTerminal(object):
    def __init__(self, node, name, pos=None):
        self.node = node
        self.name = name
        self.pos = pos
        self.optional = False
        self.connections = {}
        self.node_type = node.node_type

        name_split = name.split('/')
        self.terminal_type = name_split[0]
        self.terminal_name = name_split[-1]

        assert self.terminal_type in set.union(constants.TERMS_IN, constants.TERMS_OUT), \
            f'Invalid terminal type: {self.terminal_type}, ' \
            f'should be one of {set.union(constants.TERMS_IN, constants.TERMS_OUT)}'

        self.is_input = self.terminal_type in constants.TERMS_IN
        self.is_state = self.terminal_type in ['targets', 'states']
        self.is_feedthrough = self.terminal_type == 'feedthroughs'
        self.is_removable = node.is_object or self.node_type in ['actions', 'observations']
        self.is_renamable = self.node_type in ['actions', 'observations']
        self.is_connectable = not (self.terminal_type == 'states' and not node.is_object)

        self._graphicsItem = TerminalGraphicsItem(self, parent=self.node.graphics_item())
        self.recolor()

    def connection_tuple(self):
        return (self.node.name, self.terminal_type, self.terminal_name)

    def connected(self, term):
        """Called whenever this terminal has been connected to another.
        (note--this function is called on both terminals)"""
        self.node.connected(self, term)

    def disconnected(self, term):
        """Called whenever this terminal has been disconnected from another.
        (note--this function is called on both terminals)"""
        self.node.disconnected(self, term)

    def params(self):
        return self._get_params(graph_backup=self.node.graph)

    @exception_handler
    def _get_params(self):
        return self.node.graph.get_parameters(*self.connection_tuple())

    def set_param(self, parameter, value):
        self._set_param(parameter, value, graph_backup=self.node.graph)

    @exception_handler
    def _set_param(self, parameter, value):
        arguments = dict(zip(['name', 'component', 'cname'], self.connection_tuple()))
        self.node.graph.set_parameter(parameter, value, **arguments)

    def is_connected(self):
        return len(self.connections) > 0

    def graphics_item(self):
        return self._graphicsItem

    def connected_to(self, term):
        return term in self.connections

    def has_input(self):
        for t in self.connections:
            if not t.is_input:
                return True
        return False

    def input_terminals(self):
        """Return the terminal(s) that give input to this one."""
        return [t for t in self.connections if not t.is_input]

    def dependent_nodes(self):
        """Return the list of nodes which receive input from this terminal."""
        return set([t.node for t in self.connections if t.is_input])

    @exception_handler
    def connect_to(self, term, connection_item):
        assert not self.connected_to(term), 'Already connected'
        assert term is not self, 'Not connecting terminal to self'
        assert term.node is not self.node, "Can't connect to terminal on same node."
        assert term.is_state == self.is_state, "Cannot connect different input/output types."
        assert not term.is_input == self.is_input, "Cannot connect input with input or output with output."
        for t in [self, term]:
            assert not (t.is_input and t.is_connected()), \
                "Cannot connect %s <-> %s: Terminal %s is already connected to %s" % (
                    self, term, t, list(t.connections.keys()))

        input_term, output_term = (self, term) if self.is_input else (term, self)

        source = output_term.connection_tuple()
        target = input_term.connection_tuple()

        observation = target[2] if target[0] == 'env/observations' else None
        action = source[2] if source[0] == 'env/actions' else None
        target_params = input_term.params()
        source_params = output_term.params()
        if len(target_params) == 0:
            converter = source_params['space_converter'] if 'space_converter' in source_params else IdentityConverter
            delay, window = 0., 0
        else:
            delay = target_params['delay'] if 'delay' in target_params else None
            window = target_params['window'] if 'window' in target_params else None
            converter = target_params['converter'] if 'converter' in target_params else IdentityConverter
        connect_params = connection_item.open_connection_dialog(converter=converter, delay=delay, window=window)
        target = None if observation else target
        source = None if action else source
        self.node.graph.connect(source=source, target=target, action=action, observation=observation, **connect_params)

        self.connections[term] = connection_item
        term.connections[self] = connection_item

        self.connected(term)
        term.connected(self)

        return connection_item

    def disconnect_from(self, term):
        if not self.connected_to(term):
            return
        item = self.connections[term]
        item.close()
        del self.connections[term]
        del term.connections[self]

        self.disconnected(term)
        term.disconnected(self)

    def disconnect_all(self):
        for t in list(self.connections.keys()):
            self.disconnect_from(t)

    def recolor(self, color=None, recurse=True):
        if color is None:
            if self.is_state:
                if self.is_connectable:
                    color = QtGui.QColor(255, 0, 0)
                else:
                    color = QtGui.QColor(128, 128, 128)
            else:
                color = QtGui.QColor(0, 0, 255)
        self.graphics_item().setBrush(QtGui.QBrush(color))

        if recurse:
            for t in self.connections:
                t.recolor(color, recurse=False)

    def rename(self, name):
        if self.is_input:
            if name in self.node.inputs:
                self.graphics_item().term_renamed(self.name)
                return
        elif name in self.node.outputs:
            self.graphics_item().term_renamed(self.name)
            return
        rename_args = dict(zip(['name', 'component', 'old'], self.connection_tuple()))
        self.node.graph.rename(new=name.split('/')[-1], **rename_args)
        old_name = self.name
        self.name = name
        self.terminal_name = name.split('/')[-1]
        self.node.terminal_renamed(self, old_name)
        self.graphics_item().term_renamed(name)

    def __repr__(self):
        return "<Terminal %s.%s>" % (str(self.node.name), str(self.terminal_name))

    def __hash__(self):
        return id(self)

    def close(self):
        self.disconnect_all()
        item = self.graphics_item()
        if item.param_window is not None:
            item.param_window.close()
        if item.scene() is not None:
            item.scene().removeItem(item)


class TerminalGraphicsItem(GraphicsObject):

    def __init__(self, term, parent=None):
        self.term = term
        GraphicsObject.__init__(self, parent)
        self.brush = fn.mkBrush(0, 0, 0)
        self.box = QtGui.QGraphicsRectItem(0, 0, 10, 10, self)
        self.label = QtGui.QGraphicsTextItem(self.term.terminal_name, self)
        self.label.scale(0.7, 0.7)
        self.newConnection = None
        self.setFiltersChildEvents(True)  # to pick up mouse events on the rectitem
        if self.term.is_renamable:
            self.label.setTextInteractionFlags(QtCore.Qt.TextEditorInteraction)
            self.label.focusOutEvent = self.label_focus_out
            self.label.keyPressEvent = self.label_key_press
        self.setZValue(1)
        self.menu = None
        self.param_window = None

    def label_focus_out(self, ev):
        QtGui.QGraphicsTextItem.focusOutEvent(self.label, ev)
        self.label_changed()

    def label_key_press(self, ev):
        if ev.key() == QtCore.Qt.Key_Enter or ev.key() == QtCore.Qt.Key_Return:
            self.label_changed()
        else:
            QtGui.QGraphicsTextItem.keyPressEvent(self.label, ev)

    def label_changed(self):
        new_name = str(self.label.toPlainText())
        if new_name != self.term.terminal_name:
            new_name = self.term.terminal_type + '/' + new_name
            self.term.rename(new_name)

    def term_renamed(self, name):
        self.label.setPlainText(name.split('/')[-1])
        if self.param_window is not None:
            self.param_window.setWindowTitle('Parameters {}'.format(name.split('/')[-1]))

    def setBrush(self, brush):
        self.brush = brush
        self.box.setBrush(brush)

    def disconnect(self, target):
        input_term, output_term = (self.term, target.term) if self.term.is_input else (target.term, self.term)
        if output_term.node_type == 'actions':
            self.term.node.graph.disconnect(action=output_term.terminal_name, target=input_term.connection_tuple(),
                                            remove=False)
        elif input_term.node_type == 'observations':
            self.term.node.graph.disconnect(source=output_term.connection_tuple(), observation=input_term.terminal_name,
                                            remove=False)
        else:
            self.term.node.graph.disconnect(source=output_term.connection_tuple(), target=input_term.connection_tuple())
        self.term.disconnect_from(target.term)

    def boundingRect(self):
        br = self.box.mapRectToParent(self.box.boundingRect())
        lr = self.label.mapRectToParent(self.label.boundingRect())
        return br | lr

    def paint(self, p, *args):
        pass

    def set_anchor(self, x, y):
        pos = QtCore.QPointF(x, y)
        self.anchorPos = pos
        br = self.box.mapRectToParent(self.box.boundingRect())
        lr = self.label.mapRectToParent(self.label.boundingRect())

        if self.term.is_input:
            self.box.setPos(pos.x(), pos.y() - br.height() / 2.)
            self.label.setPos(pos.x() + br.width(), pos.y() - lr.height() / 2.)
        else:
            self.box.setPos(pos.x() - br.width(), pos.y() - br.height() / 2.)
            self.label.setPos(pos.x() - br.width() - lr.width(), pos.y() - lr.height() / 2.)
        self.update_connections()

    def update_connections(self):
        for t, c in self.term.connections.items():
            c.update_line()

    def mousePressEvent(self, ev):
        # ev.accept()
        ev.ignore()  # necessary to allow click/drag events to process correctly

    def mouseClickEvent(self, ev):
        if ev.button() == QtCore.Qt.LeftButton:
            ev.accept()
            self.label.setFocus(QtCore.Qt.MouseFocusReason)
        elif ev.button() == QtCore.Qt.RightButton:
            ev.accept()
            self.raise_context_menu(ev)

    def mouseDoubleClickEvent(self, ev):
        if int(ev.button()) == int(QtCore.Qt.LeftButton):
            ev.accept()
            self.param_window = TerminalParamWindow(self.term)
            self.param_window.show()

    def raise_context_menu(self, ev):
        # only raise menu if this terminal is removable
        menu = self.get_menu()
        pos = ev.screenPos()
        menu.popup(QtCore.QPoint(pos.x(), pos.y()))

    def get_menu(self):
        self.menu = QtGui.QMenu()
        self.menu.setTitle("Terminal")
        rem_act = QtGui.QAction("Remove {}".format(self.term.terminal_name), self.menu)
        rem_act.triggered.connect(self.remove_self)
        self.menu.addAction(rem_act)
        self.menu.remAct = rem_act
        if not self.term.is_removable:
            rem_act.setEnabled(False)
        return self.menu

    def remove_self(self):
        if self.term.node_type == 'actions':
            self.term.node.graph.remove_component(action=self.term.terminal_name)
        elif self.term.node_type == 'observations':
            self.term.node.graph.remove_component(observation=self.term.terminal_name)
        else:
            self.term.node.graph.remove_component(*self.term.connection_tuple())
        self.term.node.remove_terminal(self.term)
        if self.param_window is not None:
            self.param_window.close()

    def mouseDragEvent(self, ev):
        if ev.button() != QtCore.Qt.LeftButton:
            ev.ignore()
            return
        if not self.term.is_connectable:
            return
        ev.accept()
        if ev.isStart():
            if self.newConnection is None:
                self.newConnection = ConnectionItem(self)
                self.getViewBox().addItem(self.newConnection)

            self.newConnection.set_target(self.mapToView(ev.pos()))
        elif ev.isFinish():
            if self.newConnection is not None:
                items = self.scene().items(ev.scenePos())
                got_target = False
                for i in items:
                    if isinstance(i, TerminalGraphicsItem):
                        self.newConnection.set_target(i)
                        try:
                            self.term.connect_to(i.term, self.newConnection, graph_backup=self.term.node.graph)
                            got_target = True
                        except Exception:
                            self.newConnection.close()
                            self.newConnection = None
                            raise
                        break

                if not got_target:
                    self.newConnection.close()
                self.newConnection = None
        else:
            if self.newConnection is not None:
                self.newConnection.set_target(self.mapToView(ev.pos()))

    def hoverEvent(self, ev):
        if not ev.isExit() and ev.acceptDrags(QtCore.Qt.LeftButton):
            ev.acceptClicks(QtCore.Qt.LeftButton)
            # we don't use the click, but we also don't want anyone else to use it.
            ev.acceptClicks(QtCore.Qt.RightButton)
            self.box.setBrush(fn.mkBrush('w'))
        else:
            self.box.setBrush(self.brush)
        self.update()

    def connect_point(self):
        # return the connect position of this terminal in view coords
        return self.mapToView(self.mapFromItem(self.box, self.box.boundingRect().center()))

    def node_moved(self):
        for t, item in self.term.connections.items():
            item.update_line()


class TerminalParamWindow(QtGui.QMainWindow):
    def __init__(self, term):
        super().__init__(term.node.graph.widget().cwWin)
        self.term = term
        self.setWindowTitle('Parameters {}'.format(term.terminal_name))
        cw = QtGui.QWidget()
        self.layout = QtGui.QGridLayout()
        cw.setLayout(self.layout)
        self.setCentralWidget(cw)
        self.labels = []
        self.widgets = []
        row = 0
        for key, value in term.params().items():
            self.add_widget(key, value, row)
            row += 1

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
        elif key in constants.GUI_TERM_ITEMS:
            items = constants.GUI_TERM_ITEMS[key]
            widget = ComboBox(items=items, default=str(value))
            widget.activated.connect(partial(self.combo_box_value_changed, key=key, items=items))
        elif isinstance(value, int):
            widget = SpinBox(value=value, int=True, dec=True)
            widget.sigValueChanged.connect(partial(self.value_changed, key=key))
        elif isinstance(value, float):
            widget = SpinBox(value=value)
            widget.sigValueChanged.connect(partial(self.value_changed, key=key))
        elif key == 'msg_type':
            widget = QtGui.QLineEdit(str(value))
            widget.setEnabled(False)
        elif key == 'space_converter':
            if self.term.is_state:
                button_string = value['converter_type'].split('/')[-1] if 'converter_type' in value else \
                    'space_converter'
                widget = QtGui.QPushButton('Edit {}'.format(button_string))
                widget.pressed.connect(partial(self.open_converter_dialog, button=widget, is_space_converter=True))
            else:
                widget = QtGui.QLineEdit(str(value))
                widget.setEnabled(False)
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

    def combo_box_value_changed(self, int, items, key):
        self.term.set_param(key, yaml.safe_load(str(items[int])))

    def value_changed(self, widget, key):
        self.term.set_param(key, widget.value())

    def text_changed(self, text, key):
        try:
            self.term.set_param(key, yaml.safe_load(str(text)))
        except Exception:
            pass

    def open_converter_dialog(self, button, is_space_converter=False):
        key = 'space_converter' if is_space_converter else 'converter'
        converter_dialog = ConverterDialog(converter=self.term.params()[key], parent=self.parent())
        converter = converter_dialog.open()
        converter_dialog.close()
        self.term.set_param(key, converter)
        button_string = converter['converter_type'].split('/')[-1] if 'converter_type' in converter else key
        button.setText('Edit {}'.format(button_string))


class ConnectionItem(GraphicsObject):

    def __init__(self, source, target=None):
        GraphicsObject.__init__(self)
        self.setFlags(
            self.ItemIsSelectable |
            self.ItemIsFocusable
        )
        self.source = source
        self.target = target
        self.length = 0
        self.hovered = False
        self.path = None
        self.shapePath = None
        self.connection_window = None

        if self.source.term.is_state:
            self.style = {
                'shape': 'line',
                'color': (255, 0, 0, 255),
                'width': 2.0,
                'hoverColor': (150, 150, 250, 255),
                'hoverWidth': 2.0,
                'selectedColor': (200, 200, 0, 255),
                'selectedWidth': 4.0,
            }
        else:
            self.style = {
                'shape': 'line',
                'color': (0, 0, 255, 255),
                'width': 2.0,
                'hoverColor': (150, 150, 250),
                'hoverWidth': 2.0,
                'selectedColor': (200, 200, 0, 255),
                'selectedWidth': 4.0,
            }
        self.source.getViewBox().addItem(self)
        self.update_line()

    def open_connection_dialog(self, **kwargs):
        input_term = self.source.term if self.source.term.is_input else self.target.term
        self.connection_window = ConnectionDialog(input_term=input_term, **kwargs)
        return self.connection_window.open()

    def close(self):
        if self.scene() is not None:
            self.scene().removeItem(self)

    def set_target(self, target):
        self.target = target
        self.update_line()

    def setStyle(self, **kwds):
        self.style.update(kwds)
        if 'shape' in kwds:
            self.update_line()
        else:
            self.update()

    def update_line(self):
        start = Point(self.source.connect_point())
        if isinstance(self.target, TerminalGraphicsItem):
            stop = Point(self.target.connect_point())
        elif isinstance(self.target, QtCore.QPointF):
            stop = Point(self.target)
        else:
            return
        self.prepareGeometryChange()

        self.path = self.generate_path(start, stop)
        self.shapePath = None
        self.update()
        self.setZValue(-1)

    def generate_path(self, start, stop):
        path = QtGui.QPainterPath()
        path.moveTo(start)
        if self.style['shape'] == 'line':
            path.lineTo(stop)
        elif self.style['shape'] == 'cubic':
            path.cubicTo(Point(stop.x(), start.y()), Point(start.x(), stop.y()), Point(stop.x(), stop.y()))
        else:
            raise Exception('Invalid shape "%s"; options are "line" or "cubic"' % self.style['shape'])
        return path

    def keyPressEvent(self, ev):
        if not self.isSelected():
            ev.ignore()
            return

        if ev.key() == QtCore.Qt.Key_Delete or ev.key() == QtCore.Qt.Key_Backspace:
            self.source.disconnect(self.target)
            ev.accept()
        elif ev.key() == QtCore.Qt.Key_Control:
            ev.accept()
            if self.style['shape'] == 'line':
                self.setStyle(shape='cubic')
            elif self.style['shape'] == 'cubic':
                self.setStyle(shape='line')
        else:
            ev.ignore()

    def mousePressEvent(self, ev):
        ev.ignore()

    def mouseClickEvent(self, ev):
        if ev.button() == QtCore.Qt.LeftButton:
            ev.accept()
            sel = self.isSelected()
            self.setSelected(True)
            self.setFocus()
            if not sel and self.isSelected():
                self.update()

    def mouseDoubleClickEvent(self, ev):
        if int(ev.button()) == int(QtCore.Qt.LeftButton):
            ev.accept()
            if self.connection_window is not None:
                self.connection_window.show()

    def hoverEvent(self, ev):
        if (not ev.isExit()) and ev.acceptClicks(QtCore.Qt.LeftButton):
            self.hovered = True
        else:
            self.hovered = False
        self.update()

    def boundingRect(self):
        return self.shape().boundingRect()

    def viewRangeChanged(self):
        self.shapePath = None
        self.prepareGeometryChange()

    def shape(self):
        if self.shapePath is None:
            if self.path is None:
                return QtGui.QPainterPath()
            stroker = QtGui.QPainterPathStroker()
            px = self.pixelWidth()
            stroker.setWidth(px * 8)
            self.shapePath = stroker.createStroke(self.path)
        return self.shapePath

    def paint(self, p, *args):
        if self.isSelected():
            p.setPen(fn.mkPen(self.style['selectedColor'], width=self.style['selectedWidth']))
        else:
            if self.hovered:
                p.setPen(fn.mkPen(self.style['hoverColor'], width=self.style['hoverWidth']))
            else:
                p.setPen(fn.mkPen(self.style['color'], width=self.style['width']))

        p.drawPath(self.path)


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
        button_string = converter['converter_type'].split('/')[-1] if 'converter_type' in converter else key
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
        converter_type = self.converter['converter_type'] if 'converter_type' in self.converter else ''
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

        class_name = class_name if class_name in available_converters else None
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
        for key, value in self.converter.items():
            if key in required_args:
                required_args[key] = value
            elif key in optional_args:
                optional_args[key] = value
            else:
                invalid_arguments.append(key)
        for key in invalid_arguments:
            self.converter.pop(key)

        self.converter['converter_type'] = '/'.join([module_name, class_name]) if class_name is not None else \
            module_name
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
