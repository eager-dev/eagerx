# -*- coding: utf-8 -*-
import yaml
from functools import partial

from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.graphicsItems.GraphicsObject import GraphicsObject
from pyqtgraph import functions as fn
from pyqtgraph import ComboBox, SpinBox
from pyqtgraph.Point import Point

from eagerx_core import constants
from eagerx_core.utils.pyqtgraph_utils import exception_handler, ConnectionDialog, ParamWindow
from eagerx_core.baseconverter import IdentityConverter


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
        params = self.node.graph.get_parameters(*self.connection_tuple())
        if self.terminal_type == 'feedthroughs':
            output_params = self.node.graph.get_parameters(self.node.name, 'outputs', self.terminal_name)
            params['msg_type'] = output_params['msg_type']
            params['space_converter'] = output_params['space_converter']
        return params

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
            param_window = ParamWindow(node=self.term.node, term=self.term)
            param_window.open()
            param_window.close()

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

