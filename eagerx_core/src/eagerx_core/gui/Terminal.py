# -*- coding: utf-8 -*-
import yaml
from pyqtgraph.Qt import QtCore, QtGui
import weakref
from pyqtgraph.graphicsItems.GraphicsObject import GraphicsObject
from pyqtgraph import functions as fn
from pyqtgraph import ComboBox, SpinBox
from pyqtgraph.Point import Point
from eagerx_core import constants
from functools import partial
from eagerx_core.utils.utils import get_opposite_msg_cls, get_module_type_string, get_cls_from_string


class Terminal(object):
    def __init__(self, node, name, io, pos=None):
        """
        Construct a new terminal. 
        
        ==============  =================================================================================
        **Arguments:**
        node            the node to which this terminal belongs
        name            string, the name of the terminal
        io              'in' or 'out'
        optional        bool, whether the node may process without connection to this terminal
        multi           bool, for inputs: whether this terminal may make multiple connections
                        for outputs: whether this terminal creates a different value for each connection
        pos             [x, y], the position of the terminal within its node's boundaries
        renamable       (bool) Whether the terminal can be renamed by the user
        removable       (bool) Whether the terminal can be removed by the user
        multiable       (bool) Whether the user may toggle the *multi* option for this terminal
        bypass          (str) Name of the terminal from which this terminal's value is derived
                        when the Node is in bypass mode.
        ==============  =================================================================================
        """

        node_type = node.node_type()
        name_split = name.split('/')
        terminal_type = name_split[0]
        terminal_name = name_split[-1]

        assert terminal_type in set.union(constants.TERMS_IN, constants.TERMS_OUT), \
            f'Invalid terminal type: {terminal_type}, ' \
            f'should be one of {set.union(constants.TERMS_IN, constants.TERMS_OUT)}'

        is_state = terminal_type in ['targets', 'states']
        is_feedthrough = terminal_type == 'feedthroughs'
        removable = node.is_object() or node_type in ['actions', 'observations']
        renamable = node_type in ['actions', 'observations']
        connectable = not (terminal_type == 'states' and not node.is_object())

        self._io = io
        self._optional = False
        self._node = weakref.ref(node)
        self._name = name
        self._terminal_name = terminal_name
        self._terminal_type = terminal_type
        self._renamable = renamable
        self._removable = removable
        self._connections = {}
        self._pos = pos
        self._is_state = is_state
        self._is_feedthrough = is_feedthrough
        self._connectable = connectable
        self._value = None
        self.valueOk = None

        # Get msg type in order to check validity of connections
        if 'msg_type' in self.params():
            self._msg_type = self.params()['msg_type']
        else:
            self._msg_type = None

        self._graphicsItem = TerminalGraphicsItem(self, parent=self._node().graphics_item())
        self.recolor()

    def value(self, term=None):
        """Return the value this terminal provides for the connected terminal"""
        if term is None:
            return self._value

        return self._value

    def set_value(self, val, process=True):
        """If this is a single-value terminal, val should be a single value."""
        # if not isinstance(self._value, dict):
        #     self._value = {}
        # if val is not None:
        #     self._value.update(val)
        #
        # self.setValueAcceptable(True)  # by default, input values are 'unchecked' until Node.update().
        # if self.isInput() and process:
        #     self.node().update()
        #
        # self.recolor()
        pass

    def setOpts(self, **opts):
        self._renamable = opts.get('renamable', self._renamable)
        self._removable = opts.get('removable', self._removable)

    def connected(self, term):
        """Called whenever this terminal has been connected to another.
        (note--this function is called on both terminals)"""
        if self.is_input() and term.is_output():
            self.input_changed(term)
        if self.node().node_type() in ['actions', 'observations']:
            if self.params()['converter'] is None:
                if term.params()['space_converter'] is not None:
                    converter = term.params()['space_converter']
                    self.params()['converter'] = converter
                    self._msg_type = self.params()['msg_type'] = get_module_type_string(
                        get_opposite_msg_cls(term.params()['msg_type'], converter)
                    )
        self.node().connected(self, term)

    def disconnected(self, term):
        """Called whenever this terminal has been disconnected from another.
        (note--this function is called on both terminals)"""
        if self.is_input():
            self.set_value(None)
        if self.node().node_type() in ['actions', 'observations']:
            self._msg_type = None
            self.params()['converter'] = None
        self.node().disconnected(self, term)

    def input_changed(self, term, process=True):
        """Called whenever there is a change to the input value to this terminal.
        It may often be useful to override this function."""
        self.set_value(term.value(self), process=process)

    def value_is_acceptable(self):
        """Returns True->acceptable  None->unknown  False->Unacceptable"""
        return self.valueOk

    def set_value_acceptable(self, v=True):
        self.valueOk = v
        self.recolor()

    def connections(self):
        return self._connections

    def node(self):
        return self._node()

    def params(self):
        params = {}
        node_params = self._node().params()
        if self._terminal_type in node_params:
            if self._terminal_name in node_params[self._terminal_type]:
                if self._is_feedthrough:
                    params = node_params['outputs'][self._terminal_name]
                else:
                    params = node_params[self._terminal_type][self._terminal_name]
        return params

    def msg_type(self):
        return self._msg_type

    def connection_msg_type(self):
        if self._msg_type is None:
            connection_msg_type = None
        else:
            if 'converter' in self.params() and self.params()['converter'] is not None:
                msg_type = get_opposite_msg_cls(self._msg_type, self.params()['converter'])
            else:
                msg_type = get_cls_from_string(self._msg_type)
            connection_msg_type = get_module_type_string(msg_type)
        return connection_msg_type

    def is_state(self):
        return self._is_state

    def is_feedthrough(self):
        return self._is_feedthrough

    def is_input(self):
        return self._io == 'in'

    def is_output(self):
        return self._io == 'out'

    def is_renamable(self):
        return self._renamable

    def is_removable(self):
        return self._removable

    def is_connectable(self):
        return self._connectable

    def name(self):
        return self._name

    def terminal_name(self):
        return self._terminal_name

    def terminal_type(self):
        return self._terminal_type

    def connection_info(self):
        if self.node().node_type() in ['actions', 'observations'] or self._terminal_type == 'outputs':
            connection_info = (self.node().rx_params(), self._terminal_name)
        else:
            connection_info = (self.node().rx_params(), self._terminal_type, self._terminal_name)
        return connection_info

    def graphics_item(self):
        return self._graphicsItem

    def is_connected(self):
        return len(self.connections()) > 0

    def connected_to(self, term):
        return term in self.connections()

    def has_input(self):
        for t in self.connections():
            if t.is_output():
                return True
        return False

    def input_terminals(self):
        """Return the terminal(s) that give input to this one."""
        return [t for t in self.connections() if t.is_output()]

    def dependent_nodes(self):
        """Return the list of nodes which receive input from this terminal."""
        return set([t.node() for t in self.connections() if t.is_input()])

    def connect_to(self, term, connection_item=None):
        try:
            if self.connected_to(term):
                raise Exception('Already connected')
            if term is self:
                raise Exception('Not connecting terminal to self')
            if term.node() is self.node():
                raise Exception("Can't connect to terminal on same node.")
            if not term.is_state() == self.is_state():
                raise Exception("Cannot connect different input/output types.")
            if term.is_input() == self.is_input():
                raise Exception("Cannot connect input with input or output with output.")
            if self.connection_msg_type() is not None and term.connection_msg_type() is not None:
                if not self.connection_msg_type() == term.connection_msg_type():
                    raise Exception("Cannot connect terminals with different message types")
            for t in [self, term]:
                if t.is_input() and len(t.connections()) > 0:
                    raise Exception("Cannot connect %s <-> %s: Terminal %s is already connected to %s" % (
                        self, term, t, list(t.connections().keys())))
        except Exception:
            if connection_item is not None:
                connection_item.close()
            raise

        if connection_item is None:
            connection_item = ConnectionItem(self.graphics_item(), term.graphics_item())
            self.graphics_item().getViewBox().addItem(connection_item)

        self._connections[term] = connection_item
        term._connections[self] = connection_item

        self.recolor()

        self.connected(term)
        term.connected(self)

        return connection_item

    def disconnect_from(self, term):
        if not self.connected_to(term):
            return
        item = self._connections[term]
        item.close()
        del self._connections[term]
        del term._connections[self]
        self.recolor()
        term.recolor()

        self.disconnected(term)
        term.disconnected(self)

    def disconnect_all(self):
        for t in list(self._connections.keys()):
            self.disconnect_from(t)

    def recolor(self, color=None, recurse=True):
        if color is None:
            if self.is_state():
                if self.is_connectable():
                    color = QtGui.QColor(255, 0, 0)
                else:
                    color = QtGui.QColor(128, 128, 128)
            else:
                color = QtGui.QColor(0, 0, 255)
        self.graphics_item().setBrush(QtGui.QBrush(color))

        if recurse:
            for t in self.connections():
                t.recolor(color, recurse=False)

    def rename(self, name):
        if self._io == 'in':
            if name in self._node().inputs():
                self.graphics_item().term_renamed(self._name)
                return
        elif name in self._node().outputs():
            self.graphics_item().term_renamed(self._name)
            return
        old_name = self._name
        self._name = name
        self._terminal_name = name.split('/')[-1]

        self.node().terminal_renamed(self, old_name)
        self.graphics_item().term_renamed(name)

    def __repr__(self):
        return "<Terminal %s.%s>" % (str(self.node().name()), str(self.terminal_name()))

    def __hash__(self):
        return id(self)

    def close(self):
        self.disconnect_all()
        item = self.graphics_item()
        if item.scene() is not None:
            item.scene().removeItem(item)

    def save_state(self):
        return {'io': self._io}


class TerminalGraphicsItem(GraphicsObject):

    def __init__(self, term, parent=None):
        self.term = term
        self.terminal_type = term.terminal_type()
        self.terminal_name = term.terminal_name()
        GraphicsObject.__init__(self, parent)
        self.brush = fn.mkBrush(0, 0, 0)
        self.box = QtGui.QGraphicsRectItem(0, 0, 10, 10, self)
        self.label = QtGui.QGraphicsTextItem(self.term.terminal_name(), self)
        self.label.scale(0.7, 0.7)
        self.newConnection = None
        self.setFiltersChildEvents(True)  # to pick up mouse events on the rectitem
        if self.term.is_renamable():
            self.label.setTextInteractionFlags(QtCore.Qt.TextEditorInteraction)
            self.label.focusOutEvent = self.label_focus_out
            self.label.keyPressEvent = self.label_key_press
        self.setZValue(1)
        self.menu = None

        self.initialise_param_window()

    def initialise_param_window(self):
        node = self.term.node()
        graph = node.eagerx_graph()
        self.param_window = QtGui.QMainWindow(graph().widget().cwWin)
        self.param_window.setWindowTitle('Parameters {}'.format(self.terminal_name))
        cw = QtGui.QWidget()
        self.layout = QtGui.QGridLayout()
        cw.setLayout(self.layout)
        self.param_window.setCentralWidget(cw)
        self.labels = []
        self.widgets = []
        row = 0
        for key, value in self.term.params().items():
            self.add_widget(key, value, row)
            row += 1

    def update_param_window(self):
        label_names = [label.text() for label in self.labels]
        for key, value in self.term.params().items():
            if key not in label_names:
                self.add_widget(key, value, len(self.labels))
        for label, widget in zip(self.labels, self.widgets):
            if label.text() == 'converter':
                widget.setText(str(self.term.params()[label.text()]))

    def add_widget(self, key, value, row):
        label = QtGui.QLabel(key)
        if isinstance(value, bool):
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
            value = self.term.connection_msg_type()
            widget = QtGui.QLineEdit(str(value))
            widget.setEnabled(False)
        elif key == 'space_converter':
            widget = QtGui.QLineEdit(str(value))
            if not self.term.is_state():
                widget.setEnabled(False)
            widget.textChanged.connect(partial(self.text_changed, key=key))
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
        self.term.params()[key] = items[int]

    def value_changed(self, widget, key):
        self.term.params()[key] = widget.value()

    def text_changed(self, text, key):
        if key == 'converter':
            old_converter = self.term.params()[key]
            new_converter = yaml.safe_load(str(text))
            if 'converter_type' in old_converter and 'converter_type' in new_converter:
                if not old_converter['converter_type'] == new_converter['converter_type']:
                    self.term.disconnect_all()
            else:
                self.term.disconnect_all()
        if not key == 'msg_type':
            self.term.params()[key] = yaml.safe_load(str(text))

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
        if new_name != self.term.terminal_name():
            new_name = self.term.terminal_type() + '/' + new_name
            self.term.rename(new_name)

    def term_renamed(self, name):
        self.label.setPlainText(name.split('/')[-1])
        self.param_window.setWindowTitle('Parameters {}'.format(name.split('/')[-1]))

    def setBrush(self, brush):
        self.brush = brush
        self.box.setBrush(brush)

    def disconnect(self, target):
        self.term.disconnect_from(target.term)

    def boundingRect(self):
        br = self.box.mapRectToParent(self.box.boundingRect())
        lr = self.label.mapRectToParent(self.label.boundingRect())
        return br | lr

    def paint(self, p, *args):
        pass

    def setAnchor(self, x, y):
        pos = QtCore.QPointF(x, y)
        self.anchorPos = pos
        br = self.box.mapRectToParent(self.box.boundingRect())
        lr = self.label.mapRectToParent(self.label.boundingRect())

        if self.term.is_input():
            self.box.setPos(pos.x(), pos.y() - br.height() / 2.)
            self.label.setPos(pos.x() + br.width(), pos.y() - lr.height() / 2.)
        else:
            self.box.setPos(pos.x() - br.width(), pos.y() - br.height() / 2.)
            self.label.setPos(pos.x() - br.width() - lr.width(), pos.y() - lr.height() / 2.)
        self.updateConnections()

    def updateConnections(self):
        for t, c in self.term.connections().items():
            c.updateLine()

    def mousePressEvent(self, ev):
        # ev.accept()
        ev.ignore()   # necessary to allow click/drag events to process correctly

    def mouseClickEvent(self, ev):
        if ev.button() == QtCore.Qt.LeftButton:
            ev.accept()
            self.label.setFocus(QtCore.Qt.MouseFocusReason)
        elif ev.button() == QtCore.Qt.RightButton:
            ev.accept()
            self.raiseContextMenu(ev)

    def mouseDoubleClickEvent(self, ev):
        if int(ev.button()) == int(QtCore.Qt.LeftButton):
            ev.accept()
            self.update_param_window()
            self.param_window.show()

    def raiseContextMenu(self, ev):
        # only raise menu if this terminal is removable
        menu = self.getMenu()
        pos = ev.screenPos()
        menu.popup(QtCore.QPoint(pos.x(), pos.y()))

    def getMenu(self):
        self.menu = QtGui.QMenu()
        self.menu.setTitle("Terminal")
        remAct = QtGui.QAction("Remove {}".format(self.term.terminal_name()), self.menu)
        remAct.triggered.connect(self.removeSelf)
        self.menu.addAction(remAct)
        self.menu.remAct = remAct
        if not self.term.is_removable():
            remAct.setEnabled(False)
        return self.menu

    def removeSelf(self):
        self.term.node().remove_terminal(self.term)

    def mouseDragEvent(self, ev):
        if ev.button() != QtCore.Qt.LeftButton:
            ev.ignore()
            return
        if not self.term.is_connectable():
            return
        ev.accept()
        if ev.isStart():
            if self.newConnection is None:
                self.newConnection = ConnectionItem(self)
                self.getViewBox().addItem(self.newConnection)

            self.newConnection.setTarget(self.mapToView(ev.pos()))
        elif ev.isFinish():
            if self.newConnection is not None:
                items = self.scene().items(ev.scenePos())
                got_target = False
                for i in items:
                    if isinstance(i, TerminalGraphicsItem):
                        self.newConnection.setTarget(i)
                        try:
                            self.term.connect_to(i.term, self.newConnection)
                            got_target = True
                        except Exception:
                            self.scene().removeItem(self.newConnection)
                            self.newConnection = None
                            raise
                        break

                if not got_target:
                    self.newConnection.close()
                self.newConnection = None
        else:
            if self.newConnection is not None:
                self.newConnection.setTarget(self.mapToView(ev.pos()))

    def hoverEvent(self, ev):
        if not ev.isExit() and ev.acceptDrags(QtCore.Qt.LeftButton):
            ev.acceptClicks(QtCore.Qt.LeftButton)
            # we don't use the click, but we also don't want anyone else to use it.
            ev.acceptClicks(QtCore.Qt.RightButton)
            self.box.setBrush(fn.mkBrush('w'))
        else:
            self.box.setBrush(self.brush)
        self.update()

    def connectPoint(self):
        # return the connect position of this terminal in view coords
        return self.mapToView(self.mapFromItem(self.box, self.box.boundingRect().center()))

    def nodeMoved(self):
        for t, item in self.term.connections().items():
            item.updateLine()


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
        if self.source.term.is_state():
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
        self.updateLine()

    def close(self):
        if self.scene() is not None:
            self.scene().removeItem(self)

    def setTarget(self, target):
        self.target = target
        self.updateLine()

    def setStyle(self, **kwds):
        self.style.update(kwds)
        if 'shape' in kwds:
            self.updateLine()
        else:
            self.update()

    def updateLine(self):
        start = Point(self.source.connectPoint())
        if isinstance(self.target, TerminalGraphicsItem):
            stop = Point(self.target.connectPoint())
        elif isinstance(self.target, QtCore.QPointF):
            stop = Point(self.target)
        else:
            return
        self.prepareGeometryChange()

        self.path = self.generatePath(start, stop)
        self.shapePath = None
        self.update()
        self.setZValue(100)

    def generatePath(self, start, stop):
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
