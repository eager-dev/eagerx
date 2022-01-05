# -*- coding: utf-8 -*-
import numpy as np
from copy import deepcopy

from eagerx_core import constants
from eagerx_core.utils.utils import get_yaml_type
from eagerx_core.gui.rxgui_terminal import *

from pyqtgraph.pgcollections import OrderedDict
from pyqtgraph.debug import *
from pyqtgraph import ComboBox, SpinBox


def str_dict(d):
    return dict([(str(k), v) for k, v in d.items()])


class RxGuiNode(QtCore.QObject):
    # sigOutputChanged = QtCore.Signal(object)  # self
    sigClosed = QtCore.Signal(object)
    sigRenamed = QtCore.Signal(object, object)
    sigTerminalRenamed = QtCore.Signal(object, object)  # term, oldName
    sigTerminalAdded = QtCore.Signal(object, object)  # self, term
    sigTerminalRemoved = QtCore.Signal(object, object)  # self, term

    def __init__(self, name, graph):
        QtCore.QObject.__init__(self)

        self.params, self.default_params = graph.node_params(name)
        self.name = name
        self._graphics_item = None
        self.terminals = OrderedDict()
        self.inputs = OrderedDict()
        self.outputs = OrderedDict()
        self.graph = graph
        self.exception = None

        if 'config_name' in self.params['default'] and \
                self.params['default']['config_name'] in ['actions', 'observations', 'render']:
            self.node_type = self.params['default']['config_name']
            self.allow_add_terminal = not self.node_type == 'render'
            self.allow_remove = False
            self.is_object = False
        else:
            self.node_type = get_yaml_type(self.default_params)
            self.allow_add_terminal = self.is_object = self.node_type == 'object'
            self.allow_remove = True
        self.__initialize_terminals()

    def __next_terminal_name(self, name):
        """Return an unused terminal name"""
        name2 = name
        i = 1
        while name2 in self.terminals:
            name2 = "%s_%d" % (name, i)
            i += 1
        return name2

    def __initialize_terminals(self):
        for terminal_type in set.union(constants.TERMS_IN, constants.TERMS_OUT):
            if self.node_type == 'render' and terminal_type == 'outputs':
                continue
            if terminal_type in self.params['default']:
                for terminal in self.params['default'][terminal_type]:
                    if self.node_type in ['actions', 'observations']:
                        if terminal in self.default_params['default'][terminal_type]:
                            continue
                    name = terminal_type + '/' + terminal
                    self.add_terminal(name=name)
                    if self.node_type == 'reset_node' and terminal_type == 'outputs':
                        name = 'feedthroughs/' + terminal
                        self.add_terminal(name=name)

    def remove_terminal(self, term):
        """Remove the specified terminal from this Node. May specify either the 
        terminal's name or the terminal itself.
        
        Causes sigTerminalRemoved to be emitted."""
        if isinstance(term, RxGuiTerminal):
            name = term.name
        else:
            name = term
            term = self.terminals[name]
        term.close()

        if self.node_type == 'actions':
            self.graph._remove_action(term.terminal_name)
        elif self.node_type == 'observations':
            self.graph._remove_observation(term.terminal_name)

        del self.terminals[name]
        if term.is_input:
            del self.inputs[name]
        else:
            del self.outputs[name]

        self.graphics_item().update_terminals()
        self.sigTerminalRemoved.emit(self, term)

    def terminal_renamed(self, term, old_name):
        """Called after a terminal has been renamed        
        
        Causes sigTerminalRenamed to be emitted."""
        new_name = term.name
        for d in [self.terminals, self.inputs, self.outputs]:
            if old_name not in d:
                continue
            d[new_name] = d[old_name]
            del d[old_name]

        old_name_split = old_name.split('/')
        old_terminal_type = old_name_split[0]
        old_terminal_name = old_name_split[-1]

        new_name_split = new_name.split('/')
        new_terminal_type = new_name_split[0]
        new_terminal_name = new_name_split[-1]

        assert old_terminal_type == new_terminal_type, 'Terminal type should not change after renaming the terminal.'
        terminal_type = old_terminal_type

        self.params['default'][terminal_type].append(new_terminal_name)
        self.params['default'][terminal_type].remove(old_terminal_name)
        self.params[terminal_type][new_terminal_name] = self.params[terminal_type][old_terminal_name]
        self.params[terminal_type].pop(old_terminal_name)

        self.graphics_item().update_terminals()
        self.sigTerminalRenamed.emit(term, old_name)

    def add_terminal(self, name):
        """Add a new terminal to this Node with the given name. Extra
        keyword arguments are passed to Terminal.__init__.
                
        Causes sigTerminalAdded to be emitted."""
        name = self.__next_terminal_name(name)
        terminal_type = name.split('/')[0]
        terminal_name = name.split('/')[-1]

        if self.node_type == 'actions':
            if terminal_name not in self.params['default'][terminal_type]:
                self.graph._add_action(terminal_name)
        elif self.node_type == 'observations':
            if terminal_name not in self.params['default'][terminal_type]:
                self.graph._add_observation(terminal_name)

        term = RxGuiTerminal(self, name)
        self.terminals[name] = term

        if term.is_input:
            self.inputs[name] = term
        else:
            self.outputs[name] = term

        self.graphics_item().update_terminals()
        self.sigTerminalAdded.emit(self, term)
        return term

    def graphics_item(self):
        """Return the GraphicsItem for this node. Subclasses may re-implement
        this method to customize their appearance in the flowchart."""
        if self._graphics_item is None:
            self._graphics_item = NodeGraphicsItem(self)
        return self._graphics_item

    # this is just bad planning. Causes too many bugs.
    def __getattr__(self, attr):
        """Return the terminal with the given name"""
        if attr not in self.terminals:
            raise AttributeError(attr)
        else:
            import traceback
            traceback.print_stack()
            print("Warning: use of node.terminalName is deprecated; use node['terminalName'] instead.")
            return self.terminals[attr]

    def __getitem__(self, item):
        # return getattr(self, item)
        """Return the terminal with the given name"""
        if item not in self.terminals:
            raise KeyError(item)
        else:
            return self.terminals[item]

    def rename(self, name):
        """Rename this node. This will cause sigRenamed to be emitted."""
        old_name = self.name
        self.name = name
        self.sigRenamed.emit(self, old_name)

    def dependent_nodes(self):
        """Return the list of nodes which provide direct input to this node"""
        nodes = set()
        for t in self.inputs.values():
            nodes |= set([i.node for i in t.input_terminals()])
        return nodes

    def __repr__(self):
        return "<Node %s @%x>" % (self.name, id(self))

    def ctrl_widget(self):
        """Return this Node's control widget. 
        
        By default, Nodes have no control widget. Subclasses may reimplement this 
        method to provide a custom widget. This method is called by Flowcharts
        when they are constructing their Node list."""
        return None

    def connected(self, local_term, remote_term):
        """Called whenever one of this node's terminals is connected elsewhere."""
        pass

    def disconnected(self, local_term, remote_term):
        """Called whenever one of this node's terminals is disconnected from another."""
        pass

    def set_exception(self, exc):
        self.exception = exc
        self.recolor()

    def clear_exception(self):
        self.set_exception(None)

    def recolor(self):
        if self.exception is None:
            self.graphics_item().setPen(QtGui.QPen(QtGui.QColor(0, 0, 0)))
        else:
            self.graphics_item().setPen(QtGui.QPen(QtGui.QColor(150, 0, 0), 3))

    def load_state(self, state):
        pos = state.get('pos', (0, 0))
        self.graphics_item().setPos(*pos)

    def save_terminals(self):
        terms = OrderedDict()
        for n, t in self.terminals.items():
            terms[n] = (t.save_state())
        return terms

    def clear_terminals(self):
        for t in self.terminals.values():
            t.close()
        self.terminals = OrderedDict()
        self.inputs = OrderedDict()
        self.outputs = OrderedDict()

    def close(self):
        """Cleans up after the node--removes terminals, graphicsItem, widget"""
        self.disconnect_all()
        self.clear_terminals()
        item = self.graphics_item()
        if item.scene() is not None:
            item.scene().removeItem(item)
        self._graphics_item = None
        w = self.ctrl_widget()
        if w is not None:
            w.setParent(None)
        self.sigClosed.emit(self)

    def disconnect_all(self):
        for t in self.terminals.values():
            t.disconnect_all()


class NodeGraphicsItem(GraphicsObject):
    def __init__(self, node):
        GraphicsObject.__init__(self)
        self._node_type = node.node_type
        self._params = node.params
        self._default = node.default_params

        self.set_color()

        self.node = node
        flags = self.ItemIsMovable | self.ItemIsSelectable | self.ItemIsFocusable | self.ItemSendsGeometryChanges

        self.setFlags(flags)
        self.bounds = QtCore.QRectF(0, 0, 125, 125)
        self.nameItem = QtGui.QGraphicsTextItem(self.node.name, self)
        self.nameItem.setDefaultTextColor(QtGui.QColor(50, 50, 50))
        self.nameItem.moveBy(self.bounds.width() / 2. - self.nameItem.boundingRect().width() / 2., 0)
        self.nameItem.setTextInteractionFlags(QtCore.Qt.TextEditorInteraction)
        self.update_terminals()

        self.nameItem.focusOutEvent = self.label_focus_out
        self.nameItem.keyPressEvent = self.label_key_press

        self.menu = None
        self.buildMenu()
        self.initial_z_value = self.zValue()
        self.initialise_param_window()

    def set_color(self):
        if 'color' in self._params['default'] and self._params['default']['color'] in constants.GUI_COLORS:
            brush_color = np.array(constants.GUI_COLORS[self._params['default']['color']])
        else:
            brush_color = np.array([200, 200, 200])

        self.brush = fn.mkBrush(*brush_color, 50)
        self.hoverBrush = fn.mkBrush(*brush_color, 100)
        self.selectBrush = fn.mkBrush(*brush_color, 100)
        self.pen = fn.mkPen(0, 0, 0, 200, width=2)
        self.selectPen = fn.mkPen(0, 0, 0, 200, width=4)
        self.hovered = False

    def initialise_param_window(self):
        self.param_window = QtGui.QMainWindow(self.node.graph.widget().cwWin)
        self.param_window.setWindowTitle('Parameters {}'.format(self.node.name))
        cw = QtGui.QWidget()
        self.layout = QtGui.QGridLayout()
        cw.setLayout(self.layout)
        self.param_window.setCentralWidget(cw)
        self.labels = []
        self.widgets = []
        row = 1
        for key, value in self.node.params['default'].items():
            label = QtGui.QLabel(key)
            if isinstance(value, bool):
                items = ['True', 'False']
                widget = ComboBox(items=items, default=str(value))
                widget.activated.connect(partial(self.combo_box_value_changed, key=key, items=items))
            elif key in constants.GUI_NODE_ITEMS:
                items = constants.GUI_NODE_ITEMS[key]
                if isinstance(items, dict):
                    converter = items
                    index = list(items.values()).index(value)
                    items = list(items.keys())
                    value = items[index]
                else:
                    converter = None
                widget = ComboBox(items=items, default=str(value))
                widget.activated.connect(
                    partial(self.combo_box_value_changed, key=key, items=items, converter=converter)
                )
            elif isinstance(value, int):
                widget = SpinBox(value=value, int=True, dec=True)
                widget.sigValueChanged.connect(partial(self.value_changed, key=key))
            elif isinstance(value, float):
                widget = SpinBox(value=value, dec=True)
                widget.sigValueChanged.connect(partial(self.value_changed, key=key))
            elif key in constants.PARAMS_CONSTANT:
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
            row += 1

    def update_param_window(self):
        for idx, label in enumerate(self.labels):
            if label.text() in set.union(constants.TERMS_IN, constants.TERMS_OUT):
                self.widgets[idx].setText(str(self.node.params['default'][label.text()]))

    def combo_box_value_changed(self, int, items, key, converter=None):
        value = items[int]
        if converter is not None:
            value = converter[value]
        self.node.params['default'][key] = value
        if key == 'color':
            self.set_color()
            self.update()

    def value_changed(self, widget, key):
        self.node.params['default'][key] = widget.value()

    def text_changed(self, text, key):
        self.node.params['default'][key] = text

    def label_focus_out(self, ev):
        QtGui.QGraphicsTextItem.focusOutEvent(self.nameItem, ev)
        self.label_changed()

    def label_key_press(self, ev):
        if ev.key() == QtCore.Qt.Key_Enter or ev.key() == QtCore.Qt.Key_Return:
            self.label_changed()
        else:
            QtGui.QGraphicsTextItem.keyPressEvent(self.nameItem, ev)

    def label_changed(self):
        new_name = str(self.nameItem.toPlainText())
        if self.node.graph is not None and new_name in self.node.graph.nodes.keys():
            self.nameItem.setPlainText(self.node.name)
            return
        if new_name != self.node.name:
            self.node.rename(new_name)
        # re-center the label
        bounds = self.boundingRect()
        self.nameItem.setPos(bounds.width() / 2. - self.nameItem.boundingRect().width() / 2., 0)

    def setPen(self, *args, **kwargs):
        self.pen = fn.mkPen(*args, **kwargs)
        self.update()

    def setBrush(self, brush):
        self.brush = brush
        self.update()

    def update_terminals(self):
        bounds = self.bounds
        self.terminals = {}
        inp = self.node.inputs
        dy = bounds.height() / (len(inp) + 1)
        y = dy
        for i, t in inp.items():
            item = t.graphics_item()
            item.setParentItem(self)
            br = self.bounds
            item.set_anchor(0, y)
            self.terminals[i] = (t, item)
            y += dy

        out = self.node.outputs
        dy = bounds.height() / (len(out) + 1)
        y = dy
        for i, t in out.items():
            item = t.graphics_item()
            item.setParentItem(self)
            item.setZValue(self.initial_z_value)
            br = self.bounds
            item.set_anchor(bounds.width(), y)
            self.terminals[i] = (t, item)
            y += dy

    def boundingRect(self):
        return self.bounds.adjusted(-5, -5, 5, 5)

    def paint(self, p, *args):

        p.setPen(self.pen)
        if self.isSelected():
            p.setPen(self.selectPen)
            p.setBrush(self.selectBrush)
            self.setZValue(200)
        else:
            p.setPen(self.pen)
            if self.hovered:
                p.setBrush(self.hoverBrush)
                self.setZValue(200)
            else:
                p.setBrush(self.brush)
                self.setZValue(self.initial_z_value)

        p.drawRect(self.bounds)

    def mousePressEvent(self, ev):
        ev.ignore()

    def mouseClickEvent(self, ev):
        if int(ev.button()) == int(QtCore.Qt.LeftButton):
            ev.accept()
            sel = self.isSelected()
            self.setSelected(True)
            if not sel and self.isSelected():
                self.update()

        elif int(ev.button()) == int(QtCore.Qt.RightButton):
            self.menu = None
            self.buildMenu()
            ev.accept()
            self.raiseContextMenu(ev)

    def mouseDoubleClickEvent(self, ev):
        if int(ev.button()) == int(QtCore.Qt.LeftButton):
            ev.accept()
            self.update_param_window()
            self.param_window.show()

    def mouseDragEvent(self, ev):
        if ev.button() == QtCore.Qt.LeftButton:
            ev.accept()
            self.setPos(self.pos() + self.mapToParent(ev.pos()) - self.mapToParent(ev.lastPos()))

    def hoverEvent(self, ev):
        if not ev.isExit() and ev.acceptClicks(QtCore.Qt.LeftButton):
            ev.acceptDrags(QtCore.Qt.LeftButton)
            self.hovered = True
        else:
            self.hovered = False
        self.update()

    def keyPressEvent(self, ev):
        if ev.key() == QtCore.Qt.Key_Delete or ev.key() == QtCore.Qt.Key_Backspace:
            ev.accept()
            if not self.node.allow_remove:
                return
            self.node.graph.remove(self.node.name)
            self.node.close()
        else:
            ev.ignore()

    def itemChange(self, change, val):
        if change == self.ItemPositionHasChanged:
            self.node.graph._state['nodes'][self.node.name]['pos'] = [self.pos().x(), self.pos().y()]
            for k, t in self.terminals.items():
                t[1].node_moved()
        return GraphicsObject.itemChange(self, change, val)

    def getMenu(self):
        return self.menu

    def raiseContextMenu(self, ev):
        menu = self.getMenu()
        pos = ev.screenPos()
        menu.popup(QtCore.QPoint(pos.x(), pos.y()))

    def buildMenu(self):
        self.menu = QtGui.QMenu()
        self.menu.setTitle("Node")

        if self.node.allow_add_terminal:
            if self._node_type in ['observations', 'actions']:
                terminal = self._node_type[:-1]
                if self._node_type == 'observations':
                    terminal_name = 'inputs/' + terminal
                else:
                    terminal_name = 'outputs/' + terminal
                self.menu.addAction('Add {}'.format(terminal), partial(self.node.add_terminal, name=terminal_name))
            else:
                for terminal_type in set.union(constants.TERMS[self._node_type]['in'],
                                               constants.TERMS[self._node_type]['out']):
                    terminal_menu = QtGui.QMenu('Add {}'.format(terminal_type[:-1]), self.menu)
                    for terminal in self._default[terminal_type]:
                        terminal_name = str(terminal_type) + '/' + terminal
                        act = terminal_menu.addAction(terminal, partial(self.node.add_terminal, name=terminal_name))
                        if terminal_name in self.node.terminals.keys():
                            act.setEnabled(False)
                        self.menu.addMenu(terminal_menu)
        if self.node.allow_remove:
            self.menu.addAction("Remove {}".format(self.node.name), self.node.close)