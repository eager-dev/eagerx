# -*- coding: utf-8 -*-
import inspect
from copy import deepcopy
from eagerx_core.gui.Terminal import *
from eagerx_core import constants
from pyqtgraph.pgcollections import OrderedDict
from pyqtgraph.debug import *
from pyqtgraph import ComboBox, SpinBox
import numpy as np
from eagerx_core.utils.utils import get_yaml_type
from eagerx_core.params import RxInput, RxOutput, RxState, RxFeedthrough, RxObjectParams, RxNodeParams

def strDict(d):
    return dict([(str(k), v) for k, v in d.items()])


class Node(QtCore.QObject):
    """
    Node represents the basic processing unit of a flowchart. 
    A Node subclass implements at least:
    
    1) A list of input / ouptut terminals and their properties
    2) a process() function which takes the names of input terminals as keyword arguments and returns a dict with the names of output terminals as keys.

    A flowchart thus consists of multiple instances of Node subclasses, each of which is connected
    to other by wires between their terminals. A flowchart is, itself, also a special subclass of Node.
    This allows Nodes within the flowchart to connect to the input/output nodes of the flowchart itself.

    Optionally, a node class can implement the ctrlWidget() method, which must return a QWidget (usually containing other widgets) that will be displayed in the flowchart control panel. Some nodes implement fairly complex control widgets, but most nodes follow a simple form-like pattern: a list of parameter names and a single value (represented as spin box, check box, etc..) for each parameter. To make this easier, the CtrlNode subclass allows you to instead define a simple data structure that CtrlNode will use to automatically generate the control widget.     """

    sigOutputChanged = QtCore.Signal(object)  # self
    sigClosed = QtCore.Signal(object)
    sigRenamed = QtCore.Signal(object, object)
    sigTerminalRenamed = QtCore.Signal(object, object)  # term, oldName
    sigTerminalAdded = QtCore.Signal(object, object)  # self, term
    sigTerminalRemoved = QtCore.Signal(object, object)  # self, term

    def __init__(self, name, params, yaml, graph):
        """
        ==============  ============================================================
        **Arguments:**
        name            The name of this specific node instance. It can be any 
                        string, but must be unique within a flowchart. Usually,
                        we simply let the flowchart decide on a name when calling
                        Flowchart.addNode(...)
        terminals       Dict-of-dicts specifying the terminals present on this Node.
                        Terminal specifications look like::
                        
                            'inputTerminalName': {'io': 'in'}
                            'outputTerminalName': {'io': 'out'} 
                            
                        There are a number of optional parameters for terminals:
                        multi, pos, renamable, removable, multiable, bypass. See
                        the Terminal class for more information.
        allowAddInput   bool; whether the user is allowed to add inputs by the
                        context menu.
        allow_add_output  bool; whether the user is allowed to add outputs by the
                        context menu.
        allow_remove     bool; whether the user is allowed to remove this node by the
                        context menu.
        ==============  ============================================================  
        
        """
        QtCore.QObject.__init__(self)

        self._name = name
        self._graphicsItem = None
        self.terminals = OrderedDict()
        self._inputs = OrderedDict()
        self._outputs = OrderedDict()
        self._params = params
        self._yaml = yaml
        self._graph = None
        self._graph = weakref.ref(graph)
        self.exception = None

        if 'config_name' in params['default'] and params['default']['config_name'] in ['actions', 'observations']:
            self._node_type = params['default']['config_name']
            self._allow_add_terminal = True
            self._allow_remove = False
            self._is_object = False
        else:
            self._node_type = get_yaml_type(yaml)
            self._allow_add_terminal = self._is_object = self._node_type == 'object'
            self._allow_remove = True
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
        if 'default' in self._params:
            for terminal_type in set.union(constants.TERMS_IN, constants.TERMS_OUT):
                if terminal_type in self._params['default']:
                    for terminal in self._params['default'][terminal_type]:
                        name = terminal_type + '/' + terminal
                        self.addTerminal(name=name)
                        if self._node_type == 'reset_node' and terminal_type == 'outputs':
                            name = 'feedthroughs/' + terminal
                            self.addTerminal(name=name)

    def removeTerminal(self, term):
        """Remove the specified terminal from this Node. May specify either the 
        terminal's name or the terminal itself.
        
        Causes sigTerminalRemoved to be emitted."""
        if isinstance(term, Terminal):
            name = term.name()
        else:
            name = term
            term = self.terminals[name]
        term.close()

        name_split = name.split('/')
        terminal_name = name_split[-1]
        terminal_type = name_split[0]

        del self.terminals[name]
        if term.isInput():
            del self._inputs[name]
        if term.isOutput():
            del self._outputs[name]

        self._params['default'][terminal_type].remove(terminal_name)

        if not terminal_type in self._yaml:
            self._params.pop(terminal_type)
        elif terminal_name not in self._yaml[terminal_type]:
            self._params[terminal_type].pop(terminal_name)

        self.graphicsItem().updateTerminals()
        self.sigTerminalRemoved.emit(self, term)

    def terminalRenamed(self, term, old_name):
        """Called after a terminal has been renamed        
        
        Causes sigTerminalRenamed to be emitted."""
        new_name = term.name()
        for d in [self.terminals, self._inputs, self._outputs]:
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

        self._params['default'][terminal_type].append(new_terminal_name)
        self._params['default'][terminal_type].remove(old_terminal_name)
        self._params[terminal_type][new_terminal_name] = self._params[terminal_type][old_terminal_name]
        self._params[terminal_type].pop(old_terminal_name)

        self.graphicsItem().updateTerminals()
        self.sigTerminalRenamed.emit(term, old_name)

    def addTerminal(self, name, **opts):
        """Add a new terminal to this Node with the given name. Extra
        keyword arguments are passed to Terminal.__init__.
                
        Causes sigTerminalAdded to be emitted."""
        name = self.__next_terminal_name(name)
        name_split = name.split('/')
        terminal_type = name_split[0]
        terminal_name = name_split[-1]

        if terminal_type in constants.TERMS_IN:
            opts['io'] = 'in'
        else:
            opts['io'] = 'out'
        term = Terminal(self, name, **opts)
        self.terminals[name] = term

        if terminal_type in self._params['default'] and terminal_name not in self._params['default'][terminal_type]:
            self._params['default'][terminal_type].append(terminal_name)
        if terminal_type not in self._params:
            self._params[terminal_type] = {}
        if terminal_name not in self._params[terminal_type]:
            self._params[terminal_type][terminal_name] = {}

        if terminal_type in ['inputs', 'actuators']:
            init_function = RxInput.__init__
        elif terminal_type in ['outputs', 'sensors']:
            init_function = RxOutput.__init__
        # elif terminal_type in ['feedthroughs']:
        #     init_function = RxFeedthrough.__init__
        elif terminal_type in ['states', 'targets']:
            init_function = RxState.__init__
        else:
            init_function = None

        if init_function is not None:
            argspec = inspect.getfullargspec(init_function)
            default_values = argspec.defaults
            default_keys = argspec.args[-len(default_values):]
            for key, value in zip(default_keys, default_values):
                if key in ['msg_module']:
                    continue
                if self._node_type in constants.GUI_IGNORE_DEFAULT:
                    if key in constants.GUI_IGNORE_DEFAULT[self._node_type]:
                        continue
                if key not in self._params[terminal_type][terminal_name]:
                    if key == 'rate' and value is None:
                        value = 1.
                    self._params[terminal_type][terminal_name][key] = value

        if term.isInput():
            self._inputs[name] = term
        elif term.isOutput():
            self._outputs[name] = term

        self.graphicsItem().updateTerminals()
        self.sigTerminalAdded.emit(self, term)
        return term

    def inputs(self):
        """Return dict of all input terminals.
        Warning: do not modify."""
        return self._inputs

    def outputs(self):
        """Return dict of all output terminals.
        Warning: do not modify."""
        return self._outputs

    def process(self, **kargs):
        """Process data through this node. This method is called any time the flowchart 
        wants the node to process data. It will be called with one keyword argument
        corresponding to each input terminal, and must return a dict mapping the name
        of each output terminal to its new value.
        
        This method is also called with a 'display' keyword argument, which indicates
        whether the node should update its display (if it implements any) while processing
        this data. This is primarily used to disable expensive display operations
        during batch processing.
        """
        return {}

    def graphicsItem(self):
        """Return the GraphicsItem for this node. Subclasses may re-implement
        this method to customize their appearance in the flowchart."""
        if self._graphicsItem is None:
            self._graphicsItem = NodeGraphicsItem(self)
        return self._graphicsItem

    def graph(self):
        return self._graph

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

    def name(self):
        """Return the name of this node."""
        return self._name

    def params(self):
        """Return the parameters of this node."""
        return self._params

    def rx_params(self):
        if self._node_type in ['actions', 'observations']:
            rx_params = RxNodeParams.create('env/{}'.format(self._node_type), package_name='eagerx_core',
                                            config_name=self._node_type)
        elif self._is_object:
            rx_params = RxObjectParams(name=self._name, params=deepcopy(self._params))
        else:
            rx_params = RxNodeParams(name=self._name, params=deepcopy(self._params))
        return rx_params

    def node_type(self):
        return self._node_type

    def graph(self):
        """Return the graph."""
        return self._graph

    def info(self):
        info = deepcopy(self._params['default'])
        return info

    def is_object(self):
        return self._is_object

    def yaml(self):
        return self._yaml

    def allow_add_terminal(self):
        return self._allow_add_terminal

    def rename(self, name):
        """Rename this node. This will cause sigRenamed to be emitted."""
        oldName = self._name
        self._name = name
        self.sigRenamed.emit(self, oldName)

    def dependentNodes(self):
        """Return the list of nodes which provide direct input to this node"""
        nodes = set()
        for t in self.inputs().values():
            nodes |= set([i.node() for i in t.inputTerminals()])
        return nodes

    def __repr__(self):
        return "<Node %s @%x>" % (self.name(), id(self))

    def ctrlWidget(self):
        """Return this Node's control widget. 
        
        By default, Nodes have no control widget. Subclasses may reimplement this 
        method to provide a custom widget. This method is called by Flowcharts
        when they are constructing their Node list."""
        return None

    def setInput(self, **args):
        """Set the values on input terminals. For most nodes, this will happen automatically through Terminal.inputChanged.
        This is normally only used for nodes with no connected inputs."""
        changed = False
        for k, v in args.items():
            term = self._inputs[k]
            oldVal = term.value()
            if not fn.eq(oldVal, v):
                changed = True
            term.setValue(v, process=False)
        if changed and '_updatesHandled_' not in args:
            self.update()

    def inputValues(self):
        """Return a dict of all input values currently assigned to this node."""
        vals = {}
        for n, t in self.inputs().items():
            vals[n] = t.value()
        return vals

    def outputValues(self):
        """Return a dict of all output values currently generated by this node."""
        vals = {}
        for n, t in self.outputs().items():
            vals[n] = t.value()
        return vals

    def connected(self, localTerm, remoteTerm):
        """Called whenever one of this node's terminals is connected elsewhere."""
        pass

    def disconnected(self, localTerm, remoteTerm):
        """Called whenever one of this node's terminals is disconnected from another."""
        pass

    def update(self, signal=True):
        """Collect all input values, attempt to process new output values, and propagate downstream.
        Subclasses should call update() whenever thir internal state has changed
        (such as when the user interacts with the Node's control widget). Update
        is automatically called when the inputs to the node are changed.
        """
        # vals = self.inputValues()
        # try:
        #     out = self.process(**strDict(vals))
        #     if out is not None:
        #         if signal:
        #             self.setOutput(**out)
        #         else:
        #             self.setOutputNoSignal(**out)
        #     for n, t in self.inputs().items():
        #         t.setValueAcceptable(True)
        #     self.clearException()
        # except:
        #     for n, t in self.outputs().items():
        #         t.setValue(None)
        #     self.setException(sys.exc_info())
        #
        #     if signal:
        #         self.sigOutputChanged.emit(self)  ## triggers flowchart to propagate new data
        pass

    def setOutput(self, **vals):
        self.setOutputNoSignal(**vals)
        self.sigOutputChanged.emit(self)  ## triggers flowchart to propagate new data

    def setOutputNoSignal(self, **vals):
        for k, v in vals.items():
            term = self.outputs()[k]
            term.setValue(v)
            term.setValueAcceptable(True)

    def setException(self, exc):
        self.exception = exc
        self.recolor()

    def clearException(self):
        self.setException(None)

    def recolor(self):
        if self.exception is None:
            self.graphicsItem().setPen(QtGui.QPen(QtGui.QColor(0, 0, 0)))
        else:
            self.graphicsItem().setPen(QtGui.QPen(QtGui.QColor(150, 0, 0), 3))

    def saveState(self):
        """Return a dictionary representing the current state of this node
        (excluding input / output values). This is used for saving/reloading
        flowcharts. The default implementation returns this Node's position,
        bypass state, and information about each of its terminals. 
        
        Subclasses may want to extend this method, adding extra keys to the returned
        dict."""
        pos = self.graphicsItem().pos()
        state = {'pos': (pos.x(), pos.y()), 'params': self._params, 'yaml': self._yaml}
        termsEditable = self._allow_add_terminal
        for term in list(self._inputs.values()) + list(self._outputs.values()):
            termsEditable |= term._renamable | term._removable
        if termsEditable:
            state['terminals'] = self.saveTerminals()
        return state

    def restoreState(self, state):
        """Restore the state of this node from a structure previously generated
        by saveState(). """
        pos = state.get('pos', (0, 0))
        self.graphicsItem().setPos(*pos)
        if 'terminals' in state:
            self.restoreTerminals(state['terminals'])

    def saveTerminals(self):
        terms = OrderedDict()
        for n, t in self.terminals.items():
            terms[n] = (t.saveState())
        return terms

    def restoreTerminals(self, state):
        for name in list(self.terminals.keys()):
            if name not in state:
                self.removeTerminal(name)
        for name, opts in state.items():
            if name in self.terminals:
                term = self[name]
                term.setOpts(**opts)
                continue
            try:
                opts = strDict(opts)
                self.addTerminal(name, **opts)
            except:
                printExc("Error restoring terminal %s (%s):" % (str(name), str(opts)))

    def clearTerminals(self):
        for t in self.terminals.values():
            t.close()
        self.terminals = OrderedDict()
        self._inputs = OrderedDict()
        self._outputs = OrderedDict()

    def close(self):
        """Cleans up after the node--removes terminals, graphicsItem, widget"""
        self.disconnectAll()
        self.clearTerminals()
        item = self.graphicsItem()
        if item.scene() is not None:
            item.scene().removeItem(item)
        self._graphicsItem = None
        w = self.ctrlWidget()
        if w is not None:
            w.setParent(None)
        self.sigClosed.emit(self)

    def disconnectAll(self):
        for t in self.terminals.values():
            t.disconnectAll()


class NodeGraphicsItem(GraphicsObject):
    def __init__(self, node):
        GraphicsObject.__init__(self)
        self._node_type = node.node_type()
        self._params = node.params()
        self._yaml = node.yaml()

        if self._node_type == 'node':
            color = np.array([0, 255, 255])
        elif self._node_type == 'reset_node':
            color = np.array([0, 255, 0])
        elif self._node_type == 'object':
            color = np.array([255, 255, 0])
        else:
            color = np.array([200, 200, 200])

        self.pen = fn.mkPen(0, 0, 0)
        self.selectPen = fn.mkPen(200, 200, 200, width=2)
        self.brush = fn.mkBrush(np.append(color, 200))
        self.hoverBrush = fn.mkBrush(np.append(color, 225))
        self.selectBrush = fn.mkBrush(np.append(color, 255))
        self.hovered = False

        self.node = node
        flags = self.ItemIsMovable | self.ItemIsSelectable | self.ItemIsFocusable | self.ItemSendsGeometryChanges

        self.setFlags(flags)
        self.bounds = QtCore.QRectF(0, 0, 125, 125)
        self.nameItem = QtGui.QGraphicsTextItem(self.node.name(), self)
        self.nameItem.setDefaultTextColor(QtGui.QColor(50, 50, 50))
        self.nameItem.moveBy(self.bounds.width() / 2. - self.nameItem.boundingRect().width() / 2., 0)
        self.nameItem.setTextInteractionFlags(QtCore.Qt.TextEditorInteraction)
        self.updateTerminals()

        self.nameItem.focusOutEvent = self.labelFocusOut
        self.nameItem.keyPressEvent = self.labelKeyPress

        self.menu = None
        self.buildMenu()
        self.initial_z_value = self.zValue()
        self.initialise_param_window()

    def initialise_param_window(self):
        graph = self.node.graph()
        self.param_window = QtGui.QMainWindow(graph().widget().cwWin)
        self.param_window.setWindowTitle('Parameters {}'.format(self.node.name()))
        cw = QtGui.QWidget()
        self.layout = QtGui.QGridLayout()
        cw.setLayout(self.layout)
        self.param_window.setCentralWidget(cw)
        self.labels = []
        self.widgets = []
        row = 1
        for key, value in self.node.params()['default'].items():
            label = QtGui.QLabel(key)
            if isinstance(value, bool):
                items = ['True', 'False']
                widget = ComboBox(items=items, default=str(value))
                widget.activated.connect(partial(self.combo_box_value_changed, key=key, items=items))
            elif key in constants.GUI_NODE_ITEMS:
                items = constants.GUI_NODE_ITEMS[key]
                widget = ComboBox(items=items, default=str(value))
                widget.activated.connect(partial(self.combo_box_value_changed, key=key, items=items))
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
                self.widgets[idx].setText(str(self.node.params()['default'][label.text()]))

    def combo_box_value_changed(self, int, items, key):
        self.node.params()['default'][key] = items[int]

    def value_changed(self, widget, key):
        self.node.params()['default'][key] = widget.value()

    def text_changed(self, text, key):
        self.node.params()['default'][key] = text

    def labelFocusOut(self, ev):
        QtGui.QGraphicsTextItem.focusOutEvent(self.nameItem, ev)
        self.labelChanged()

    def labelKeyPress(self, ev):
        if ev.key() == QtCore.Qt.Key_Enter or ev.key() == QtCore.Qt.Key_Return:
            self.labelChanged()
        else:
            QtGui.QGraphicsTextItem.keyPressEvent(self.nameItem, ev)

    def labelChanged(self):
        newName = str(self.nameItem.toPlainText())
        graph = self.node.graph()
        if graph is not None and newName in graph().nodes().keys():
            self.nameItem.setPlainText(self.node.name())
            return
        if newName != self.node.name():
            self.node.rename(newName)
        ### re-center the label
        bounds = self.boundingRect()
        self.nameItem.setPos(bounds.width() / 2. - self.nameItem.boundingRect().width() / 2., 0)

    def setPen(self, *args, **kwargs):
        self.pen = fn.mkPen(*args, **kwargs)
        self.update()

    def setBrush(self, brush):
        self.brush = brush
        self.update()

    def updateTerminals(self):
        bounds = self.bounds
        self.terminals = {}
        inp = self.node.inputs()
        dy = bounds.height() / (len(inp) + 1)
        y = dy
        for i, t in inp.items():
            item = t.graphicsItem()
            item.setParentItem(self)
            br = self.bounds
            item.setAnchor(0, y)
            self.terminals[i] = (t, item)
            y += dy

        out = self.node.outputs()
        dy = bounds.height() / (len(out) + 1)
        y = dy
        for i, t in out.items():
            item = t.graphicsItem()
            item.setParentItem(self)
            item.setZValue(self.initial_z_value)
            br = self.bounds
            item.setAnchor(bounds.width(), y)
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
            if not self.node._allow_remove:
                return
            self.node.close()
        else:
            ev.ignore()

    def itemChange(self, change, val):
        if change == self.ItemPositionHasChanged:
            for k, t in self.terminals.items():
                t[1].nodeMoved()
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

        if self.node.allow_add_terminal():
            if self._node_type in ['observations', 'actions']:
                terminal = self._node_type[:-1]
                if self._node_type == 'observations':
                    terminal_name = 'inputs/' + terminal
                else:
                    terminal_name = 'outputs/' + terminal
                self.menu.addAction('Add {}'.format(terminal), partial(self.node.addTerminal, name=terminal_name))
            else:
                for terminal_type in set.union(constants.TERMS[self._node_type]['in'],
                                               constants.TERMS[self._node_type]['out']):
                    terminal_menu = QtGui.QMenu('Add {}'.format(terminal_type[:-1]), self.menu)
                    for terminal in self._yaml[terminal_type]:
                        terminal_name = str(terminal_type) + '/' + terminal
                        act = terminal_menu.addAction(terminal, partial(self.node.addTerminal, name=terminal_name))
                        if terminal_name in self.node.terminals.keys():
                            act.setEnabled(False)
                        self.menu.addMenu(terminal_menu)
        if self.node._allow_remove:
            self.menu.addAction("Remove {}".format(self.node.name()), self.node.close)
