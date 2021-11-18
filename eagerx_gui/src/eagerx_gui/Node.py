# -*- coding: utf-8 -*-
from copy import deepcopy
from functools import partial
from eagerx_gui.Terminal import *
from pyqtgraph.pgcollections import OrderedDict
from pyqtgraph.debug import *
import numpy as np


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

        if not('node_type' in params.keys()):
            node_type = 'object'
        elif params['default']['config_name'] == 'actions':
            node_type = 'actions'
        elif params['default']['config_name'] == 'observations':
            node_type = 'observations'
        elif 'feedthroughs' in params.keys():
            node_type = 'real_reset'
        else:
            node_type = 'node'
        self._node_type = node_type
        self._is_object = node_type == 'object'

        allow_remove = False
        allow_add_input = False
        allow_add_output = False

        if self._node_type == 'actions':
            allow_add_output = True
            allow_remove = False
        elif self._node_type == 'observations':
            allow_add_input = True
            allow_remove = False
        elif self._is_object:
            allow_add_input = True
            allow_add_output = True

        self._allow_remove = allow_remove
        self._allow_add_input = allow_add_input
        self._allow_add_output = allow_add_output

        self.exception = None
        self.initializeTerminals()

    def nextTerminalName(self, name):
        """Return an unused terminal name"""
        name2 = name
        i = 1
        while name2 in self.terminals:
            name2 = "%s_%d" % (name, i)
            i += 1
        return name2

    def addInput(self, name="Input", **args):
        """Add a new input terminal to this Node with the given name. Extra
        keyword arguments are passed to Terminal.__init__.
        
        This is a convenience function that just calls addTerminal(io='in', ...)"""
        # print "Node.addInput called."
        return self.addTerminal(name, io='in', **args)

    def addOutput(self, name="Output", **args):
        """Add a new output terminal to this Node with the given name. Extra
        keyword arguments are passed to Terminal.__init__.
        
        This is a convenience function that just calls addTerminal(io='out', ...)"""
        return self.addTerminal(name, io='out', **args)

    def initializeTerminals(self):
        if 'default' in self._params:
            for output_type in ['sensors', 'states', 'outputs']:
                if self._node_type == 'observations':
                    break
                is_state = output_type == 'states'
                if output_type in self._params['default']:
                    if self._is_object:
                        for output in self._params['default'][output_type]:
                            name = output_type + '/' + output
                            self.addOutput(name=name, is_state=is_state, removable=True,
                                           params=self._params[output_type][output])
                    else:
                        connectable = not is_state
                        for output in self._params[output_type]:
                            name = output_type + '/' + output
                            self.addOutput(name=name, is_state=is_state, params=self._params[output_type][output],
                                           connectable=connectable)
            for input_type in ['actuators', 'inputs', 'targets', 'feedthroughs']:
                if self._node_type == 'actions':
                    break
                is_state = input_type == 'targets'
                is_feedthrough = input_type == 'feedthroughs'
                if input_type in self._params['default']:
                    if self._is_object:
                        for input in self._params['default'][input_type]:
                            name = input_type + '/' + input
                            self.addInput(name=name, is_state=is_state, removable=True,
                                          params=self._params[input_type][input])
                    else:
                        for input in self._params[input_type]:
                            name = input_type + '/' + input
                            if is_feedthrough:
                                params = self._params['outputs'][input]
                            else:
                                params = self._params[input_type][input]
                            self.addInput(name=name, is_state=is_state, params=params, is_feedthrough=is_feedthrough)

    def removeTerminal(self, term):
        """Remove the specified terminal from this Node. May specify either the 
        terminal's name or the terminal itself.
        
        Causes sigTerminalRemoved to be emitted."""
        if isinstance(term, Terminal):
            name = term.name()
            name_split = term.nameSplit()
        else:
            name = term
            name_split = name.split('/')[-1]
            term = self.terminals[name]

        term.close()
        del self.terminals[name]
        default = self._params['default']
        if term.isInput():
            del self._inputs[name]
            if self.node_type() == 'object':
                default['actuators'].remove(name_split)
            elif term.isFeedthrough():
                default['feedthroughs'].pop(name_split)
            elif term.isState():
                default['targets'].pop(name_split)
            else:
                default['inputs'].pop(name_split)
        if term.isOutput():
            del self._outputs[name]
            if self.node_type() == 'object':
                if term.isState():
                    default['states'].remove(name_split)
                else:
                    default['sensors'].remove(name_split)
            elif term.isState():
                default['states'].pop(name_split)
            else:
                default['outputs'].pop(name_split)
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

        old_name_split = old_name.split('/')[-1]
        new_name_split = new_name.split('/')[-1]

        default = self._params['default']
        if term.isInput():
            if term.isState():
                default['targets'][new_name_split] = default['targets'][old_name_split]
                default['targets'].pop(old_name_split)
            elif self.node_type() == 'object':
                default['actuators'].remove(old_name_split)
                default['actuators'].append(new_name_split)
            elif term.isFeedthrough():
                default['feedthroughs'][new_name_split] = default['feedthroughs'][old_name_split]
                default['feedthroughs'].pop(old_name_split)
            else:
                default['inputs'][new_name_split] = default['inputs'][old_name_split]
                default['inputs'].pop(old_name_split)
        if term.isOutput():
            if self._node_type == 'object':
                if term.isState():
                    default['states'].remove(old_name_split)
                    default['states'].append(new_name_split)
                else:
                    default['sensors'].remove(old_name_split)
                    default['sensors'].append(new_name_split)
            elif term.isState():
                default['states'][new_name_split] = self.name() + '/' + new_name
                default['states'].pop(old_name_split)
            else:
                default['outputs'][new_name_split] = self.name() + '/' + new_name
                default['outputs'].pop(old_name_split)
        self.graphicsItem().updateTerminals()
        self.sigTerminalRenamed.emit(term, old_name)

    def addTerminal(self, name, **opts):
        """Add a new terminal to this Node with the given name. Extra
        keyword arguments are passed to Terminal.__init__.
                
        Causes sigTerminalAdded to be emitted."""
        name = self.nextTerminalName(name)
        term = Terminal(self, name, **opts)
        self.terminals[name] = term
        name_split = name.split('/')[-1]

        default = self._params['default']

        if term.isInput():
            self._inputs[name] = term
            if self.node_type() == 'object':
                if 'actuators' not in default or not isinstance(default['actuators'], list):
                    default['actuators'] = [name_split]
                elif name_split not in default['actuators']:
                    default['actuators'].append(name_split)
            elif 'is_feedthrough' in opts.keys() and opts['is_feedthrough']:
                if 'feedthroughs' not in default or not isinstance(default['feedthroughs'], dict):
                    default['feedthroughs'] = {name_split: None}
                elif name_split not in default['feedthroughs'].keys():
                    default['feedthroughs'][name_split] = None
            elif 'is_state' in opts.keys() and opts['is_state']:
                if 'targets' not in default or not isinstance(default['targets'], dict):
                    default['targets'] = {name_split: None}
                elif name_split not in default['targets'].keys():
                    default['targets'][name_split] = None
            else:
                if 'inputs' not in default or not isinstance(default['inputs'], dict):
                    default['inputs'] = {name_split: None}
                elif name_split not in default['inputs'].keys():
                    default['inputs'][name_split] = None
        elif term.isOutput():
            self._outputs[name] = term
            if self.node_type() == 'object':
                if 'is_state' in opts.keys() and opts['is_state']:
                    if 'states' not in default or not isinstance(default['states'], list):
                        default['states'] = [name_split]
                    elif name_split not in default['states']:
                        default['states'].append(name_split)
                elif 'sensors' not in default or not isinstance(default['sensors'], list):
                    default['sensors'] = [name_split]
                elif name_split not in default['sensors']:
                    default['sensors'].append(name_split)
            elif 'is_state' in opts.keys() and opts['is_state']:
                if 'states' not in default or not isinstance(default['states'], dict):
                    default['states'] = {name_split: self.name() + '/' + name}
                elif name_split not in default['states'].keys():
                    default['states'][name_split] = self.name() + '/' + name
            else:
                if 'outputs' not in default or not isinstance(default['outputs'], dict):
                    default['outputs'] = {name_split: self.name() + '/' + name}
                elif name_split not in default['outputs'].keys():
                    default['outputs'][name_split] = self.name() + '/' + name
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

    ## this is just bad planning. Causes too many bugs.
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

    def node_type(self):
        return self._node_type

    def graph(self):
        """Return the graph."""
        return self._graph

    def info(self):
        self.updateInfo()
        return self._info

    def yaml(self):
        return self._yaml

    def updateInfo(self):
        self._info = deepcopy(self._params['default'])

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
        vals = self.inputValues()
        try:
            out = self.process(**strDict(vals))
            if out is not None:
                if signal:
                    self.setOutput(**out)
                else:
                    self.setOutputNoSignal(**out)
            for n, t in self.inputs().items():
                t.setValueAcceptable(True)
            self.clearException()
        except:
            for n, t in self.outputs().items():
                t.setValue(None)
            self.setException(sys.exc_info())

            if signal:
                self.sigOutputChanged.emit(self)  ## triggers flowchart to propagate new data

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
        termsEditable = self._allow_add_input | self._allow_add_output
        for term in list(self._inputs.values()) + list(self._outputs.values()):
            termsEditable |= term._renamable | term._removable | term._multiable
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

        if self._node_type == 'node':
            color = np.array([0, 255, 255])
        elif self._node_type == 'real_reset':
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
        # flags =  self.ItemIsFocusable |self.ItemSendsGeometryChanges

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
        add_input_text = 'Add input'
        add_output_text = 'Add output'
        if self._node_type == 'action':
            add_output_text = 'Add action'
        elif self._node_type == 'object':
            add_input_text = 'Add actuator'
            add_output_text = 'Add sensor'
        elif self._node_type == 'observation':
            add_input_text = 'Add observation'
        if self.node._allow_add_input:
            if self._node_type == 'observation':
                self.menu.addAction(add_input_text, partial(self.addInputFromMenu, name='outputs/' + 'observation'))
            elif self._node_type == 'object':
                sensor_menu = QtGui.QMenu(add_input_text, self.menu)
                for actuator in self._params['actuators'].keys():
                    actuator_name = 'actuators/' + actuator
                    act = sensor_menu.addAction(
                        actuator,
                        partial(self.addInputFromMenu, name=actuator_name, multiable=False, renamable=False)
                    )
                    if actuator_name in self.node.inputs().keys():
                        act.setEnabled(False)
                self.menu.addMenu(sensor_menu)
            else:
                self.menu.addAction(add_input_text, self.addInputFromMenu)
        if self.node._allow_add_output:
            if self._node_type == 'action':
                self.menu.addAction(add_output_text, partial(self.addOutputFromMenu,
                                                             name='outputs/' + 'action'))
            elif self._node_type == 'object':
                sensor_menu = QtGui.QMenu(add_output_text, self.menu)
                state_menu = QtGui.QMenu('Add state', self.menu)
                for state in self._params['states'].keys():
                    state_name = 'states/' + state
                    act = state_menu.addAction(
                        state,
                        partial(self.addOutputFromMenu, name=state_name, multiable=False,
                                renamable=False, is_state=True)
                    )
                    if state_name in self.node.outputs().keys():
                        act.setEnabled(False)
                self.menu.addMenu(state_menu)
                for sensor in self._params['sensors'].keys():
                    sensor_name = 'sensors/' + sensor
                    act = sensor_menu.addAction(
                        sensor,
                        partial(self.addOutputFromMenu, name=sensor_name, multiable=False, renamable=False)
                    )
                    if sensor_name in self.node.outputs().keys():
                        act.setEnabled(False)
                self.menu.addMenu(sensor_menu)
            else:
                self.menu.addAction(add_output_text, self.addOutputFromMenu)
        if self.node._allow_remove:
            self.menu.addAction("Remove {}".format(self._node_type), self.node.close)

    def addInputFromMenu(self, name='Input', renamable=True, removable=True,
                         multiable=True, is_state=False):
        params = {}
        name_split = name.split('/')[-1]
        if is_state and 'targets' in self._params and name_split in self._params['targets']:
            params = self._params['targets'][name_split]
        elif self._node_type == 'object' and name_split in self._params['actuators']:
            params = self._params['actuators'][name_split]
        elif 'inputs' in self._params and name_split in self._params['inputs']:
            params = self._params['inputs'][name_split]
        self.node.addInput(name=name, renamable=renamable, removable=removable, multiable=multiable, params=params,
                           is_state=is_state)

    def addOutputFromMenu(self, name='Output', renamable=True, removable=True, multiable=True,
                          is_state=False):
        params = {}
        name_split = name.split('/')[-1]
        if is_state and 'states' in self._params and name_split in self._params['states']:
            params = self._params['states'][name_split]
        elif self._node_type == 'object' and name_split in self._params['sensors']:
            params = self._params['sensors'][name_split]
        elif 'outputs' in self._params and name_split in self._params['outputs']:
            params = self._params['outputs'][name_split]
        self.node.addOutput(name=name, renamable=renamable, removable=removable, multiable=multiable, is_state=is_state,
                            params=params)
