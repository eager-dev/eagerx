# -*- coding: utf-8 -*-
import weakref
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

    def __init__(self, name, terminals=None, allowAddInput=False, allowAddOutput=False, allowRemove=True,
                 node_type=None, node_params={}, graph=None):
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
        allowAddOutput  bool; whether the user is allowed to add outputs by the
                        context menu.
        allowRemove     bool; whether the user is allowed to remove this node by the
                        context menu.
        ==============  ============================================================  
        
        """
        QtCore.QObject.__init__(self)
        self._name = name
        self._bypass = False
        self.bypassButton = None  ## this will be set by the flowchart ctrl widget..
        self._graphicsItem = None
        self.terminals = OrderedDict()
        self._inputs = OrderedDict()
        self._outputs = OrderedDict()
        self._allowAddInput = allowAddInput  ## flags to allow the user to add/remove terminals
        self._allowAddOutput = allowAddOutput
        self._allowRemove = allowRemove
        self._node_type = node_type
        self._node_params = node_params
        self._graph = None
        self._info = {}

        if graph is not None:
            self._graph = weakref.ref(graph)

        self.exception = None
        if 'default' in node_params:
            self.updateInfo(**node_params['default'])
        if terminals is None:
            if node_type == 'object':
                if 'default' in node_params:
                    if 'actuators' in node_params['default']:
                        for input in node_params['default']['actuators']:
                            self.addInput(name=input, removable=True)
                    if 'sensors' in node_params['default']:
                        for output in node_params['default']['sensors']:
                            self.addOutput(name=output, removable=True)
                    if 'states' in node_params['default']:
                        for state in node_params['default']['states']:
                            self.addOutput(name=state, removable=True, is_state=True)
            elif node_type == 'real_reset':
                if 'inputs' in node_params:
                    for input in node_params['inputs'].keys():
                        self.addInput(name=input)
                if 'outputs' in node_params:
                    for output in node_params['outputs'].keys():
                        self.addOutput(name=output)
                if 'states' in node_params:
                    for output in node_params['states'].keys():
                        self.addOutput(name=output, is_state=True, connectable=False)
                if 'targets' in node_params:
                    for target in node_params['targets'].keys():
                        self.addInput(name=target, is_state=True)
            elif node_type == 'node':
                if 'inputs' in node_params:
                    for input in node_params['inputs'].keys():
                        self.addInput(name=input)
                if 'outputs' in node_params:
                    for output in node_params['outputs'].keys():
                        self.addOutput(name=output)
                if 'states' in node_params:
                    for output in node_params['states'].keys():
                        self.addOutput(name=output, is_state=True, connectable=False)
        else:
            for name, opts in terminals.items():
                self.addTerminal(name, **opts)

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
        del self.terminals[name]
        if name in self._inputs:
            del self._inputs[name]
            if term.isState():
                states = self._info['states']
                states.remove(name)
                self.updateInfo(states=states)
            else:
                if self.node_type() == 'object':
                    actuators = self._info['actuators']
                    actuators.remove(name)
                    self.updateInfo(actuators=actuators)
                else:
                    inputs = self._info['inputs']
                    inputs.remove(name)
                    self.updateInfo(inputs=inputs)
        if name in self._outputs:
            del self._outputs[name]
            if term.isState():
                states = self._info['states']
                states.remove(name)
                self.updateInfo(states=states)
            else:
                if self.node_type() == 'object':
                    sensors = self._info['sensors']
                    sensors.remove(name)
                    self.updateInfo(sensors=sensors)
                else:
                    outputs = self._info['outputs']
                    outputs.remove(name)
                    self.updateInfo(inputs=outputs)
        self.graphicsItem().updateTerminals()
        self.sigTerminalRemoved.emit(self, term)

    def terminalRenamed(self, term, oldName):
        """Called after a terminal has been renamed        
        
        Causes sigTerminalRenamed to be emitted."""
        newName = term.name()
        for d in [self.terminals, self._inputs, self._outputs]:
            if oldName not in d:
                continue
            d[newName] = d[oldName]
            del d[oldName]

        self.graphicsItem().updateTerminals()
        self.sigTerminalRenamed.emit(term, oldName)

    def addTerminal(self, name, **opts):
        """Add a new terminal to this Node with the given name. Extra
        keyword arguments are passed to Terminal.__init__.
                
        Causes sigTerminalAdded to be emitted."""
        name = self.nextTerminalName(name)
        term = Terminal(self, name, **opts)
        self.terminals[name] = term
        if term.isInput():
            self._inputs[name] = term
            if self.node_type() == 'object':
                if 'actuators' not in self._info or not isinstance(self._info['actuators'], list):
                    self.updateInfo(actuators=[name])
                elif name not in self._info['actuators']:
                    actuators = self._info['actuators']
                    actuators.append(name)
                    self.updateInfo(actuators=actuators)
        elif term.isOutput():
            self._outputs[name] = term
            if self.node_type() == 'object':
                if 'is_state' in opts.keys() and opts['is_state']:
                    if 'states' not in self._info or not isinstance(self._info['states'], list):
                        self.updateInfo(states=[name])
                    elif name not in self._info['states']:
                        states = self._info['states']
                        states.append(name)
                        self.updateInfo(states=states)
                else:
                    if 'sensors' not in self._info or not isinstance(self._info['sensors'], list):
                        self.updateInfo(sensors=[name])
                    elif name not in self._info['sensors']:
                        sensors = self._info['sensors']
                        sensors.append(name)
                        self.updateInfo(sensors=sensors)
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

    def node_type(self):
        """Return the type of this node."""
        return self._node_type

    def node_params(self):
        """Return the parameters of this node."""
        return self._node_params

    def graph(self):
        """Return the graph."""
        return self._graph

    def info(self):
        return self._info

    def updateInfo(self, **kwargs):
        for key, value in kwargs.items():
            self._info[key] = value

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

    def bypass(self, byp):
        """Set whether this node should be bypassed.
        
        When bypassed, a Node's process() method is never called. In some cases,
        data is automatically copied directly from specific input nodes to 
        output nodes instead (see the bypass argument to Terminal.__init__). 
        This is usually called when the user disables a node from the flowchart 
        control panel.
        """
        self._bypass = byp
        if self.bypassButton is not None:
            self.bypassButton.setChecked(byp)
        self.update()

    def isBypassed(self):
        """Return True if this Node is currently bypassed."""
        return self._bypass

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
            if self.isBypassed():
                out = self.processBypassed(vals)
            else:
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

    def processBypassed(self, args):
        """Called when the flowchart would normally call Node.process, but this node is currently bypassed.
        The default implementation looks for output terminals with a bypass connection and returns the
        corresponding values. Most Node subclasses will _not_ need to reimplement this method."""
        result = {}
        for term in list(self.outputs().values()):
            byp = term.bypassValue()
            if byp is None:
                result[term.name()] = None
            else:
                result[term.name()] = args.get(byp, None)
        return result

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
        state = {'pos': (pos.x(), pos.y()), 'bypass': self.isBypassed()}
        termsEditable = self._allowAddInput | self._allowAddOutput
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
        self.bypass(state.get('bypass', False))
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
        self._node_params = node.node_params()
        
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
        self.bounds = QtCore.QRectF(0, 0, 100, 100)
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
            if not self.node._allowRemove:
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
        if self.node._allowAddInput:
            if self._node_type == 'observation':
                self.menu.addAction(add_input_text, partial(self.addInputFromMenu, name='observation'))
            elif self._node_type == 'object':
                sensor_menu = QtGui.QMenu(add_input_text, self.menu)
                for actuator in self._node_params['actuators'].keys():
                    act = sensor_menu.addAction(
                        actuator,
                        partial(self.addInputFromMenu, name=actuator, multiable=False, renamable=False)
                    )
                    if actuator in self.node.inputs().keys():
                        act.setEnabled(False)
                self.menu.addMenu(sensor_menu)
            else:
                self.menu.addAction(add_input_text, self.addInputFromMenu)
        if self.node._allowAddOutput:
            if self._node_type == 'action':
                self.menu.addAction(add_output_text, partial(self.addOutputFromMenu, name='action'))
            elif self._node_type == 'object':
                sensor_menu = QtGui.QMenu(add_output_text, self.menu)
                state_menu = QtGui.QMenu('Add state', self.menu)
                for state in self._node_params['states'].keys():
                    act = state_menu.addAction(
                        state,
                        partial(self.addOutputFromMenu, name=state, multiable=False, renamable=False, is_state=True)
                    )
                    if state in self.node.outputs().keys():
                        act.setEnabled(False)
                self.menu.addMenu(state_menu)
                for sensor in self._node_params['sensors'].keys():
                    act = sensor_menu.addAction(
                        sensor,
                        partial(self.addOutputFromMenu, name=sensor, multiable=False, renamable=False)
                    )
                    if sensor in self.node.outputs().keys():
                        act.setEnabled(False)
                self.menu.addMenu(sensor_menu)
            else:
                self.menu.addAction(add_output_text, self.addOutputFromMenu)
        if self.node._allowRemove:
            self.menu.addAction("Remove {}".format(self._node_type), self.node.close)

    def addInputFromMenu(self, name='Input', renamable=True, removable=True, multiable=True):  ## called when add input is clicked in context menu
        self.node.addInput(name=name, renamable=renamable, removable=removable, multiable=multiable)

    def addOutputFromMenu(self, name='Output', renamable=True, removable=True, multiable=True, is_state=False):  ## called when add output is clicked in context menu
        self.node.addOutput(name=name, renamable=renamable, removable=removable, multiable=multiable, is_state=is_state)

    def addConverterFromMenu(self, name, type='input', converter_arguments=None):  ## called when add input is clicked in context menu
        self.node.addConverter(name, type=type, converter_arguments=converter_arguments)
