"""
 Mostly copy paste from https://github.com/pyqtgraph/pyqtgraph/blob/master/pyqtgraph/flowchart/Flowchart.py
"""

# -*- coding: utf-8 -*-
from pyqtgraph.Qt import QT_LIB
from pyqtgraph.flowchart.Node import *
from pyqtgraph import FileDialog, DataTreeWidget
import tabulate

## pyside and pyqt use incompatible ui files.
if QT_LIB == 'PySide':
    from eagerx_gui import EagerxGraphCtrlTemplate_pyside as EagerxGraphCtrlTemplate
elif QT_LIB == 'PySide2':
    from eagerx_gui import EagerxGraphCtrlTemplate_pyside2 as EagerxGraphCtrlTemplate
elif QT_LIB == 'PyQt5':
    from eagerx_gui import EagerxGraphCtrlTemplate_pyqt5 as EagerxGraphCtrlTemplate
else:
    from eagerx_gui import EagerxGraphCtrlTemplate_pyqt as EagerxGraphCtrlTemplate
    
from pyqtgraph.flowchart.Terminal import Terminal
from numpy import ndarray
from pyqtgraph import configfile as configfile
from pyqtgraph import dockarea as dockarea
from eagerx_gui import eagerx_graph_view
from pyqtgraph import functions as fn
from pyqtgraph.python2_3 import asUnicode

import os
from pyqtgraph.debug import printExc
from eagerx_gui.Node import Node as EagerxNode
from eagerx_gui.Terminal import Terminal as EagerxTerminal


def strDict(d):
    return dict([(str(k), v) for k, v in d.items()])


class EagerxGraph(Node):
    sigFileLoaded = QtCore.Signal(object)
    sigFileSaved = QtCore.Signal(object)
    sigChartLoaded = QtCore.Signal()
    sigStateChanged = QtCore.Signal()  # called when output is expected to have changed
    sigChartChanged = QtCore.Signal(object, object, object)  # called when nodes are added, removed, or renamed.
    
    def __init__(self, terminals=None, name=None, filePath=None, library=None):
        self.library = library
        if name is None:
            name = "EAGERX Graph"
        if terminals is None:
            terminals = {}
        self.filePath = filePath
        Node.__init__(self, name, allowAddInput=True, allowAddOutput=True)  # create node without terminals; we'll add these later

        self.inputWasSet = False  # flag allows detection of changes in the absence of input change.
        self._nodes = {}
        self.nextZVal = 10
        self._widget = None
        self._scene = None
        self.processing = False  # flag that prevents recursive node updates
        
        self.widget()
        
        self.inputNode = EagerxNode('Actions', allowRemove=False, allowAddOutput=True, node_type='action', graph=self)
        self.outputNode = EagerxNode('Observations', allowRemove=False, allowAddInput=True, node_type='observation', graph=self)
        self.addNode(self.inputNode, 'Actions', [-150, 0])
        self.addNode(self.outputNode, 'Observations', [300, 0])
        
        self.outputNode.sigOutputChanged.connect(self.outputChanged)
        self.outputNode.sigTerminalRenamed.connect(self.internalTerminalRenamed)
        self.inputNode.sigTerminalRenamed.connect(self.internalTerminalRenamed)
        self.outputNode.sigTerminalRemoved.connect(self.internalTerminalRemoved)
        self.inputNode.sigTerminalRemoved.connect(self.internalTerminalRemoved)
        self.outputNode.sigTerminalAdded.connect(self.internalTerminalAdded)
        self.inputNode.sigTerminalAdded.connect(self.internalTerminalAdded)
        
        self.viewBox.autoRange(padding=0.04)
            
        for name, opts in terminals.items():
            self.addTerminal(name, **opts)
      
    def setLibrary(self, lib):
        self.library = lib
        self.widget().chartWidget.buildMenu()
      
    def setInput(self, **args):
        """Set the input values of the flowchart. This will automatically propagate
        the new values throughout the flowchart, (possibly) causing the output to change.
        """
        self.inputWasSet = True
        self.inputNode.setOutput(**args)
        
    def outputChanged(self):
        ## called when output of internal node has changed
        vals = self.outputNode.inputValues()
        self.widget().outputChanged(vals)
        self.setOutput(**vals)
        
    def output(self):
        """Return a dict of the values on the Flowchart's output terminals.
        """
        return self.outputNode.inputValues()
        
    def nodes(self):
        return self._nodes
        
    def addTerminal(self, name, **opts):
        term = Node.addTerminal(self, name, **opts)
        name = term.name()
        if opts['io'] == 'in':  ## inputs to the flowchart become outputs on the input node
            opts['io'] = 'out'
            opts['multi'] = False
            self.inputNode.sigTerminalAdded.disconnect(self.internalTerminalAdded)
            try:
                term2 = self.inputNode.addTerminal(name, **opts)
            finally:
                self.inputNode.sigTerminalAdded.connect(self.internalTerminalAdded)
                
        else:
            opts['io'] = 'in'
            self.outputNode.sigTerminalAdded.disconnect(self.internalTerminalAdded)
            try:
                term2 = self.outputNode.addTerminal(name, **opts)
            finally:
                self.outputNode.sigTerminalAdded.connect(self.internalTerminalAdded)
        return term

    def removeTerminal(self, name):
        term = self[name]
        inTerm = self.internalTerminal(term)
        Node.removeTerminal(self, name)
        inTerm.node().removeTerminal(inTerm.name())
        
    def internalTerminalRenamed(self, term, oldName):
        self[oldName].rename(term.name())
        
    def internalTerminalAdded(self, node, term):
        if term._io == 'in':
            io = 'out'
        else:
            io = 'in'
        Node.addTerminal(self, term.name(), io=io, renamable=term.isRenamable(), removable=term.isRemovable(), multiable=term.isMultiable())
        
    def internalTerminalRemoved(self, node, term):
        try:
            Node.removeTerminal(self, term.name())
        except KeyError:
            pass
        
    def terminalRenamed(self, term, oldName):
        newName = term.name()
        Node.terminalRenamed(self, self[oldName], oldName)
        for n in [self.inputNode, self.outputNode]:
            if oldName in n.terminals:
                n[oldName].rename(newName)

    def createNode(self, node_definition, name=None, pos=None):
        """Create a new Node and add it to this flowchart.
        """
        if name is None:
            n = 0
            while True:
                name = '{}'.format(node_definition['name'])
                if n > 0:
                    name += '_{}'.format(n+1)
                if name not in self._nodes:
                    break
                n += 1
        allow_add_input = node_definition['type'] == 'object'
        allow_add_output = node_definition['type'] == 'object'
        node = EagerxNode(
            name,
            node_type=node_definition['type'],
            node_params=node_definition['parameters'],
            allowAddInput=allow_add_input,
            allowAddOutput=allow_add_output,
            graph=self,
        )
        self.addNode(node, name, pos)
        return node
        
    def addNode(self, node, name, pos=None):
        """Add an existing Node to this flowchart.
        
        See also: createNode()
        """
        if pos is None:
            pos = [0, 0]
        if type(pos) in [QtCore.QPoint, QtCore.QPointF]:
            pos = [pos.x(), pos.y()]
        item = node.graphicsItem()
        item.setZValue(self.nextZVal*2)
        self.nextZVal += 1
        self.viewBox.addItem(item)
        item.moveBy(*pos)
        self._nodes[name] = node
        if node is not self.inputNode and node is not self.outputNode:
            self.widget().addNode(node) 
        node.sigClosed.connect(self.nodeClosed)
        node.sigRenamed.connect(self.nodeRenamed)
        node.sigOutputChanged.connect(self.nodeOutputChanged)
        self.sigChartChanged.emit(self, 'add', node)
        
    def removeNode(self, node):
        """Remove a Node from this flowchart.
        """
        node.close()
        
    def nodeClosed(self, node):
        del self._nodes[node.name()]
        self.widget().removeNode(node)
        for signal in ['sigClosed', 'sigRenamed', 'sigOutputChanged']:
            try:
                getattr(node, signal).disconnect(self.nodeClosed)
            except (TypeError, RuntimeError):
                pass
        self.sigChartChanged.emit(self, 'remove', node)
        
    def nodeRenamed(self, node, oldName):
        del self._nodes[oldName]
        self._nodes[node.name()] = node
        self.widget().nodeRenamed(node, oldName)
        self.sigChartChanged.emit(self, 'rename', node)
        
    def arrangeNodes(self):
        pass
        
    def internalTerminal(self, term):
        """If the terminal belongs to the external Node, return the corresponding internal terminal"""
        if term.node() is self:
            if term.isInput():
                return self.inputNode[term.name()]
            else:
                return self.outputNode[term.name()]
        else:
            return term
        
    def connectTerminals(self, term1, term2):
        """Connect two terminals together within this flowchart."""
        term1 = self.internalTerminal(term1)
        term2 = self.internalTerminal(term2)
        term1.connectTo(term2)
        
    def process(self, **args):
        """
        Process data through the flowchart, returning the output.
        
        Keyword arguments must be the names of input terminals. 
        The return value is a dict with one key per output terminal.
        
        """
        data = {}  ## Stores terminal:value pairs
        
        ## determine order of operations
        ## order should look like [('p', node1), ('p', node2), ('d', terminal1), ...] 
        ## Each tuple specifies either (p)rocess this node or (d)elete the result from this terminal
        order = self.processOrder()
        
        ## Record inputs given to process()
        for n, t in self.inputNode.outputs().items():
            # if n not in args:
            #     raise Exception("Parameter %s required to process this chart." % n)
            if n in args:
                data[t] = args[n]
        
        ret = {}
            
        ## process all in order
        for c, arg in order:
            
            if c == 'p':     ## Process a single node
                node = arg
                if node is self.inputNode:
                    continue  ## input node has already been processed.

                ## get input and output terminals for this node
                outs = list(node.outputs().values())
                ins = list(node.inputs().values())
                
                # construct input value dictionary
                args = {}
                for inp in ins:
                    inputs = inp.inputTerminals()
                    if len(inputs) == 0:
                        continue
                    if inp.isMultiValue():  # multi-input terminals require a dict of all inputs
                        args[inp.name()] = dict([(i, data[i]) for i in inputs if i in data])
                    else:                   # single-inputs terminals only need the single input value available
                        args[inp.name()] = data[inputs[0]]  
                        
                if node is self.outputNode:
                    ret = args  # we now have the return value, but must keep processing in case there are other endpoint nodes in the chart
                else:
                    try:
                        if node.isBypassed():
                            result = node.processBypassed(args)
                        else:
                            result = node.process(display=False, **args)
                    except:
                        print("Error processing node %s. Args are: %s" % (str(node), str(args)))
                        raise
                    for out in outs:
                        try:
                            data[out] = result[out.name()]
                        except KeyError:
                            pass
            elif c == 'd':   ## delete a terminal result (no longer needed; may be holding a lot of memory)
                if arg in data:
                    del data[arg]

        return ret
        
    def processOrder(self):
        """Return the order of operations required to process this chart.
        The order returned should look like [('p', node1), ('p', node2), ('d', terminal1), ...] 
        where each tuple specifies either (p)rocess this node or (d)elete the result from this terminal
        """
        
        ## first collect list of nodes/terminals and their dependencies
        deps = {}
        tdeps = {}   ## {terminal: [nodes that depend on terminal]}
        for name, node in self._nodes.items():
            deps[node] = node.dependentNodes()
            for t in node.outputs().values():
                tdeps[t] = t.dependentNodes()

        # determine correct node-processing order
        order = fn.toposort(deps)
        
        # construct list of operations
        ops = [('p', n) for n in order]
        
        # determine when it is safe to delete terminal values
        dels = []
        for t, nodes in tdeps.items():
            lastInd = 0
            lastNode = None
            for n in nodes:  ## determine which node is the last to be processed according to order
                if n is self:
                    lastInd = None
                    break
                else:
                    try:
                        ind = order.index(n)
                    except ValueError:
                        continue
                if lastNode is None or ind > lastInd:
                    lastNode = n
                    lastInd = ind
            if lastInd is not None:
                dels.append((lastInd+1, t))
        dels.sort(key=lambda a: a[0], reverse=True)
        for i, t in dels:
            ops.insert(i, ('d', t))
        return ops
        
        
    def nodeOutputChanged(self, startNode):
        """Triggered when a node's output values have changed. (NOT called during process())
        Propagates new data forward through network."""
        # first collect list of nodes/terminals and their dependencies
        
        if self.processing:
            return
        self.processing = True
        try:
            deps = {}
            for name, node in self._nodes.items():
                deps[node] = []
                for t in node.outputs().values():
                    deps[node].extend(t.dependentNodes())
            
            ## determine order of updates 
            order = fn.toposort(deps, nodes=[startNode])
            order.reverse()
            
            ## keep track of terminals that have been updated
            terms = set(startNode.outputs().values())

            for node in order[1:]:
                update = False
                for term in list(node.inputs().values()):
                    deps = list(term.connections().keys())
                    for d in deps:
                        if d in terms:
                            update |= True
                            term.inputChanged(d, process=False)
                if update:
                    node.update()
                    terms |= set(node.outputs().values())
                    
        finally:
            self.processing = False
            if self.inputWasSet:
                self.inputWasSet = False
            else:
                self.sigStateChanged.emit()

    def chartGraphicsItem(self):
        """Return the graphicsItem that displays the internal nodes and
        connections of this flowchart.
        
        Note that the similar method `graphicsItem()` is inherited from Node
        and returns the *external* graphical representation of this flowchart."""
        return self.viewBox
        
    def widget(self):
        """Return the control widget for this flowchart.
        
        This widget provides GUI access to the parameters for each node and a
        graphical representation of the flowchart.
        """
        if self._widget is None:
            self._widget = EagerxGraphCtrlWidget(self)
            self.scene = self._widget.scene()
            self.viewBox = self._widget.viewBox()
        return self._widget

    def listConnections(self):
        conn = set()
        for n in self._nodes.values():
            terms = n.outputs()
            for n, t in terms.items():
                for c in t.connections():
                    conn.add((t, c))
        return conn

    def saveState(self):
        """Return a serializable data structure representing the current state of this flowchart. 
        """
        state = Node.saveState(self)
        state['nodes'] = []
        state['connects'] = []
        
        for name, node in self._nodes.items():
            cls = type(node)
            if hasattr(cls, 'nodeName'):
                clsName = cls.nodeName
                pos = node.graphicsItem().pos()
                ns = {'class': clsName, 'name': name, 'pos': (pos.x(), pos.y()), 'state': node.saveState()}
                state['nodes'].append(ns)
            
        conn = self.listConnections()
        for a, b in conn:
            state['connects'].append((a.node().name(), a.name(), b.node().name(), b.name()))
        
        state['inputNode'] = self.inputNode.saveState()
        state['outputNode'] = self.outputNode.saveState()
        
        return state
        
    def restoreState(self, state, clear=False):
        """Restore the state of this flowchart from a previous call to `saveState()`.
        """
        self.blockSignals(True)
        try:
            if clear:
                self.clear()
            Node.restoreState(self, state)
            nodes = state['nodes']
            nodes.sort(key=lambda a: a['pos'][0])
            for n in nodes:
                if n['name'] in self._nodes:
                    self._nodes[n['name']].restoreState(n['state'])
                    continue
                try:
                    node = self.createNode(n['class'], name=n['name'])
                    node.restoreState(n['state'])
                except:
                    printExc("Error creating node %s: (continuing anyway)" % n['name'])
                
            self.inputNode.restoreState(state.get('inputNode', {}))
            self.outputNode.restoreState(state.get('outputNode', {}))

            for n1, t1, n2, t2 in state['connects']:
                try:
                    self.connectTerminals(self._nodes[n1][t1], self._nodes[n2][t2])
                except:
                    print(self._nodes[n1].terminals)
                    print(self._nodes[n2].terminals)
                    printExc("Error connecting terminals %s.%s - %s.%s:" % (n1, t1, n2, t2))
                
        finally:
            self.blockSignals(False)
            
        self.outputChanged()
        self.sigChartLoaded.emit()
        self.sigStateChanged.emit()
            
    def loadFile(self, fileName=None, startDir=None):
        """Load a flowchart (``*.graph``) file.
        """
        if fileName is None:
            if startDir is None:
                startDir = self.filePath
            if startDir is None:
                startDir = '.'
            self.fileDialog = FileDialog(None, "Load Graph..", startDir, "Graph (*.graph)")
            self.fileDialog.show()
            self.fileDialog.fileSelected.connect(self.loadFile)
            return
        fileName = asUnicode(fileName)
        state = configfile.readConfigFile(fileName)
        self.restoreState(state, clear=True)
        self.viewBox.autoRange()
        self.sigFileLoaded.emit(fileName)

    def saveFile(self, fileName=None, startDir=None, suggestedFileName='graph.graph'):
        """Save this flowchart to a .graph file
        """
        if fileName is None:
            if startDir is None:
                startDir = self.filePath
            if startDir is None:
                startDir = '.'
            self.fileDialog = FileDialog(None, "Save Graph..", startDir, "Graph (*.graph)")
            self.fileDialog.setDefaultSuffix("graph")
            self.fileDialog.setAcceptMode(QtGui.QFileDialog.AcceptSave) 
            self.fileDialog.show()
            self.fileDialog.fileSelected.connect(self.saveFile)
            return
        fileName = asUnicode(fileName)
        configfile.writeConfigFile(self.saveState(), fileName)
        self.sigFileSaved.emit(fileName)

    def clear(self):
        """Remove all nodes from this flowchart except the original input/output nodes.
        """
        for n in list(self._nodes.values()):
            if n is self.inputNode or n is self.outputNode:
                continue
            n.close()  ## calls self.nodeClosed(n) by signal
        self.widget().clear()
        
    def clearTerminals(self):
        Node.clearTerminals(self)
        self.inputNode.clearTerminals()
        self.outputNode.clearTerminals()


class EagerxGraphicsItem(GraphicsObject):
    
    def __init__(self, chart):
        GraphicsObject.__init__(self)
        self.chart = chart ## chart is an instance of Flowchart()
        self.updateTerminals()
        
    def updateTerminals(self):
        self.terminals = {}
        bounds = self.boundingRect()
        inp = self.chart.inputs()
        dy = bounds.height() / (len(inp)+1)
        y = dy
        for n, t in inp.items():
            item = t.graphicsItem()
            self.terminals[n] = item
            item.setParentItem(self)
            item.setAnchor(bounds.width(), y)
            y += dy
        out = self.chart.outputs()
        dy = bounds.height() / (len(out)+1)
        y = dy
        for n, t in out.items():
            item = t.graphicsItem()
            self.terminals[n] = item
            item.setParentItem(self)
            item.setAnchor(0, y)
            y += dy
        
    def boundingRect(self):
        return QtCore.QRectF()
        
    def paint(self, p, *args):
        pass
    

class EagerxGraphCtrlWidget(QtGui.QWidget):
    """The widget that contains the list of all the nodes in a flowchart and their controls, as well as buttons for loading/saving flowcharts."""
    
    def __init__(self, chart):
        self.items = {}
        self.currentFileName = None
        QtGui.QWidget.__init__(self)
        self.chart = chart
        self.ui = EagerxGraphCtrlTemplate.Ui_Form()
        self.ui.setupUi(self)
        self.ui.ctrlList.setColumnCount(2)
        self.ui.ctrlList.setColumnWidth(1, 20)
        self.ui.ctrlList.setVerticalScrollMode(self.ui.ctrlList.ScrollPerPixel)
        self.ui.ctrlList.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        
        self.chartWidget = EagerxGraphWidget(chart, self)
        self.cwWin = QtGui.QMainWindow()
        self.cwWin.setWindowTitle('EAGERx Graph')
        self.cwWin.setCentralWidget(self.chartWidget)
        self.cwWin.resize(1000, 800)
        
        h = self.ui.ctrlList.header()
        if QT_LIB in ['PyQt4', 'PySide']:
            h.setResizeMode(0, h.Stretch)
        else:
            h.setSectionResizeMode(0, h.Stretch)
        
        self.ui.ctrlList.itemChanged.connect(self.itemChanged)
        self.ui.loadBtn.clicked.connect(self.loadClicked)
        self.ui.saveBtn.clicked.connect(self.saveClicked)
        self.ui.saveAsBtn.clicked.connect(self.saveAsClicked)
        self.ui.showChartBtn.toggled.connect(self.chartToggled)
        self.chart.sigFileLoaded.connect(self.setCurrentFile)
        self.ui.reloadBtn.clicked.connect(self.reloadClicked)
        self.chart.sigFileSaved.connect(self.fileSaved)

    def chartToggled(self, b):
        if b:
            self.cwWin.show()
        else:
            self.cwWin.hide()

    def reloadClicked(self):
        try:
            self.chartWidget.reloadLibrary()
            self.ui.reloadBtn.success("Reloaded.")
        except:
            self.ui.reloadBtn.success("Error.")
            raise
            
            
    def loadClicked(self):
        newFile = self.chart.loadFile()
        
    def fileSaved(self, fileName):
        self.setCurrentFile(asUnicode(fileName))
        self.ui.saveBtn.success("Saved.")
        
    def saveClicked(self):
        if self.currentFileName is None:
            self.saveAsClicked()
        else:
            try:
                self.chart.saveFile(self.currentFileName)
            except:
                self.ui.saveBtn.failure("Error")
                raise
        
    def saveAsClicked(self):
        try:
            if self.currentFileName is None:
                newFile = self.chart.saveFile()
            else:
                newFile = self.chart.saveFile(suggestedFileName=self.currentFileName)
        except:
            self.ui.saveBtn.failure("Error")
            raise
            
        #self.setCurrentFile(newFile)
            
    def setCurrentFile(self, fileName):
        self.currentFileName = asUnicode(fileName)
        if fileName is None:
            self.ui.fileNameLabel.setText("<b>[ new ]</b>")
        else:
            self.ui.fileNameLabel.setText("<b>%s</b>" % os.path.split(self.currentFileName)[1])
        self.resizeEvent(None)

    def itemChanged(self, *args):
        pass
    
    def scene(self):
        return self.chartWidget.scene() ## returns the GraphicsScene object
    
    def viewBox(self):
        return self.chartWidget.viewBox()

    def nodeRenamed(self, node, oldName):
        self.items[node].setText(0, node.name())

    def addNode(self, node):
        ctrl = node.ctrlWidget()
        item = QtGui.QTreeWidgetItem([node.name(), '', ''])
        self.ui.ctrlList.addTopLevelItem(item)
        byp = QtGui.QPushButton('X')
        byp.setCheckable(True)
        byp.setFixedWidth(20)
        item.bypassBtn = byp
        self.ui.ctrlList.setItemWidget(item, 1, byp)
        byp.node = node
        node.bypassButton = byp
        byp.setChecked(node.isBypassed())
        byp.clicked.connect(self.bypassClicked)
        
        if ctrl is not None:
            item2 = QtGui.QTreeWidgetItem()
            item.addChild(item2)
            self.ui.ctrlList.setItemWidget(item2, 0, ctrl)
            
        self.items[node] = item
        
    def removeNode(self, node):
        if node in self.items:
            item = self.items[node]
            try:
                item.bypassBtn.clicked.disconnect(self.bypassClicked)
            except (TypeError, RuntimeError):
                pass
            self.ui.ctrlList.removeTopLevelItem(item)
            
    def bypassClicked(self):
        btn = QtCore.QObject.sender(self)
        btn.node.bypass(btn.isChecked())
            
    def chartWidget(self):
        return self.chartWidget

    def outputChanged(self, data):
        pass

    def clear(self):
        self.chartWidget.clear()
        
    def select(self, node):
        item = self.items[node]
        self.ui.ctrlList.setCurrentItem(item)

    def clearSelection(self):
        self.ui.ctrlList.selectionModel().clearSelection()


class EagerxGraphWidget(dockarea.DockArea):
    """Includes the actual graphical flowchart and debugging interface"""
    def __init__(self, chart, ctrl):
        dockarea.DockArea.__init__(self)
        self.chart = chart
        self.ctrl = ctrl
        self.hoverItem = None
        
        # build user interface (it was easier to do it here than via developer)
        self.view = eagerx_graph_view.EagerxGraphView(self)
        self.viewDock = dockarea.Dock('view', size=(1000, 600))
        self.viewDock.addWidget(self.view)
        self.viewDock.hideTitleBar()
        self.addDock(self.viewDock)

        self.hoverText = QtGui.QTextEdit()
        self.hoverText.setReadOnly(True)
        self.hoverDock = dockarea.Dock('Hover Info', size=(1000, 20))
        self.hoverDock.addWidget(self.hoverText)
        self.addDock(self.hoverDock, 'bottom')

        self.selInfo = QtGui.QWidget()
        self.selInfoLayout = QtGui.QGridLayout()
        self.selInfo.setLayout(self.selInfoLayout)
        self.selDescLabel = QtGui.QLabel()
        self.selNameLabel = QtGui.QLabel()
        self.selDescLabel.setWordWrap(True)
        self.selectedTree = DataTreeWidget()
        self.selInfoLayout.addWidget(self.selDescLabel)
        self.selInfoLayout.addWidget(self.selectedTree)
        
        self._scene = self.view.scene()
        self._viewBox = self.view.viewBox()
        
        self.buildMenu()
            
        self._scene.selectionChanged.connect(self.selectionChanged)
        self._scene.sigMouseHover.connect(self.hoverOver)

    def reloadLibrary(self):
        self.nodeMenu.triggered.disconnect(self.nodeMenuTriggered)
        self.nodeMenu = None
        self.chart.library.reload()
        self.buildMenu()
        
    def buildMenu(self, pos=None):
        def buildSubMenu(library, root_menu, subMenus, node_type, pos=None):
            for package, nodes in library.items():
                package_menu = QtGui.QMenu(package)
                root_menu.addMenu(package_menu)
                subMenus.append(package_menu)
                for node in nodes:
                    act = package_menu.addAction(node['name'])
                    node['package'] = package
                    node['type'] = node_type
                    act.nodeType = node
                    act.pos = pos
        self.subMenus = []
        self.nodeMenu = []
        for node_type, library in self.chart.library.items():
            menu = QtGui.QMenu('Add {}'.format(node_type))
            buildSubMenu(library, menu, self.subMenus, node_type, pos=pos)
            menu.triggered.connect(self.nodeMenuTriggered)
            self.nodeMenu.append(menu)
        return self.nodeMenu
    
    def menuPosChanged(self, pos):
        self.menuPos = pos
    
    def showViewMenu(self, ev):
        self.buildMenu(ev.scenePos())
        self.nodeMenu.popup(ev.screenPos())
        
    def scene(self):
        return self._scene ## the GraphicsScene item

    def viewBox(self):
        return self._viewBox ## the viewBox that items should be added to

    def nodeMenuTriggered(self, action):
        nodeType = action.nodeType
        if action.pos is not None:
            pos = action.pos
        else:
            pos = self.menuPos
        pos = self.viewBox().mapSceneToView(pos)

        self.chart.createNode(nodeType, pos=pos)


    def selectionChanged(self):
        items = self._scene.selectedItems()
        if len(items) == 0:
            data = None
        else:
            item = items[0]
            if hasattr(item, 'node') and isinstance(item.node, Node):
                n = item.node
                if n in self.ctrl.items:
                    self.ctrl.select(n)
                else:
                    self.ctrl.clearSelection()
                data = {'outputs': n.outputValues(), 'inputs': n.inputValues()}
                self.selNameLabel.setText(n.name())
                if hasattr(n, 'nodeName'):
                    self.selDescLabel.setText("<b>%s</b>: %s" % (n.nodeName, n.__class__.__doc__))
                else:
                    self.selDescLabel.setText("")
                if n.exception is not None:
                    data['exception'] = n.exception
            else:
                data = None
        self.selectedTree.setData(data, hideRoot=True)

    def hoverOver(self, items):
        for item in items:
            self.hoverItem = item
            if hasattr(item, 'term') and isinstance(item.term, EagerxTerminal):
                self.hoverText.setPlainText(item.term.name())
                return
            elif hasattr(item, 'node') and isinstance(item.node, EagerxNode):
                text = item.node.name()
                for key, value in item.node.info().items():
                    text += '\n' + '{}: {}'.format(key, value)
                self.hoverText.setPlainText(text)
                return
        self.hoverText.setPlainText("")


    def clear(self):
        self.selectedTree.setData(None)
        self.hoverText.setPlainText('')
        self.selNameLabel.setText('')
        self.selDescLabel.setText('')
        
        
class EagerxGraphNode(Node):
    pass
