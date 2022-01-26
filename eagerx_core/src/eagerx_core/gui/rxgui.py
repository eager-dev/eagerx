"""
 Mostly copy paste from https://github.com/pyqtgraph/pyqtgraph/blob/master/pyqtgraph/flowchart/Flowchart.py
"""

# -*- coding: utf-8 -*-
import os
import numpy as np
import importlib
from copy import deepcopy

# Import pyqtgraph modules
from pyqtgraph.graphicsItems.GraphicsObject import GraphicsObject
from pyqtgraph.Qt import QtCore, QtGui, QT_LIB
from pyqtgraph import FileDialog, DataTreeWidget
from pyqtgraph import dockarea
from pyqtgraph.debug import printExc

# Import eagerx modules
from eagerx_core import constants
from eagerx_core.rxgraph import RxGraph
from eagerx_core.gui import rxgui_view
from eagerx_core.gui.rxgui_node import RxGuiNode, NodeGraphicsItem
from eagerx_core.gui.rxgui_terminal import TerminalGraphicsItem, ConnectionItem
from eagerx_core.params import RxObjectParams, RxNodeParams
from eagerx_core.utils.utils import get_nodes_and_objects_library
from eagerx_core.utils.pyqtgraph_utils import exception_handler


# pyside and pyqt use incompatible ui files.
rx_ui_template = importlib.import_module('eagerx_core.gui.templates.ui_{}'.format(QT_LIB.lower()))


class RxGui(RxGraph, QtCore.QObject):
    library = get_nodes_and_objects_library()
    sigFileLoaded = QtCore.Signal(object)
    sigFileSaved = QtCore.Signal(object)
    sigChartLoaded = QtCore.Signal()
    sigStateChanged = QtCore.Signal()  # called when output is expected to have changed
    sigChartChanged = QtCore.Signal(object, object, object)  # called when nodes are added, removed, or renamed.

    def __init__(self, state):
        RxGraph.__init__(self, state=state)
        QtCore.QObject.__init__(self)
        self.nodes = {}
        self.next_z_val = 10
        self._widget = None
        self.scene = None
        self.file_path = None
        self.widget()
        self.load_state(clear=True)
        self.viewBox.autoRange(padding=0.04)

    @staticmethod
    def add_pos_to_state(state):
        for n in state['nodes'].values():
            if 'pos' not in n:
                if n['params']['default']['name'] in ['env/observations', 'env/actions', 'env/render']:
                    if n['params']['default']['name'] == 'env/actions':
                        pos = [0, 0]
                    elif n['params']['default']['name'] == 'env/observations':
                        pos = [600, 0]
                    elif n['params']['default']['name'] == 'env/render':
                        pos = [600, 150]
                    else:
                        pos = np.array([150, 0]) + np.random.randint((300, 150))
                else:
                    pos = np.array([150, 0]) + np.random.randint((300, 150))
                n['pos'] = pos
        return state

    def create_node(self, name, pos):
        """Create a new Node and add it to this flowchart.
        """
        node = RxGuiNode(name, graph=self)
        self.add_node(node, name, pos)
        return node

    def add_node(self, node, name, pos=None):
        """Add an existing Node to this flowchart.
        
        See also: createnode
        """
        if pos is None:
            pos = [0, 0]
        if type(pos) in [QtCore.QPoint, QtCore.QPointF]:
            pos = [pos.x(), pos.y()]
        item = node.graphics_item()
        item.setZValue(self.next_z_val * 2)
        self.next_z_val += 1
        self.viewBox.addItem(item)
        item.moveBy(*pos)
        self.nodes[name] = node
        self.widget().add_node(node)
        node.sigClosed.connect(self.node_closed)
        node.sigRenamed.connect(self.node_renamed)
        self.sigChartChanged.emit(self, 'add', node)

    def remove_node(self, node):
        """Remove a Node from this flowchart.
        """
        node.close()

    def node_closed(self, node):
        del self.nodes[node.name]
        self.widget().remove_node(node)
        for signal in ['sigClosed', 'sigRenamed']:
            try:
                getattr(node, signal).disconnect(self.node_closed)
            except (TypeError, RuntimeError):
                pass
        self.sigChartChanged.emit(self, 'remove', node)

    def node_renamed(self, node, old_name):
        del self.nodes[old_name]
        self.nodes[node.name] = node
        self.widget().node_renamed(node, old_name)
        self.sigChartChanged.emit(self, 'rename', node)

    def connect_terminals(self, term1, term2):
        """Connect two terminals together within this flowchart."""
        connection_item = ConnectionItem(term1.graphics_item(), term2.graphics_item())
        term1.graphics_item().getViewBox().addItem(connection_item)

        term1.connections[term2] = connection_item
        term2.connections[term1] = connection_item

        term1.connected(term2)
        term2.connected(term1)

    def chart_graphics_item(self):
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
            self._widget = RxCtrlWidget(self)
            self.scene = self._widget.scene
            self.viewBox = self._widget.viewBox()
        return self._widget

    def load_state(self, clear=False):
        self.blockSignals(True)
        try:
            if clear:
                self.clear()
            if 'env/render' not in self._state['nodes']:
                render = RxNodeParams.create('env/render', 'eagerx_core', 'render', rate=1.)
                self.add(render)
            self.add_pos_to_state(self._state)
            nodes = self._state['nodes']
            nodes = [dict(**node, name=name) for name, node in nodes.items()]
            nodes.sort(key=lambda a: a['pos'][0])
            for n in nodes:
                if n['name'] in self.nodes:
                    self.nodes[n['name']].load_state(n)
                    continue
                try:
                    name = n['name']
                    pos = n['pos']
                    node = self.create_node(name, pos)
                    node.load_state(n)
                except Exception:
                    printExc("Error creating node %s: (continuing anyway)" % n['name'])

            connects = [(connection[0][0], '/'.join(connection[0][1:3]),
                         connection[1][0], '/'.join(connection[1][1:3])) for connection in self._state['connects']]
            for n1, t1, n2, t2 in connects:
                try:
                    self.connect_terminals(self.nodes[n1].terminals[t1], self.nodes[n2].terminals[t2])
                except Exception:
                    print(self.nodes[n1].terminals)
                    print(self.nodes[n2].terminals)
                    printExc("Error connecting terminals %s.%s - %s.%s:" % (n1, t1, n2, t2))

        finally:
            self.blockSignals(False)

        self.sigChartLoaded.emit()
        self.sigStateChanged.emit()

    def state(self):
        state = deepcopy(self._state)
        target = ['env/render', 'inputs', 'image']
        render_connected = False
        for idx, c in enumerate(self._state['connects']):
            if target == c[1]:
                render_connected = True
                break
        if not render_connected:
            state['nodes'].pop('env/render')
        return state

    def load_file(self, file_name=None, start_dir=None):
        """Load a flowchart (``*.graph``) file.
        """
        if file_name is None:
            if start_dir is None:
                start_dir = self.file_path
            if start_dir is None:
                start_dir = '.'
            self.fileDialog = FileDialog(None, "Load Graph..", start_dir, "Graph (*.graph)")
            self.fileDialog.show()
            self.fileDialog.fileSelected.connect(self.load_file)
            return
        file_name = file_name
        self.load(file_name)
        self.load_state(clear=True)
        self.viewBox.autoRange()
        self.sigFileLoaded.emit(file_name)

    def save_file(self, file_name=None, start_dir=None):
        """Save this flowchart to a .graph file
        """
        if file_name is None:
            if start_dir is None:
                start_dir = self.file_path
            if start_dir is None:
                start_dir = '.'
            self.fileDialog = FileDialog(None, "Save Graph..", start_dir, "Graph (*.graph)")
            self.fileDialog.setDefaultSuffix("graph")
            self.fileDialog.setAcceptMode(QtGui.QFileDialog.AcceptSave)
            self.fileDialog.show()
            self.fileDialog.fileSelected.connect(self.save_file)
            return
        file_name = file_name
        self._state = self.state()
        self.save(file_name)
        self.sigFileSaved.emit(file_name)
        self.load_state(clear=True)

    def clear(self):
        """Remove all nodes from this flowchart except the original input/output nodes.
        """
        for n in list(self.nodes.values()):
            n.close()  # calls self.nodeClosed(n) by signal
        self.widget().clear()


class RxGraphicsItem(GraphicsObject):

    def __init__(self, chart):
        GraphicsObject.__init__(self)
        self.chart = chart  # chart is an instance of Flowchart()
        self.update_terminals()

    def update_terminals(self):
        self.terminals = {}

    def boundingRect(self):
        return QtCore.QRectF()

    def paint(self, p, *args):
        pass


class RxCtrlWidget(QtGui.QWidget):
    """The widget that contains the list of all the nodes in a flowchart and their controls, as well as buttons for
    loading/saving flowcharts."""

    def __init__(self, chart):
        self.items = {}
        self.current_file_name = None
        QtGui.QWidget.__init__(self)
        self.chart = chart
        self.ui = rx_ui_template.Ui_Form()
        self.ui.setupUi(self)
        self.ui.ctrlList.setColumnCount(2)
        self.ui.ctrlList.setColumnWidth(1, 20)
        self.ui.ctrlList.setVerticalScrollMode(self.ui.ctrlList.ScrollPerPixel)
        self.ui.ctrlList.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

        self.chartWidget = EagerxGraphWidget(chart, self)
        self.scene = self.chartWidget.scene
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
        self.ui.loadBtn.clicked.connect(self.load_clicked)
        self.ui.saveBtn.clicked.connect(self.save_clicked)
        self.ui.saveAsBtn.clicked.connect(self.save_as_clicked)
        self.ui.showChartBtn.toggled.connect(self.chart_toggled)
        self.ui.checkValidityBtn.clicked.connect(self.check_validity_toggled)
        self.ui.showCompatibleBridgesBtn.clicked.connect(self.show_compatible_bridges_toggled)

        self.chart.sigFileLoaded.connect(self.set_current_file)
        self.chart.sigFileSaved.connect(self.file_saved)

    def chart_toggled(self, b):
        if b:
            self.cwWin.show()
        else:
            self.cwWin.hide()

    def load_clicked(self):
        new_file = self.chart.load_file()

    def file_saved(self, file_name):
        self.set_current_file(file_name)
        self.ui.saveBtn.success("Saved.")

    def save_clicked(self):
        if self.current_file_name is None:
            self.save_as_clicked()
        else:
            try:
                self.chart.save_file(self.current_file_name)
            except:
                self.ui.saveBtn.failure("Error")
                raise

    def save_as_clicked(self):
        try:
            if self.current_file_name is None:
                new_file = self.chart.save_file()
            else:
                new_file = self.chart.save_file(suggested_file_name=self.current_file_name)
        except:
            self.ui.saveBtn.failure("Error")
            raise

    def check_validity_toggled(self):
        try:
            self._check_validity(graph_backup=self.chart, dialog_title='Invalid Graph')
            self.ui.checkValidityBtn.success("Valid")
        except Exception:
            self.ui.checkValidityBtn.failure("Invalid")

    @exception_handler
    def _check_validity(self):
        self.chart._is_valid(state=self.chart.state())

    def show_compatible_bridges_toggled(self):
        try:
            label_string = self.chart.check_exists_compatible_bridge(self.chart.state(), tablefmt='html')
        except Exception as e:
            label_string = str(e)
        bridges_window = QtGui.QDialog(self.chart.widget().cwWin)
        bridges_window.setWindowTitle('Compatible Bridges')
        layout = QtGui.QGridLayout()
        label = QtGui.QLabel(label_string)
        label.setWordWrap(True)
        layout.addWidget(label)
        bridges_window.setLayout(layout)
        bridges_window.exec_()

    def set_current_file(self, file_name):
        self.current_file_name = file_name
        if file_name is None:
            self.ui.fileNameLabel.setText("<b>[ new ]</b>")
        else:
            self.ui.fileNameLabel.setText("<b>%s</b>" % os.path.split(self.current_file_name)[1])
        self.resizeEvent(None)

    def itemChanged(self, *args):
        pass

    def viewBox(self):
        return self.chartWidget.viewBox()

    def node_renamed(self, node, old_name):
        self.items[node].setText(0, node.name)

    def add_node(self, node):
        ctrl = node.ctrl_widget()
        item = QtGui.QTreeWidgetItem([node.name, '', ''])
        self.ui.ctrlList.addTopLevelItem(item)

        if ctrl is not None:
            item2 = QtGui.QTreeWidgetItem()
            item.addChild(item2)
            self.ui.ctrlList.setItemWidget(item2, 0, ctrl)

        self.items[node] = item

    def remove_node(self, node):
        if node in self.items:
            item = self.items[node]
            self.ui.ctrlList.removeTopLevelItem(item)

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
        self.view = rxgui_view.RxView(self)
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

        self.scene = self.view.scene()
        self._viewBox = self.view.viewBox()

        self.buildMenu()

        self.scene.selectionChanged.connect(self.selection_changed)
        self.scene.sigMouseHover.connect(self.hover_over)

    def buildMenu(self, pos=None):
        def build_sub_menu(library, root_menu, submenus, pos=None):
            for node in library:
                id = node['id']
                if id in constants.GUI_NODE_IDS_TO_IGNORE: continue
                act = root_menu.addAction(id)
                act.nodeType = node
                act.pos = pos

        self.submenus = []
        self.nodeMenu = []
        for node_type, library in self.chart.library.items():
            if node_type in constants.GUI_ENTITIES_TO_IGNORE: continue
            menu = QtGui.QMenu('Add {}'.format(node_type.replace('_', ' ')))
            build_sub_menu(library, menu, self.submenus, pos=pos)
            menu.triggered.connect(self.node_menu_triggered)
            self.nodeMenu.append(menu)
        return self.nodeMenu

    def menuPosChanged(self, pos):
        self.menuPos = pos

    def showViewMenu(self, ev):
        self.buildMenu(ev.scenePos())
        self.nodeMenu.popup(ev.screenPos())

    def viewBox(self):
        return self._viewBox  # the viewBox that items should be added to

    def node_menu_triggered(self, action):
        node_type = action.nodeType
        if action.pos is not None:
            pos = action.pos
        else:
            pos = self.menuPos
        pos = self.viewBox().mapSceneToView(pos)

        if type(pos) in [QtCore.QPoint, QtCore.QPointF]:
            pos = [pos.x(), pos.y()]

        n = 0
        while True:
            name = '{}'.format(node_type['id'])
            if n > 0:
                name += '_{}'.format(n + 1)
            if name not in self.chart.nodes:
                break
            n += 1

        from eagerx_core.registration import get_spec
        signature = node_type['entity_cls'].get_spec(node_type['id'])
        args = signature.parameters.keys()
        mapping = dict()
        if 'name' in args:
            mapping['name'] = name
        if 'rate' in args:
            mapping['rate'] = 1.0
        rx_entity = node_type['spec'](**mapping)
        self.chart.add(rx_entity)
        self.chart._state['nodes'][name]['pos'] = pos
        self.chart.create_node(name, pos)

    def selection_changed(self):
        items = self.scene.selectedItems()
        if len(items) == 0:
            data = None
        else:
            item = items[0]
            if hasattr(item, 'node') and isinstance(item.node, RxGuiNode):
                n = item.node
                if n in self.ctrl.items:
                    self.ctrl.select(n)
                else:
                    self.ctrl.clearSelection()
                data = {}
                self.selNameLabel.setText(n.name)
                if hasattr(n, 'name'):
                    self.selDescLabel.setText("<b>%s</b>: %s" % (n.name, n.__class__.__doc__))
                else:
                    self.selDescLabel.setText("")
                if n.exception is not None:
                    data['exception'] = n.exception
            else:
                data = None
        self.selectedTree.setData(data, hideRoot=True)

    def hover_over(self, items):
        for item in items:
            self.hoverItem = item
            if hasattr(item, 'term') and isinstance(item, TerminalGraphicsItem):
                text = 'name: ' + item.term.terminal_name
                for key, value in item.term.params().items():
                    if key in constants.GUI_WIDGETS['term']['hide']['all']:
                        continue
                    elif item.term.node_type in constants.GUI_WIDGETS['term']['hide'] and \
                            key in constants.GUI_WIDGETS['term']['hide'][item.term.node_type]:
                        continue
                    elif item.term.terminal_type in constants.GUI_WIDGETS['term']['hide'] and \
                            key in constants.GUI_WIDGETS['term']['hide'][item.term.terminal_type]:
                        continue
                    text += '\n' + '{}: {}'.format(key, value)
                self.hoverText.setPlainText(text)
                return
            elif hasattr(item, 'node') and isinstance(item, NodeGraphicsItem):
                text = ''
                for key, value in item.node.params().items():
                    if key in constants.GUI_WIDGETS['node']['hide']['all']:
                        continue
                    elif item.node.node_type in constants.GUI_WIDGETS['node']['hide'] and \
                            key in constants.GUI_WIDGETS['node']['hide'][item.node.node_type]:
                        continue
                    text += '{}: {}\n'.format(key, value)
                self.hoverText.setPlainText(text)
                return
        self.hoverText.setPlainText("")

    def clear(self):
        self.selectedTree.setData(None)
        self.hoverText.setPlainText('')
        self.selNameLabel.setText('')
        self.selDescLabel.setText('')
