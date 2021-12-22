import sys
from pyqtgraph.Qt import QtGui, QtCore
from eagerx_core.gui.eagerx_graph import EagerxGraph


def draw_graph(graph=None):
    app = QtGui.QApplication([])

    # Create main window with grid layout
    win = QtGui.QMainWindow()
    win.setWindowTitle('EAGERx Graph Creator')
    cw = QtGui.QWidget()
    win.setCentralWidget(cw)
    layout = QtGui.QGridLayout()
    cw.setLayout(layout)

    fc = EagerxGraph()
    w = fc.widget()

    # Add flowchart control panel to the main window
    layout.addWidget(fc.widget(), 0, 0, 2, 1)
    win.show()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

    state = fc.saveState()