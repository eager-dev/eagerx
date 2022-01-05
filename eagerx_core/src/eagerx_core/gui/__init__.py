import sys
from pyqtgraph.Qt import QtGui, QtCore
from eagerx_core.gui.rxgui import RxGui


def launch_gui(state):
    app = QtGui.QApplication([])

    # Create main window with grid layout
    win = QtGui.QMainWindow()
    win.setWindowTitle('EAGERx Graph Creator')
    cw = QtGui.QWidget()
    win.setCentralWidget(cw)
    layout = QtGui.QGridLayout()
    cw.setLayout(layout)

    rx_gui = RxGui(state)
    w = rx_gui.widget()

    # Add flowchart control panel to the main window
    layout.addWidget(w, 0, 0, 2, 1)
    win.show()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

    new_state = rx_gui._state

    return new_state

# TODO: Check if flowchart can run in notebook
# TODO: Add render node
# TODO: Draw graph based on list of nodes and objects
# TODO: Nice to have: show rate in outputs
# TODO: Nice to have: allow to add terminals in nodes
# TODO: Nice to have: Input terminals get address on connection
# TODO: Nice to have: Generate rate based on connection (not changeable)
# TODO: Nice to have: Show addresses and update on changes in connection (not changeable)
