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
    new_state = rx_gui.state()

    return new_state

# todo: Check converter validity before connecting
# todo: Check if flowchart can run in notebook
# todo: Add render node
# todo: What to do if text is invalid yaml?
# todo: rate should not be none (now happens when connecting an observation)
# todo: Nice to have: show rate in outputs
# todo: Nice to have: allow to add terminals in nodes
# todo: Nice to have: Input terminals get address on connection
# todo: Nice to have: Generate rate based on connection (not changeable)
# todo: Nice to have: Show addresses and update on changes in connection (not changeable)
