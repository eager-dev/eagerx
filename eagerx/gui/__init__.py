import sys
from pyqtgraph.Qt import QtGui, QtCore
from eagerx.gui.gui import Gui


def launch_gui(state):
    app = QtGui.QApplication([])

    # Create main window with grid layout
    win = QtGui.QMainWindow()
    win.setWindowTitle('EAGERx Graph Creator')
    cw = QtGui.QWidget()
    win.setCentralWidget(cw)
    layout = QtGui.QGridLayout()
    cw.setLayout(layout)

    rx_gui = Gui(state)
    w = rx_gui.widget()

    # Add flowchart control panel to the main window
    layout.addWidget(w, 0, 0, 2, 1)
    win.show()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
    new_state = rx_gui.state()

    return new_state
