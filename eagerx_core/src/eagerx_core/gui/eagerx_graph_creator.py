import sys
from pyqtgraph.Qt import QtGui, QtCore
from eagerx_core.gui.eagerx_graph import EagerxGraph


def draw_graph():
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

    return fc.return_graph()


if __name__ == '__main__':
    graph = draw_graph()

# TODO: Check if flowchart can run in notebook
# TODO: Return connections as in main.py
# TODO: Nice to have: show rate in outputs
# TODO: Nice to have: Recolor node on changing color
# TODO: Nice to have: Implement process args:{0: NEW_PROCESS, 1: ENVIRONMENT, 2: BRIDGE, 3: EXTERNAL} (Possibly use eagerx_core.constants.process class)
# TODO: Nice to have: Change loglevel integers to their corresponding strings
# TODO: Nice to have: allow to add terminals in nodes
# TODO: Nice to have: Input terminals get address on connection
# TODO: Nice to have: Generate rate based on connection (not changeable)
# TODO: Nice to have: Show addresses and update on changes in connection (not changeable)
