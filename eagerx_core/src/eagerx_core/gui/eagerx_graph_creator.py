import sys
from pyqtgraph.Qt import QtGui, QtCore
from eagerx_core.gui.eagerx_graph import EagerxGraph
from eagerx_core.utils.utils import get_nodes_and_objects_library


def EagerxGraphCreator():
    library = get_nodes_and_objects_library()
    app = QtGui.QApplication([])

    # Create main window with grid layout
    win = QtGui.QMainWindow()
    win.setWindowTitle('EAGERx Graph Creator')
    cw = QtGui.QWidget()
    win.setCentralWidget(cw)
    layout = QtGui.QGridLayout()
    cw.setLayout(layout)

    fc = EagerxGraph(library=library)
    w = fc.widget()

    # Add flowchart control panel to the main window
    layout.addWidget(fc.widget(), 0, 0, 2, 1)
    win.show()

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()


if __name__ == '__main__':
    EagerxGraphCreator()

# TODO: Check if flowchart can run in notebook
# TODO: Allow to add processor with arguments when right click on input or output
# TODO: Open new window on double clicking on nodes/terminals
# TODO: Input terminals get address on connection
# TODO: Show addresses and update on changes in connection (not changeable)
# TODO: Generate rate based on connection (not changeable)
# TODO: Space converters of states are changeable
