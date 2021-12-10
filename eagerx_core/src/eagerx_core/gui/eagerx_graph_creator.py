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
# TODO: Allow to add converter
# TODO: Input terminals get address on connection
# TODO: Show addresses and update on changes in connection (not changeable)
# TODO: Generate rate based on connection (not changeable)
# TODO: Remove connection if converter type has changed
# TODO: Remove converter actions and observations on removing connection
# TODO: Rate is float (possibly show rate in outputs)
# TODO: Recolor node on changing color
# TODO: Inputs are now strings, convert them to native
# TODO: Implement process args:{0: NEW_PROCESS, 1: ENVIRONMENT, 2: BRIDGE, 3: EXTERNAL} (Possibly use eagerx_core.constants.process class)
# TODO: Change loglevel integers to their corresponding strings
# TODO: Nice to have: allow to add terminals in nodes
# TODO: Remove from observations: rate, is_reactive, space_converter
# TODO: Remove from actions: start_with_msg, space_converter
# TODO: Make get_nodes_and_objects_library look into all packages inside workspace (look for first 'src' from right-to-left?)
