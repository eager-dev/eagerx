import glob
import sys
from pathlib import Path
from pyqtgraph.Qt import QtGui, QtCore
from eagerx_gui.eagerx_graph import EagerxGraph
from eagerx_core.utils.utils import load_yaml


def load_library():
    library = {'real_reset': {}, 'node': {}, 'object': {}}
    search_path = Path(__file__).parent.parent.parent.parent / '*' / 'config' / '*.yaml'
    node_paths = glob.glob(str(search_path))
    for path in node_paths:
        package = None
        node = None
        for split in path.split('/'):
            if split.startswith('eagerx'):
                package = split
            elif split.endswith('.yaml'):
                node = split[:-5]
        if package == 'eagerx_core' and node in ['actions', 'observations', 'bridge', 'supervisor']:
            continue
        if package is not None and node is not None:
            yaml = load_yaml(package, node)
            if 'node_type' in yaml:
                if 'targets' in yaml:
                    type = 'real_reset'
                else:
                    type = 'node'
            else:
                type = 'object'
            if package not in library[type].keys():
                library[type][package] = []
            library[type][package].append({'name': node, 'yaml': yaml})
    return library


def EagerxGraphCreator():
    library = load_library()
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
# TODO: Update addresses on changes in connection
# TODO: Update information of feedthroughs
