import sys
import yaml
from yaml import dump
from typing import List, Union, Dict, Tuple, Optional
from pyqtgraph.Qt import QtGui, QtCore

from eagerx_core.params import RxNodeParams, RxObjectParams
from eagerx_core.utils.connection_utils import register_connections
from eagerx_core.gui.eagerx_graph import EagerxGraph

class RxGraph:
    def __init__(self, state: Dict):
        self._state = state

    @classmethod
    def create_with_gui(cls):
        # todo:
        pass

    @classmethod
    def create(cls, nodes: List[RxNodeParams], objects: List[RxObjectParams]):
        if isinstance(nodes, RxNodeParams):
            nodes = [nodes]
        if isinstance(objects, RxObjectParams):
            objects = [objects]

        # Add action & observation node to list
        # todo: make deepcopy of all nodes/objects? or make deep copy into self._state?
        actions = RxNodeParams.create(name='env/actions', package_name='eagerx_core', config_name='actions')
        observations = RxNodeParams.create(name='env/observations', package_name='eagerx_core', config_name='observations')
        nodes = [actions, observations] + nodes

        # todo: convert to _state --> dict with node yamls/params & 'connection'
        state = dict()
        return cls(state)

    def connect(self,
                source: Optional[Tuple[str, str, str]] = None,
                target: Optional[Tuple[str, str, str]] = None,
                action: str = None, observation: str = None,
                converter: Optional[Dict] = None,
                window: Optional[int] = None,
                delay: Optional[float] = None):
        assert not source or not action, 'You cannot specify a source if you wish to connect action "%s", as the action will act as the source.' % action
        assert not target or not observation, 'You cannot specify a target if you wish to connect observation "%s", as the observation will act as the target.' % observation
        assert not (observation and action), 'You cannot connect an action directly to an observation.'

        if action:
        # todo: add action details to params inside self._state
            source = ('env/actions', 'outputs', action)

        if observation:
            # todo: add action details to params inside self._state
            target = ('env/observations', 'inputs', observation)

        # todo: add converter, window, delay to self._state[target[0]=name]['params'][target[1]=component][target[2]=cname]

        # todo: add connection (based on source & target), to self._state['connects'].append((target[0]=name, 'target[1]/target[2]', source[0]=name, 'source[1]/source[2]' ))

    def remove_connection(self):
        # todo: implement this function as well?
        # todo: What do we remove (delay, converter, window): We go back to original yaml?
        pass

    def register_connection(self):
        # todo: called inside RxEnv
        # todo: convert self._state to RxObjectParams, RxNodeParams & connection list
        # todo: output params:  actions, observations, nodes, objects, nodes, render to be initialized
        ...

    def render(self, source: Tuple[str, str, str],
               rate: float,
               converter: Optional[Dict] = None,
               window: Optional[int] = None,
               delay: Optional[float] = None,
               **kwargs):
        # if 'env/render' in self._state['nodes']:
            # todo: delete old render from self._state['nodes']
            # ...

        # todo: add render node to self._state
        # Add render node to self._state['node']
        render = RxNodeParams.create('env/render', 'eagerx_core', 'render', rate=rate, **kwargs)

        # Create connection
        target = ('env/render', 'inputs', 'image')
        self.connect(source=source, target=target, converter=converter, window=window, delay=delay)

    def save(self, path: str):
        # todo: save self._state somewhere (automatically append .graph to path? Or let user decide?)
        pass

    def load(self, path: str):
        # todo: load a graph (automatically append .graph to path? Or let user decide?)
        self._state = yaml.load(path)

    def is_valid(self):
        # todo: test if valid (DAG checks)
        pass

    def gui(self):
        # todo: Open GUI by passing it the self._state
        # todo: When the GUI is closed, overwrite self._state
        app = QtGui.QApplication([])

        # Create main window with grid layout
        win = QtGui.QMainWindow()
        win.setWindowTitle('EAGERx Graph')
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

        self._state = fc.saveState()
