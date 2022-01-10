from functools import wraps
from copy import deepcopy
from pyqtgraph.Qt import QtGui


def exception_handler(function_to_decorate):
    wraps(function_to_decorate)

    def exception_handler_wrapper(*args, graph_backup=None, dialog_title='Invalid Action', **kwargs):
        assert graph_backup is not None, 'Graph should be defined for recovering state.'
        state_copy = deepcopy(graph_backup._state)
        try:
            return function_to_decorate(*args, **kwargs)
        except Exception as e:
            graph_backup._state = state_copy
            graph_backup.load_state(clear=True)
            error_window = QtGui.QDialog(graph_backup.widget().cwWin)
            error_window.setWindowTitle(dialog_title)
            layout = QtGui.QGridLayout()
            label = QtGui.QLabel(str(e))
            layout.addWidget(label)
            error_window.setLayout(layout)
            error_window.exec_()
            raise
    return exception_handler_wrapper
