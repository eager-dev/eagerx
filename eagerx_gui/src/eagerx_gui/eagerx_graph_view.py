# -*- coding: utf-8 -*-
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph.widgets.GraphicsView import GraphicsView
from pyqtgraph.graphicsItems.ViewBox import ViewBox


class EagerxGraphView(GraphicsView):
    
    sigHoverOver = QtCore.Signal(object)
    sigClicked = QtCore.Signal(object)
    
    def __init__(self, widget, *args):
        GraphicsView.__init__(self, *args, useOpenGL=False)
        self._vb = EagerxGraphViewBox(widget, lockAspect=True, invertY=True)
        self.setCentralItem(self._vb)
        self.setRenderHint(QtGui.QPainter.Antialiasing, True)
    
    def viewBox(self):
        return self._vb
    
        
class EagerxGraphViewBox(ViewBox):
    
    def __init__(self, widget, *args, **kwargs):
        ViewBox.__init__(self, *args, **kwargs)
        self.widget = widget
        
    def getMenu(self, ev):
        ## called by ViewBox to create a new context menu
        self._fc_menu = QtGui.QMenu()
        self._subMenus = self.getContextMenus(ev)
        for menu in self._subMenus:
            self._fc_menu.addMenu(menu)
        return self._fc_menu
    
    def getContextMenus(self, ev):
        ## called by scene to add menus on to someone else's context menu
        menus = self.widget.buildMenu(ev.scenePos())
        menus.append(ViewBox.getMenu(self, ev))
        return menus
