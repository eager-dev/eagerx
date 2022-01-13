"""
 Mostly copy paste from https://github.com/pyqtgraph/pyqtgraph/blob/master/pyqtgraph/flowchart/FlowchartCtrlTemplate_pyside2.py
"""
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *

from pyqtgraph.widgets.TreeWidget import TreeWidget
from pyqtgraph.widgets.FeedbackButton import FeedbackButton


class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(217, 499)
        self.gridLayout = QGridLayout(Form)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setVerticalSpacing(0)
        self.loadBtn = QPushButton(Form)
        self.loadBtn.setObjectName(u"loadBtn")

        self.gridLayout.addWidget(self.loadBtn, 1, 0, 1, 1)

        self.saveBtn = FeedbackButton(Form)
        self.saveBtn.setObjectName(u"saveBtn")

        self.gridLayout.addWidget(self.saveBtn, 1, 1, 1, 2)

        self.saveAsBtn = FeedbackButton(Form)
        self.saveAsBtn.setObjectName(u"saveAsBtn")

        self.gridLayout.addWidget(self.saveAsBtn, 1, 3, 1, 1)

        self.checkValidityBtn = FeedbackButton(Form)
        self.checkValidityBtn.setObjectName(u"checkValidityBtn")

        self.gridLayout.addWidget(self.checkValidityBtn, 4, 0, 1, 1)

        self.showChartBtn = QtWidgets.QPushButton(Form)
        self.showChartBtn.setCheckable(True)
        self.showChartBtn.setObjectName(u"showChartBtn")

        self.gridLayout.addWidget(self.showChartBtn, 4, 1, 1, 2)
        self.showCompatibleBridgesBtn = FeedbackButton(Form)
        self.showCompatibleBridgesBtn.setObjectName(u"showCompatibleBridgesBtn")

        self.gridLayout.addWidget(self.showCompatibleBridgesBtn, 4, 1, 1, 1)

        self.ctrlList = TreeWidget(Form)
        __qtreewidgetitem = QTreeWidgetItem()
        __qtreewidgetitem.setText(0, u"1");
        self.ctrlList.setHeaderItem(__qtreewidgetitem)
        self.ctrlList.setObjectName(u"ctrlList")
        self.ctrlList.header().setVisible(False)
        self.ctrlList.header().setStretchLastSection(False)

        self.gridLayout.addWidget(self.ctrlList, 3, 0, 1, 4)

        self.fileNameLabel = QLabel(Form)
        self.fileNameLabel.setObjectName(u"fileNameLabel")
        font = QFont()
        font.setBold(True)
        self.fileNameLabel.setFont(font)
        self.fileNameLabel.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.fileNameLabel, 0, 1, 1, 1)


        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"PyQtGraph", None))
        self.loadBtn.setText(QCoreApplication.translate("Form", u"Load..", None))
        self.saveBtn.setText(QCoreApplication.translate("Form", u"Save", None))
        self.saveAsBtn.setText(QCoreApplication.translate("Form", u"As..", None))
        self.showChartBtn.setText(QCoreApplication.translate("Form", u"Show Graph", None))
        self.checkValidityBtn.setText(QCoreApplication.translate("Form", u"Check Validity", None))
        self.showCompatibleBridgesBtn.setText(QCoreApplication.translate("Form", u"Compatible Bridges", None))
        self.fileNameLabel.setText("")
