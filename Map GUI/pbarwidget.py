# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'pbarwidget.ui'
#
# Created by: PyQt5 UI code generator 5.6
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_pbarwin(object):
    def setupUi(self, pbarwin):
        pbarwin.setObjectName("pbarwin")
        pbarwin.resize(383, 224)
        pbarwin.setMinimumSize(QtCore.QSize(383, 224))
        pbarwin.setMaximumSize(QtCore.QSize(383, 224))
        self.progressBar = QtWidgets.QProgressBar(pbarwin)
        self.progressBar.setGeometry(QtCore.QRect(9, 150, 371, 23))
        self.progressBar.setProperty("value", 24)
        self.progressBar.setObjectName("progressBar")
        self.label = QtWidgets.QLabel(pbarwin)
        self.label.setGeometry(QtCore.QRect(130, 50, 101, 71))
        self.label.setObjectName("label")

        self.retranslateUi(pbarwin)
        QtCore.QMetaObject.connectSlotsByName(pbarwin)

    def retranslateUi(self, pbarwin):
        _translate = QtCore.QCoreApplication.translate
        pbarwin.setWindowTitle(_translate("pbarwin", "Calibrating"))
        self.label.setText(_translate("pbarwin", "<html><head/><body><p align=\"center\">Calibrating</p><p align=\"center\">Do Not Move</p></body></html>"))
  

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_pbarwin()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
