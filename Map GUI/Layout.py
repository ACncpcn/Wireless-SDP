# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'vertlist.ui'
#
# Created by: PyQt5 UI code generator 5.6
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt

from pbarwidget import Ui_pbarwin

class Ui_MainWindow(object):
    
    def openWindow(self):
        self.window = QtWidgets.QMainWindow()
        self.ui = Ui_pbarwin()
        self.ui.setupUi(self.window)
        self.pbar = self.ui.progressBar
        self.window.show()
        
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        
        # a figure instance to plot on
        self.figure = plt.figure()

        # this is the Canvas Widget that displays the `figure`
        # it takes the `figure` instance as a parameter to __init__
        self.canvas = FigureCanvas(self.figure)
        
        self.verticalLayout.addWidget(self.canvas)
        self.StartToggle = QtWidgets.QRadioButton(self.centralwidget)
        self.StartToggle.setObjectName("StartToggle")
        self.verticalLayout.addWidget(self.StartToggle)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        
                # Personalization
        self.setWindowTitle("Wi-Find")
        self.setWindowIcon(QtGui.QIcon('WiFindLogo.png'))
        
        # Quitter
        self.QuitAction = QtWidgets.QAction("&Quit", self)
        self.QuitAction.setShortcut("Ctrl+Q")
        self.QuitAction.setStatusTip('Close Application')
        self.QuitAction.triggered.connect(self.close)
        
        self.ComAction = QtWidgets.QActionGroup(self)
        self.Com1Action = self.ComAction.addAction(QtWidgets.QAction('COM1',self))
        self.Com2Action = self.ComAction.addAction(QtWidgets.QAction('COM2',self))
        self.Com3Action = self.ComAction.addAction(QtWidgets.QAction('COM3',self))
        self.Com4Action = self.ComAction.addAction(QtWidgets.QAction('COM4',self))
        self.Com5Action = self.ComAction.addAction(QtWidgets.QAction('COM5',self))
        
        self.Com1Action.setStatusTip('COM1')
        self.Com1Action.setCheckable(True)
        self.Com2Action.setStatusTip('COM2')
        self.Com2Action.setCheckable(True)
        self.Com3Action.setStatusTip('COM3')
        self.Com3Action.setCheckable(True)
        self.Com4Action.setStatusTip('COM4')
        self.Com4Action.setCheckable(True)
        self.Com5Action.setStatusTip('COM5')
        self.Com5Action.setCheckable(True)
        
        # Displaying Menu and Status Bar
        self.statusBar()
        
        mainMenu = self.menuBar()
        
        fileMenu = mainMenu.addMenu('&Serial Port')
        fileMenu.addAction(self.Com1Action)
        fileMenu.addAction(self.Com2Action)
        fileMenu.addAction(self.Com3Action)
        fileMenu.addAction(self.Com4Action)
        fileMenu.addAction(self.Com5Action)
        fileMenu = mainMenu.addMenu('&Options')
        fileMenu.addAction(self.QuitAction)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Wi-Find"))
        self.StartToggle.setText(_translate("MainWindow", "Start/Stop"))

