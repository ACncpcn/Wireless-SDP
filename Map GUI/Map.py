# -*- coding: utf-8 -*-
"""
Created on Wed Apr 11 21:28:14 2018
@author: Alex
"""

from PyQt5 import QtWidgets, QtGui  # Import the PyQt5 module we'll need
import sys  # We need sys so that we can pass argv to QApplication
from PyQt5.QtCore import QThread, pyqtSignal # Used to work with threads
from PyQt5.QtWidgets import QInputDialog

import serial 
import numpy as np
from time import time
from scipy.signal import argrelextrema

import Layout # Layout of GUI

class CoreMap(QThread):
    
    add_data = pyqtSignal(object) # signal that emits data to be plotted on map
    err_line = pyqtSignal(int)    # signal that emits transmitter number that disconnected
    calib_line = pyqtSignal(int)  #signal that emits calibration progress
    
    def __init__(self, ser, parent = None):
        #super(getSerial, self).__init__(parent)
        QThread.__init__(self) 
        self.ser = ser
        
    def run(self):
        

        #start_time = time()
        timepoints = []
        ntr = 2 # number of transmitters
        
        # Receiver parameters
        accel = [[]for i in range(ntr)]
        accelfor = [[]for i in range(ntr)]
        angle =[[]for i in range(ntr)]
        
        # initialize with 0
        x = [(0)for i in range(ntr)]
        y = [(0)for i in range(ntr)]
        
        #view_time = 1 # seconds of data to view at once
        #duration = 30  # total seconds to collect data
        ind = [(0)for i in range(ntr)]
        
        # Values to trigger a step and forward movement
        steplim = 150
        #forlim = 100
        
        # Store Location
        verts0 = [(0,0)]
        verts1 = [(0,0)]
        
        # total steps stored in vector
        totes = [(0)for i in range(ntr)]
        
        # get step length
        self.feet
        self.inch
        
        totinch = self.feet*12 + self.inch
        stridelen = totinch * .41
        stepsize = stridelen * 0.0254
        
        # correction offset
        accelcor = np.array([0,0])  
        accelforcor = np.array([0,0])
        anglecor = np.array([0,0])
        i = np.array([0,0])
        itst = np.array([0,0])
        
        # time variables
        start_time = time()
        calib_tim = 10 #20
        
        # calibration        
        while (time() <= start_time + calib_tim):
            #just read data to get calibration moving
            self.ser.reset_input_buffer()
            data = self.ser.readline()
            data = data.decode().split() # parse data 
            
            try:
                # get current transmitter number
                client = int(data[3])
                itst[client] = itst[client] + 1
            except:
                # if current trasnmitter is incorrect data type there has been
                # an error in transmission and we pass to next transmission
                pass
        
            # % done
            percent = (time() - start_time) / (calib_tim)
            #print("Progress {:2.0%}".format(percent), end="\r")
            self.calib_line.emit(np.around(percent*100))
            # Store last values to caLculate offsets
            
            # Check if transmitters are connected
            if (np.amax(itst) > np.amin(itst) + 10):
                #print('Transmitter %d has stopped working' % np.argmin(itst))
                #input('Press Enter when you have reconnected transmitter')
                
                # emit signal to pop up warning message argmin(itst) gives the
                # transmitter number that has been disconnected
                self.err_line.emit(np.argmin(itst)) 
                
                self.Pause = True
                while self.Pause == True:
                    # Wait here until warning message has been closes
                    pass
                
                # reset calibration values
                start_time = time() # reset timer to calibrate again
                itst = np.array([0,0])          
            
            if (time() > start_time + calib_tim - 2):
                #print('do not move')
                client = int(data[3])
                anglecor[client] = anglecor[client] + float(data[0])
                accelcor[client] = accelcor[client] + float(data[1])
                accelforcor[client] = accelforcor[client] + float(data[2])
                i[client] = i[client] + 1
                
        
        # Average calibration data to find offsets for each transmitter
        anglecor = anglecor/i        
        accelcor = accelcor/i
        accelforcor = accelforcor/i
        
        # reset start time to use as reference for next loop
        start_time = time()
        
        run =  True
        initial = [0,0] # used to get first dataset
        
        # collect the data and plot a moving frame. When this loop stops, the map stops
        while run:
            run2 = True
            startind = ind*1
            
            # count of iterations taken in next loop
            iloop = [0,0]

            # Loop to get a chunk of data to later process
            while run2:
                
                try:
                    # get data
                    self.ser.reset_input_buffer()
                    data = self.ser.readline()
                    data = data.decode().split()
                except:
                    # An error will occur when serial comm is closed outside run
                    # this happens when we finish mapping so this is an external
                    # signal to stop gathering data
                    run2 = False
                
                try:                      
                    # store the entire dataset for later
                    client = int(data[3]) # the address of current transmitter data
                    accel[client].append(float(data[1]) - accelcor[client]) #acceleration up/down
                    angle[client].append(float(data[0]) - anglecor[client]) #angle 
                    accelfor[client].append(float(data[2]) - accelforcor[client]) # acceleration forward/backward
                    
                    if initial[client] == 1: # increment loop count after first dataset
                        iloop[client] = iloop[client] + 1  
                    
                    # Get time at end of cycle to know whether to stop outer loop
                    if client == 0: 
                        timepoints.append(time()-start_time)
        
                    # when time's up, kill the collect loop
                    # kill run2 when we gather at least 5 samples from both transmitters
                    if (iloop[0] >= 5) & (iloop[1] >= 5): run2 = False
                        
                    # check if a transmitter has stopped. If samples are 3 behind
                    # zero pad the difference of samples
                    elif (np.amax(iloop) > np.amin(iloop)+10):
                        troff = np.argmin(iloop) # transmitter that turned off
                        
                        # Don't move on until pop up message is dismissed
                        self.err_line.emit(troff) 
                        self.Pause = True
                        while self.Pause == True:
                            pass
                        
                        npad = np.amax(iloop) - np.amin(iloop)
                        
                        for i in range(npad):
                            # zero pad the missed data, append timepoints
                            accel[troff].append(0)
                            angle[troff].append(0)
                            accelfor[troff].append(0)
                            iloop[troff] = iloop[troff] + 1
                            if client == 0: 
                                # append time points if necessary
                                timepoints.append(time()-start_time) 
                                       
        
                # if the try statement throws an error, just do nothing
                except: pass
                
                # Record current sample index (depending on current transmitter data)
                if initial[client] == 1 :
                    ind[client] = ind[client] + 1
                else: # This is reached after first dataset so change initial to count loops
                    initial[client] = 1
                
            overlap = 1 # frame overlap
        
            # Acceleration Peaks
            # determine the indices of the local maxima
            # i is for transmitter data being processed
            for i in range(2):
        
                if(startind[i] < overlap): #don't begin overlap until we have enough samples to overlap
                    overlap = 0
                    
                accelframe = np.array(accel[i][startind[i]-overlap:ind[i] + 1])
                #accelforframe = np.array(accelfor[i][startind[i]-overlap:ind[i] + 1])
                #print(accelframe)
                maxInd = np.array(argrelextrema(accelframe, np.greater)[0])
        
                # get the actual values using these indices
                peaks = accelframe[maxInd]  # array([5, 3, 6]) peak moving average accel values
                stepind = np.nonzero(peaks > steplim)[0] # index of local maxima that are higher than 500       
                stepsamp = maxInd[stepind] # index of whole sample vector where step occurs
                peaks = peaks[stepind]
                step = peaks.shape[0]
                #dist = step* stepsize
                    
                
                totes[i] = totes[i] + step
            
                # Forward Backward Angle
        
                # polar to cartesian coordinates
            
            # j refers to index in stepsamp and is used to determine which step is being evaluated
                for samp in stepsamp:
        
                    # Find whether step was forward/backward or in place
                    # forward acceleration at moment of step, we move our bodies 2 samples before we move up
                    #stepfor = accelforframe[samp]
                    #steprange = 1000
                    direction = 1
                    #if (abs(stepfor) <= steprange): direction = 1 
                    #elif (abs(stepfor) > steprange): direction  = 1.5
                    #elif (stepfor < -forlim+50): direction = -1
                    #else: direction = 0
                    #print(stepfor)
                    #if i  == 0:
                    #    print(direction)
                    #    print(accelforframe)
        
                    # take angle and find distance for each step
                    anglestep = angle[i][samp + startind[i]-overlap]
                    #print('Angle: ' + str(anglestep))
                    x[i] = direction*stepsize*np.cos(np.deg2rad(anglestep)) + x[i]
                    y[i] = direction*stepsize*np.sin(np.deg2rad(anglestep)) + y[i]
            
            # get plot coordinates
            verts0.append((x[0],y[0]))
            xs, ys = zip(*verts0)
            verts1.append((x[1],y[1]))
            xs1, ys1 = zip(*verts1)
            
            # emit signal when it is time to update map with coordinates
            # xs is for path history and x[0] is for current position marker
            self.add_data.emit([xs, ys, xs1, ys1, x[0], y[0], x[1], y[1]])


class ExampleApp(QtWidgets.QMainWindow, Layout.Ui_MainWindow):
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setupUi(self)  # This is defined in design.py file automatically
        # It sets up layout and widgets that are defined
        self.StartToggle.toggled.connect(self.Wait)  # change stream status if toggled        

        # default COM port is COM3
        self.Com3Action.setChecked(True)
        self.Com = ("COM3")
        
        # if user changes com port menu option go to serchoose
        self.ComAction.triggered.connect(self.serchoose)

        
    def serchoose(self):
        ''' Select which com port to open'''\
        
        if (self.Com1Action.isChecked() == 1):
            self.Com = "COM1"
        elif (self.Com2Action.isChecked() == 1):
            self.Com = "COM2" 
        elif (self.Com3Action.isChecked() == 1):
            self.Com = "COM3"
        elif (self.Com4Action.isChecked() == 1):
            self.Com = "COM4"
        elif (self.Com5Action.isChecked() == 1):
            self.Com = "COM5"            
            
            
    def Wait(self):
        ''' 
        Connect signal to plot function and
        wait until signal fires
        '''
        
        # run if user selects to start mapping
        if self.StartToggle.isChecked() == 1:
    
            try:
                # start serial communication with com port chosen prior
                self.ser = serial.Serial(self.Com, 9600)    
            
                # input dialog to get user height
                text1, feetinput = QInputDialog.getInt(self, 'Settings', 'Enter feet: ')
                
                if feetinput == True:
                    text2, inchinput = QInputDialog.getInt(self, 'Settings', 'Enter inches: ')
                    if inchinput == True:
                        
    
                            
                    
                        self.openWindow()
                        self.pbar.setMinimum(0)
                        self.pbar.setMaximum(100)
                        self.pbar.setValue(0)
            
                        # initialize map thread
                        self.map_thread = CoreMap(self.ser)
                        
                        # send feet and inch input values to map thread
                        self.map_thread.feet = feetinput 
                        self.map_thread.inch = inchinput
                        
                        # add signal. When signal fires go to plot data
                        self.map_thread.add_data.connect(self.Plot)
                        
                        # when error signal fires pop up error message
                        self.map_thread.err_line.connect(self.errormsg)
                        
                        self.map_thread.calib_line.connect(self.progbar)
                        
                        self.map_thread.start() #begin running CoreMap
                    else:
                        self.close()
                else:
                    self.close()
            
            # if com port can not be opened show warning message and don't map
            except:
                self.StartToggle.setChecked(False) # unselect start button
                msg = QtWidgets.QMessageBox() 
                msg.setIcon(QtWidgets.QMessageBox.Warning)
                text = self.Com + " is not available"
                msg.setText(text)
                msg.setWindowTitle("COM Port Error")
                msg.exec_()
       
        # if start button is toggled off
        else:
            self.map_thread.disconnect() # disconnect thread
            self.ser.close() # close serial communication
                
    
    def Plot(self, data):
        ''' plot to matplotlib figure '''
        
        # instead of ax.hold(False)
        self.figure.clear()
        
        # create an axis
        ax = self.figure.add_subplot(111)
        ax.grid(True)
        
        # map limits
        mrange = [-1, 1] # map range
        yrange = [-1,1]
        ax.set_xlim(mrange)
        ax.set_ylim(yrange)
        
        # data is vector that is emitted from run
        x , y , x1, y1, xp, yp, xp1, yp1 = data
        
        # plot data
        ax.set_title('Map')
        ax.plot(x, y, '--', lw=2, color='red', ms=10)
        ax.plot(xp, yp, 'ro')
        ax.plot(x1, y1, '--', lw=2, color='blue', ms=10)
        ax.plot(xp1, yp1, 'ro', color = 'blue')
        
        # refresh canvas
        self.canvas.draw()
        
    def errormsg(self, errtr):
        '''errtr is passed from CoreMap when a transmitter is disconnected. errtr
        gives which transmitter is disconnected. A warning window pop ups.
        '''
        
        # Warning Message
        msg = QtWidgets.QMessageBox() 
        msg.setIcon(QtWidgets.QMessageBox.Warning)
        text = "Transmitter " + str (errtr) + " has stopped working\nRecconect it"
        msg.setText(text)
        msg.setWindowTitle("Transmitter Error")
        msg.exec_()
        
        # Resume CoreMap when messagebox is dismissed
        self.map_thread.Pause = False
        
    def progbar(self, value):
        ''' set progress bar percent value'''
        
        self.pbar.setValue(value)
        
        if int(value) == 100:
            self.window.hide()


def main():
    app = QtWidgets.QApplication(sys.argv)  # A new instance of QApplication
    form = ExampleApp()  # We set the form to be our ExampleApp (design)
    form.show()  # Show the form
    app.exec_()  # and execute the app

if __name__ == '__main__':  # if we're running file directly and not importing it
    main()  # run the main function