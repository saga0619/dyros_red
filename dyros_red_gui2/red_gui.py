#!/usr/bin/env python

import sys
import signal
import time
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QThread, QCoreApplication, QObject, QRunnable, QThreadPool, pyqtSignal
import rospy
from mujoco_ros_msgs.msg import SimStatus
import std_msgs
import geometry_msgs.msg

bup = True

class rosupdate(QThread):
    signal = pyqtSignal('PyQt_PyObject')

    def __init__(self, parent = None):

        signal.signal(signal.SIGINT, signal.SIG_DFL)
        super(rosupdate,self).__init__(parent)
        rospy.init_node('red_gui',anonymous=True)
        #rospy.Subscriber("/mujoco_ros_interface/sim_status", SimStatus, self.simstatus_callback)
        rospy.Subscriber('/dyros_red/time', std_msgs.msg.Float32, self.sub_cb, queue_size=1)

    def run(self):
        count = 0
        global bup
        while bup:
            time.sleep(0.01)
            #rospy.spin()

    def sub_cb(self, msg):
        self.signal.emit(msg)


class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        super(Ui, self).__init__()
        uic.loadUi('main.ui',self)

        rospy.init_node('red_gui',anonymous=True)
        #rospy.Subscriber("/mujoco_ros_interface/sim_status", SimStatus, self.simstatus_callback)
        rospy.Subscriber('/dyros_red/time', std_msgs.msg.Float32, self.sub_cb, queue_size=1)
        rospy.Subscriber('/dyros_red/point', geometry_msgs.msg.PolygonStamped, self.sub_cb2, queue_size=1)


        self.pushButton.clicked.connect(self.btnclicked)
        self.pushButton_2.clicked.connect(self.btnclicked2)
        #self.threadclass = rosupdate()
        #self.threadclass.signal.connect(self.threadhandle)

        self.show()

    def btnclicked(self):
        global bup
        bup = True
        self.threadclass.start()

    def btnclicked2(self):
        global bup
        self.label.setText(str(0))
        bup = False

    def threadhandle(self, result):
        #self.label.setText(str(round(result,4)))
        sim_time = result.data
        if bup:
            self.label.setText(str(sim_time))

    def sub_cb(self, msg):
        self.label.setText(str(msg.data))

    def sub_cb2(self, msg):
        self.label.setText(str(round(msg.polygon.points[0].x, 6)))
        self.label_2.setText(str(round(msg.polygon.points[0].y, 6)))
        self.label_3.setText(str(round(msg.polygon.points[0].z, 6)))

        # pelvis
        self.label_14.setText(str(round(msg.polygon.points[3].x, 6)))
        self.label_15.setText(str(round(msg.polygon.points[3].y, 6)))
        self.label_16.setText(str(round(msg.polygon.points[3].z, 6)))

        # right foot
        self.label_73.setText(str(round(msg.polygon.points[1].x, 6)))
        self.label_74.setText(str(round(msg.polygon.points[1].y, 6)))
        self.label_75.setText(str(round(msg.polygon.points[1].z, 6)))

        # left foot
        self.label_64.setText(str(round(msg.polygon.points[2].x, 6)))
        self.label_65.setText(str(round(msg.polygon.points[2].y, 6)))
        self.label_66.setText(str(round(msg.polygon.points[2].z, 6)))

        # zmp  
        self.label_22.setText(str(round(msg.polygon.points[9].x, 6)))
        self.label_23.setText(str(round(msg.polygon.points[9].y, 6)))

        #self._widget.label_24.setText(str(round(msg.polygon.points[5].x, 6)))
        #self._widget.label_27.setText(str(round(msg.polygon.points[5].y, 6)))

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()

