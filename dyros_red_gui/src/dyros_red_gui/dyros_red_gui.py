import os
import rospy
import rospkg
import numpy
import math
import std_msgs.msg
import geometry_msgs.msg
import dyros_red_msgs.msg

import sensor_msgs.msg._JointState
from std_msgs.msg import String

#from sensor_msgs.msg import JointState
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

#from python_qt_binding.QtCore import Qt, QTimer, Slot
#from python_qt_binding.QtWidgets import QWidget

from PyQt5.QtCore import Qt, QTimer, Slot
from PyQt5.QtWidgets import QWidget
from PyQt5.QtWidgets import QErrorMessage, QMessageBox
from PyQt5.QtGui import QKeySequence
from PyQt5.QtGui import QDoubleValidator
from PyQt5.QtGui import QIcon

run_mode = ''
gain_var = 0
control_time = 0.0


class DyrosRedGuiPlugin(Plugin):

    def __init__(self, context):
        super(DyrosRedGuiPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DyrosRedGUI')
        self._publisher = rospy.Publisher(
            '/dyros_red/command', String, queue_size=10)
        self._publisher2 = rospy.Publisher(
            '/dyros_red/taskcommand', dyros_red_msgs.msg.TaskCommand, queue_size=10)

        global run_mode
        run_mode = rospy.get_param('/dyros_red_controller/run_mode', 'realrobot')

        if run_mode == "simulation":
            self._publisher3 = rospy.Publisher('/mujoco_ros_interface/sim_command_con2sim', String, queue_size=10)

        if run_mode == "realrobot":
            self._publisher4 = rospy.Publisher('/dyros_red/gain_command', dyros_red_msgs.msg.GainCommand, queue_size=10)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path(
            'dyros_red_gui'), 'resource', 'DyrosRed.ui')
        icon_file = os.path.join(rospkg.RosPack().get_path(
            'dyros_red_gui'), 'resource', 'Dyros_Logo3.png')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DyrosRedUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
            self._widget.setWindowIcon(QIcon(icon_file))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self._widget.gravity_button.pressed.connect(self.gravity_button)
        self._widget.task_button.pressed.connect(self.task_button)
        self._widget.contact_button.pressed.connect(self.contact_button)
        self._widget.data_button.pressed.connect(self.data_button)
        self._widget.com_send_button.pressed.connect(self.com_command_sender)

        # self._widget.pause_button.pressed.connect(self.pause_button)
        # self._widget.slowmotion_button.pressed.connect(self.slowmotion_button)
        # self._widget.reset_button.pressed.connect(self.reset_button)

        self._widget.comboBox.currentIndexChanged.connect(self.comboBox_select)

        # torque on/off buttons
        self._widget.torqueon_button.pressed.connect(self.torqueon_button)
        self._widget.emergencyoff_button.pressed.connect(self.emergencyoff_button)
        self._widget.torqueoff_button.pressed.connect(self.torqueoff_button)
        self._widget.torqueoff_button.setEnabled(True)
        self._widget.torqueon_button.setEnabled(True)

        # tune command buttons
        self._widget.tune_command_button.pressed.connect(self.tune_command_button)
        self._widget.tune_reset_button.pressed.connect(self.tune_reset_button)
        self._widget.tune_off_button.pressed.connect(self.tune_off_button)

        self._widget.helpbutton.setShortcut(QKeySequence("h"))
        self._widget.emergencyoff_button.setShortcut(QKeySequence(Qt.Key_Escape))
        self._widget.com_send_button.setShortcut(QKeySequence(Qt.Key_Return))
        self._widget.torqueon_button.setShortcut(QKeySequence("w"))
        self._widget.torqueoff_button.setShortcut(QKeySequence("q"))
        self._widget.task_button.setShortcut(QKeySequence("p"))
        torqueon = False

        self._widget.torqueredis.pressed.connect(self.torqueredis_button)

        # command tab buttons

        self._widget.sebutton.pressed.connect(self.sebutton)

        # Limit QLineEdit input to double.
        dbl_val = QDoubleValidator()

        self._widget.tn1.setValidator(dbl_val)
        self._widget.tn2.setValidator(dbl_val)
        self._widget.tn3.setValidator(dbl_val)
        self._widget.tn4.setValidator(dbl_val)
        self._widget.tn5.setValidator(dbl_val)
        self._widget.tn6.setValidator(dbl_val)

        self._widget.tn7.setValidator(dbl_val)
        self._widget.tn8.setValidator(dbl_val)
        self._widget.tn9.setValidator(dbl_val)
        self._widget.tn10.setValidator(dbl_val)
        self._widget.tn11.setValidator(dbl_val)
        self._widget.tn12.setValidator(dbl_val)

        self._widget.text_angle.setValidator(dbl_val)
        self._widget.text_height.setValidator(dbl_val)
        self._widget.text_pos.setValidator(dbl_val)
        self._widget.text_time.setValidator(dbl_val)

        if run_mode == "realrobot":
            self._widget.gravity_button.setEnabled(True)
            self._widget.task_button.setEnabled(True)
            self._widget.contact_button.setEnabled(True)
            self._widget.data_button.setEnabled(True)
            self._widget.torqueoff_button.setEnabled(True)
            self._widget.torqueon_button.setEnabled(True)
            self._widget.tab_2.setEnabled(True)

        self._widget.run_mode.setText(run_mode)
        self.sub = rospy.Subscriber('/dyros_red/point', geometry_msgs.msg.PolygonStamped, self.sub_cb, queue_size=1)
        self.sub2 = rospy.Subscriber('/dyros_red/jointstates', sensor_msgs.msg.JointState, self.sub_cb2, queue_size=1)
        self.sub3 = rospy.Subscriber('/dyros_red/time', std_msgs.msg.Float32, self.sub_cb3, queue_size=1)
        self.sub4 = rospy.Subscriber('/dyros_red/motorinfo', dyros_red_msgs.msg.MotorInfo, self.sub_cb4, queue_size=1)
        self.sub5 = rospy.Subscriber('/dyros_red/torquegain', std_msgs.msg.Float32, self.sub_cb5, queue_size=1)

    def sub_cb(self, msg):
        self._widget.label.setText(str(round(msg.polygon.points[0].x, 6)))
        self._widget.label_2.setText(str(round(msg.polygon.points[0].y, 6)))
        self._widget.label_3.setText(str(round(msg.polygon.points[0].z, 6)))

        # pelvis
        self._widget.label_14.setText(str(round(msg.polygon.points[3].x, 6)))
        self._widget.label_15.setText(str(round(msg.polygon.points[3].y, 6)))
        self._widget.label_16.setText(str(round(msg.polygon.points[3].z, 6)))

        # right foot
        self._widget.label_73.setText(str(round(msg.polygon.points[1].x, 6)))
        self._widget.label_74.setText(str(round(msg.polygon.points[1].y, 6)))
        self._widget.label_75.setText(str(round(msg.polygon.points[1].z, 6)))

        # left foot
        self._widget.label_64.setText(str(round(msg.polygon.points[2].x, 6)))
        self._widget.label_65.setText(str(round(msg.polygon.points[2].y, 6)))
        self._widget.label_66.setText(str(round(msg.polygon.points[2].z, 6)))

        # zmp  
        self._widget.label_22.setText(str(round(msg.polygon.points[9].x, 6)))
        self._widget.label_23.setText(str(round(msg.polygon.points[9].y, 6)))

        #self._widget.label_24.setText(str(round(msg.polygon.points[5].x, 6)))
        #self._widget.label_27.setText(str(round(msg.polygon.points[5].y, 6)))

    def sub_cb2(self, msg):
        self._widget.p0.setText(str(round(msg.position[0], 6)))
        self._widget.p1.setText(str(round(msg.position[1], 6)))
        self._widget.p2.setText(str(round(msg.position[2], 6)))
        self._widget.p3.setText(str(round(msg.position[3], 6)))
        self._widget.p4.setText(str(round(msg.position[4], 6)))
        self._widget.p5.setText(str(round(msg.position[5], 6)))
        self._widget.p6.setText(str(round(msg.position[6], 6)))
        self._widget.p7.setText(str(round(msg.position[7], 6)))
        self._widget.p8.setText(str(round(msg.position[8], 6)))
        self._widget.p9.setText(str(round(msg.position[9], 6)))
        self._widget.p10.setText(str(round(msg.position[10], 6)))
        self._widget.p11.setText(str(round(msg.position[11], 6)))

        self._widget.v0.setText(str(round(msg.velocity[0], 6)))
        self._widget.v1.setText(str(round(msg.velocity[1], 6)))
        self._widget.v2.setText(str(round(msg.velocity[2], 6)))
        self._widget.v3.setText(str(round(msg.velocity[3], 6)))
        self._widget.v4.setText(str(round(msg.velocity[4], 6)))
        self._widget.v5.setText(str(round(msg.velocity[5], 6)))
        self._widget.v6.setText(str(round(msg.velocity[6], 6)))
        self._widget.v7.setText(str(round(msg.velocity[7], 6)))
        self._widget.v8.setText(str(round(msg.velocity[8], 6)))
        self._widget.v9.setText(str(round(msg.velocity[9], 6)))
        self._widget.v10.setText(str(round(msg.velocity[10], 6)))
        self._widget.v11.setText(str(round(msg.velocity[11], 6)))

        self._widget.t0.setText(str(round(msg.effort[0], 6)))
        self._widget.t1.setText(str(round(msg.effort[1], 6)))
        self._widget.t2.setText(str(round(msg.effort[2], 6)))
        self._widget.t3.setText(str(round(msg.effort[3], 6)))
        self._widget.t4.setText(str(round(msg.effort[4], 6)))
        self._widget.t5.setText(str(round(msg.effort[5], 6)))
        self._widget.t6.setText(str(round(msg.effort[6], 6)))
        self._widget.t7.setText(str(round(msg.effort[7], 6)))
        self._widget.t8.setText(str(round(msg.effort[8], 6)))
        self._widget.t9.setText(str(round(msg.effort[9], 6)))
        self._widget.t10.setText(str(round(msg.effort[10], 6)))
        self._widget.t11.setText(str(round(msg.effort[11], 6)))

        global gain_var
        self._widget.torque_status.setText(str(round(gain_var, 0)) + '%')
        global control_time
        self._widget.currenttime.setText(str(round(control_time, 4)))

    def sub_cb3(self, msg):
        global control_time
        control_time = msg.data

    def sub_cb4(self, msg):
        self._widget.mi0.setText(str(round(msg.motorinfo1[0], 6)))
        self._widget.mi1.setText(str(round(msg.motorinfo1[1], 6)))
        self._widget.mi2.setText(str(round(msg.motorinfo1[2], 6)))
        self._widget.mi3.setText(str(round(msg.motorinfo1[3], 6)))
        self._widget.mi4.setText(str(round(msg.motorinfo1[4], 6)))
        self._widget.mi5.setText(str(round(msg.motorinfo1[5], 6)))
        self._widget.mi6.setText(str(round(msg.motorinfo1[6], 6)))
        self._widget.mi7.setText(str(round(msg.motorinfo1[7], 6)))
        self._widget.mi8.setText(str(round(msg.motorinfo1[8], 6)))
        self._widget.mi9.setText(str(round(msg.motorinfo1[9], 6)))
        self._widget.mi10.setText(str(round(msg.motorinfo1[10], 6)))
        self._widget.mi11.setText(str(round(msg.motorinfo1[11], 6)))

        self._widget.mi12.setText(str(round(msg.motorinfo2[0], 6)))
        self._widget.mi13.setText(str(round(msg.motorinfo2[1], 6)))
        self._widget.mi14.setText(str(round(msg.motorinfo2[2], 6)))
        self._widget.mi15.setText(str(round(msg.motorinfo2[3], 6)))
        self._widget.mi16.setText(str(round(msg.motorinfo2[4], 6)))
        self._widget.mi17.setText(str(round(msg.motorinfo2[5], 6)))
        self._widget.mi18.setText(str(round(msg.motorinfo2[6], 6)))
        self._widget.mi19.setText(str(round(msg.motorinfo2[7], 6)))
        self._widget.mi20.setText(str(round(msg.motorinfo2[8], 6)))
        self._widget.mi21.setText(str(round(msg.motorinfo2[9], 6)))
        self._widget.mi22.setText(str(round(msg.motorinfo2[10], 6)))
        self._widget.mi23.setText(str(round(msg.motorinfo2[11], 6)))

    def sub_cb5(self, msg):
        global gain_var
        gain_var = round(msg.data*100, 0)

    def pause_button(self):
        self.send_msg2("pause")

    def slowmotion_button(self):
        self.send_msg2("mjslowmotion")

    def sebutton(self):
        self.send_msg("stateestimation")

    def reset_button(self):
        self.send_msg2("mjreset")

    def tune_reset_button(self):
        self._widget.tnc1.setText('0.1724')
        self._widget.tnc2.setText('0.2307')
        self._widget.tnc3.setText('0.2834')
        self._widget.tnc4.setText('0.2734')
        self._widget.tnc5.setText('0.2834')
        self._widget.tnc6.setText('0.0811')

        self._widget.tn1.setText('0.1724')
        self._widget.tn2.setText('0.2307')
        self._widget.tn3.setText('0.2834')
        self._widget.tn4.setText('0.2734')
        self._widget.tn5.setText('0.2834')
        self._widget.tn6.setText('0.0811')

        self._widget.tn7.setText('0.1724')
        self._widget.tn8.setText('0.2307')
        self._widget.tn9.setText('0.2834')
        self._widget.tn10.setText('0.2734')
        self._widget.tn11.setText('0.2834')
        self._widget.tn12.setText('0.0811')

        self.send_msg("tunecurrent")

    def tune_off_button(self):
        self.send_msg("tunereset")

    def com_command_sender(self):
        idx = self._widget.comboBox.currentIndex()
        print("Sending COM command mgs")
        com_command_msg = dyros_red_msgs.msg.ComCommand()
        c_pos = float(self._widget.text_pos.text())
        if idx == 0:
            if c_pos < 0 and c_pos > 1:
                print("com pos value must between 0 ~ 1")
            list = [0, c_pos, 1]
            list.sort()
            com_command_msg.ratio = list[1]
        else:
            com_command_msg.ratio = c_pos

        t_time = float(self._widget.text_time.text())
        if t_time < 0:
            print("traj time is negative. changing it to positive")
            t_time = - t_time
        com_command_msg.time = t_time
        c_height = float(self._widget.text_height.text())
        com_command_msg.height = c_height
        com_command_msg.angle = float(self._widget.text_angle.text())
        idx = self._widget.comboBox.currentIndex()
        com_command_msg.mode = idx
        self._publisher2.publish(com_command_msg)

    def comboBox_select(self):
        idx = self._widget.comboBox.currentIndex()
        if idx == 6:
            self._widget.label_4.setText('Step length')
            self._widget.label_21.setText('Step number')
            self._widget.label_5.setText('Step time')
        elif idx == 7:
            self._widget.label_4.setText('Step length')
            self._widget.label_21.setText('Step number')
            self._widget.label_5.setText('Step time')
        else:
            self._widget.label_4.setText('COM pos')
            self._widget.label_21.setText('angle')
            self._widget.label_5.setText('Traj Time')

    def torqueon_button(self):
        self.send_msg("torqueon")
        self._widget.gravity_button.setEnabled(True)
        self._widget.task_button.setEnabled(True)
        self._widget.contact_button.setEnabled(True)
        self._widget.data_button.setEnabled(True)
        self._widget.msg_box.setText("Ready to Go !")
        self._widget.torqueoff_button.setChecked(False)

    def emergencyoff_button(self):
        self.send_msg("emergencyoff")
        self._widget.msg_box.setText("Turned off with Emergency off")

    def torqueoff_button(self):
        self.send_msg("torqueoff")
        self._widget.msg_box.setText("Sleep mode")
        self._widget.torqueon_button.setChecked(False)

    def data_button(self):
        self.send_msg("data")

    def gravity_button(self):
        self.send_msg("gravity")

    def task_button(self):
        self.send_msg("positioncontrol")

    def contact_button(self):
        self.send_msg("fixedgravity")

    def send_msg(self, msg):
        self._publisher.publish(msg)

    def send_msg2(self, msg):
        self._publisher3.publish(msg)

    def torqueredis_button(self):
        self.send_msg("torqueredis")

    def tune_command_button(self):
        cnt1 = self._widget.tn1.text()
        cnt2 = self._widget.tn2.text()
        cnt3 = self._widget.tn3.text()
        cnt4 = self._widget.tn4.text()
        cnt5 = self._widget.tn5.text()
        cnt6 = self._widget.tn6.text()

        cnt7 = self._widget.tn7.text()
        cnt8 = self._widget.tn8.text()
        cnt9 = self._widget.tn9.text()
        cnt10 = self._widget.tn10.text()
        cnt11 = self._widget.tn11.text()
        cnt12 = self._widget.tn12.text()

        self._widget.tnc1.setText(cnt1)
        self._widget.tnc2.setText(cnt2)
        self._widget.tnc3.setText(cnt3)
        self._widget.tnc4.setText(cnt4)
        self._widget.tnc5.setText(cnt5)
        self._widget.tnc6.setText(cnt6)

        self._widget.tnc1.setText(cnt7)
        self._widget.tnc2.setText(cnt8)
        self._widget.tnc3.setText(cnt9)
        self._widget.tnc4.setText(cnt10)
        self._widget.tnc5.setText(cnt11)
        self._widget.tnc6.setText(cnt12)

        ctotal = cnt1.replace('.', '', 1)+cnt2.replace('.', '', 1)+cnt3.replace('.', '', 1)+cnt4.replace('.', '', 1)+cnt5.replace('.', '', 1)+cnt6.replace('.', '', 1) + \
            cnt7.replace('.', '', 1)+cnt8.replace('.', '', 1)+cnt9.replace('.', '', 1)+cnt10.replace('.', '', 1)+cnt11.replace('.', '', 1)+cnt12.replace('.', '', 1)
        if ctotal.isdigit():
            cntlist = dyros_red_msgs.msg.GainCommand()
            cntlist.gain.append(float(cnt1))
            cntlist.gain.append(float(cnt2))
            cntlist.gain.append(float(cnt3))
            cntlist.gain.append(float(cnt4))
            cntlist.gain.append(float(cnt5))
            cntlist.gain.append(float(cnt6))
            cntlist.gain.append(float(cnt7))
            cntlist.gain.append(float(cnt8))
            cntlist.gain.append(float(cnt9))
            cntlist.gain.append(float(cnt10))
            cntlist.gain.append(float(cnt11))
            cntlist.gain.append(float(cnt12))
            self._publisher4.publish(cntlist)
        else:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText("float warning!")
            msg.setInformativeText("input must be float!")
            msg.setWindowTitle("float warning")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None
        if self._publisher2 is not None:
            self._publisher2.unregister()
            self._publisher2 = None
        if run_mode == "simultaion":
            if self._publisher3 is not None:
                self._publisher3.unregister()
                self._publisher3 = None
        if run_mode == "realrobot":
            if self._publisher4 is not None:
                self._publisher4.unregister()
                self._publisher4 = None

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self._unregister_publisher()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
