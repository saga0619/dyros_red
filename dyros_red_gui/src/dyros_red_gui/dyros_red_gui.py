import os
import rospy
import rospkg

import geometry_msgs.msg
import dyros_red_msgs.msg
from std_msgs.msg import String
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot

from python_qt_binding.QtWidgets import QWidget

class DyrosRedGuiPlugin(Plugin):

    def __init__(self, context):
        super(DyrosRedGuiPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DyrosRedGUI')
        self._publisher = rospy.Publisher('/dyros_red/command', String, queue_size=10)
        self._publisher2 = rospy.Publisher('/dyros_red/com_command', dyros_red_msgs.msg.ComCommand, queue_size=10)
        self._publisher3 = rospy.Publisher('/mujoco_ros_interface/sim_command_con2sim', String, queue_size=10)


        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('dyros_red_gui'), 'resource', 'DyrosRed.ui')
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
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self._widget.gravity_button.pressed.connect(self.gravity_button)
        self._widget.task_button.pressed.connect(self.task_button)
        self._widget.contact_button.pressed.connect(self.contact_button)
        self._widget.data_button.pressed.connect(self.data_button)
        self._widget.com_send_button.pressed.connect(self.com_command_sender)
        self._widget.pause_button.pressed.connect(self.pause_button)
        self._widget.slowmotion_button.pressed.connect(self.slowmotion_button)
        self._widget.reset_button.pressed.connect(self.reset_button)


        self.sub = rospy.Subscriber('/dyros_red/point',geometry_msgs.msg.PolygonStamped, self.sub_cb)

    def sub_cb(self, msg):
        self._widget.label.setText(str(round(msg.polygon.points[0].x,6)) )
        self._widget.label_2.setText(str(round(msg.polygon.points[0].y,6)) )
        self._widget.label_3.setText(str(round(msg.polygon.points[0].z,6)) )

    def pause_button(self):
        self.send_msg2("pause")

    def slowmotion_button(self):
        self.send_msg2("mjslowmotion")

    def reset_button(self):
        self.send_msg2("mjreset")


    def com_command_sender(self):
        if self._widget.text_pos.text().replace('.','',1).isdigit() and self._widget.text_time.text().replace('.','',1).isdigit():
            print("Sending COM command mgs")
            com_command_msg = dyros_red_msgs.msg.ComCommand()
            c_pos = float(self._widget.text_pos.text())
            if c_pos < 0 and c_pos > 1:
                print("com pos value must between 0 ~ 1")
            list = [0, c_pos, 1]
            list.sort()
            com_command_msg.ratio = list[1]
            t_time = float(self._widget.text_time.text())
            if t_time < 0:
                print("traj time is negative. changing it to positive")
                t_time = - t_time
            com_command_msg.time = t_time
            self._publisher2.publish(com_command_msg)
        else:
            print("Commands need to be float and positive ! ")

    def data_button(self):
        self.send_msg("data")

    def gravity_button(self):
        self.send_msg("gravity")

    def task_button(self):
        self.send_msg("task")

    def contact_button(self):
        self.send_msg("contact")

    def send_msg(self, msg):
        self._publisher.publish(msg)

    def send_msg2(self, msg):
        self._publisher3.publish(msg)

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None
        if self._publisher2 is not None:
            self._publisher2.unregister()
            self._publisher2 = None

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

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
