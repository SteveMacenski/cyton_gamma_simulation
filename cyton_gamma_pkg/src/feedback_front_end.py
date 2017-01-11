#!/usr/bin/env python

import sys
from sensor_msgs.msg import JointState
import copy
import rospy
from std_msgs.msg import Float64
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import PyQt4
import tf
from math import degrees, radians
import rviz
import robot_planning_class

class CommandCenter(PyQt4.QtGui.QMainWindow):

    def __init__(self,parent=None):
        #make window and initialize ROS node
        PyQt4.QtGui.QWidget.__init__(self, parent)
        rospy.init_node('planning_background',
                    anonymous=True)
       
        self.init()
        self.clock = 0
        self.state_subscriber = rospy.Subscriber(
                                  '/joint_states',
                                       JointState,
                            self.CytonStateUpdate)
        self.action = robot_planning_class.CytonMotion()


    def init(self):
        #initilize the window and populate

        PyQt4.QtGui.QToolTip.setFont(PyQt4.QtGui.QFont(
                                        'SansSerif', 10))

        self.init_buttons()     
        self.init_labels()
        self.init_fields()
        
        self.setGeometry(500, 500, 220, 450)
        self.setWindowTitle('Feedback Center')    
        self.show()

    def init_buttons(self):
        #initialize all the buttons and locations

        self.Estop = PyQt4.QtGui.QPushButton('E-STOP', self)
        self.Estop.setStyleSheet("QPushButton {color: red;}")
        self.Estop.setToolTip('Stop robot while executing')
        self.Estop.resize(200,170)
        self.Estop.move(10, 250)  
        self.Estop.clicked.connect(self.stopMotion)


    def init_labels(self):
        #initialize all the labels and locations

        inputXLab = PyQt4.QtGui.QLabel('X',self)
        inputXLab.move(10,20)
        inputXLab.resize(60,30)

        inputYLab = PyQt4.QtGui.QLabel('Y',self)
        inputYLab.move(10,50)
        inputYLab.resize(60,30)

        inputZLab = PyQt4.QtGui.QLabel('Z',self)
        inputZLab.move(10,80)
        inputZLab.resize(60,30)

        inputXLab = PyQt4.QtGui.QLabel('J1',self)
        inputXLab.move(120,20)
        inputXLab.resize(60,30)

        inputYLab = PyQt4.QtGui.QLabel('J2',self)
        inputYLab.move(120,50)
        inputYLab.resize(60,30)

        inputZLab = PyQt4.QtGui.QLabel('J3',self)
        inputZLab.move(120,80)
        inputZLab.resize(60,30)

        inputXLab = PyQt4.QtGui.QLabel('J4',self)
        inputXLab.move(120,110)
        inputXLab.resize(60,30)

        inputYLab = PyQt4.QtGui.QLabel('J5',self)
        inputYLab.move(120,140)
        inputYLab.resize(60,30)

        inputZLab = PyQt4.QtGui.QLabel('J6',self)
        inputZLab.move(120,170)
        inputZLab.resize(60,30)

        inputZLab = PyQt4.QtGui.QLabel('J7',self)
        inputZLab.move(120,200)
        inputZLab.resize(60,30)

        inputRXLab = PyQt4.QtGui.QLabel('RotX',self)
        inputRXLab.move(10,125)
        inputRXLab.resize(60,20)

        inputRYLab = PyQt4.QtGui.QLabel('RotY',self)
        inputRYLab.move(10,150)
        inputRYLab.resize(60,30)

        inputRZLab = PyQt4.QtGui.QLabel('RotZ',self)
        inputRZLab.move(10,180)
        inputRZLab.resize(60,30)


    def init_fields(self):
        #initialize all the fields and locations

        self.currentR1 = PyQt4.QtGui.QLineEdit(self)
        self.currentR1.move(145,20)
        self.currentR1.resize(60,30)

        self.currentR2 = PyQt4.QtGui.QLineEdit(self)
        self.currentR2.move(145,50)
        self.currentR2.resize(60,30)

        self.currentR3 = PyQt4.QtGui.QLineEdit(self)
        self.currentR3.move(145,80)
        self.currentR3.resize(60,30)

        self.currentR4 = PyQt4.QtGui.QLineEdit(self)
        self.currentR4.move(145,110)
        self.currentR4.resize(60,30)

        self.currentR5 = PyQt4.QtGui.QLineEdit(self)
        self.currentR5.move(145,140)
        self.currentR5.resize(60,30)

        self.currentR6 = PyQt4.QtGui.QLineEdit(self)
        self.currentR6.move(145,170)
        self.currentR6.resize(60,30)

        self.currentR7 = PyQt4.QtGui.QLineEdit(self)
        self.currentR7.move(145,200)
        self.currentR7.resize(60,30)

        self.currentX = PyQt4.QtGui.QLineEdit(self)
        self.currentX.move(50,20)
        self.currentX.resize(60,30)

        self.currentY = PyQt4.QtGui.QLineEdit(self)
        self.currentY.move(50,50)
        self.currentY.resize(60,30)

        self.currentZ = PyQt4.QtGui.QLineEdit(self)
        self.currentZ.move(50,80)
        self.currentZ.resize(60,30)

        self.currentRotX = PyQt4.QtGui.QLineEdit(self)
        self.currentRotX.move(50,120)
        self.currentRotX.resize(60,30)

        self.currentRotY = PyQt4.QtGui.QLineEdit(self)
        self.currentRotY.move(50,150)
        self.currentRotY.resize(60,30)

        self.currentRotZ = PyQt4.QtGui.QLineEdit(self)
        self.currentRotZ.move(50,180)
        self.currentRotZ.resize(60,30)


    def CytonStateUpdate(self, JState):
        #update current robot state
        states = JState.position
        self.currentR1.setText(str(round(degrees(states[4]),2)))
        self.currentR2.setText(str(round(degrees(states[0]),2)))
        self.currentR3.setText(str(round(degrees(states[6]),2)))
        self.currentR4.setText(str(round(degrees(states[5]),2)))
        self.currentR5.setText(str(round(degrees(states[3]),2)))
        self.currentR6.setText(str(round(degrees(states[1]),2)))
        self.currentR7.setText(str(round(degrees(states[2]),2)))

        self.clock+=1

        if self.clock % 1 == 0:
            pose = self.action.group.get_current_pose().pose

            self.currentX.setText(str(round(pose.position.x,2)))
            self.currentY.setText(str(round(pose.position.y,2)))
            self.currentZ.setText(str(round(pose.position.z,2)))

            quaternion = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
            Euler = tf.transformations.euler_from_quaternion(
                                               quaternion)
            self.currentRotX.setText(str(round(
                                     degrees(Euler[0]),2)))
            self.currentRotY.setText(str(round(
                                     degrees(Euler[1]),2)))
            self.currentRotZ.setText(str(round(
                                     degrees(Euler[2]),2)))



    def stopMotion(self):
        self.action.stopMotion()


if __name__ == "__main__":
    app = PyQt4.QtGui.QApplication(sys.argv)
    myapp = CommandCenter()
    myapp.show()
    sys.exit(app.exec_())
