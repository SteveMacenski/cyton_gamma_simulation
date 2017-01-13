#!/usr/bin/env python
from __future__ import division 
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
import threading
import os

class feedback(PyQt4.QtCore.QThread):

    state_info = PyQt4.QtCore.pyqtSignal(list, name="state_info")

    def __init__(self,this):
        PyQt4.QtCore.QThread.__init__(self)

        self.this = this

        self.state_subscriber = rospy.Subscriber(
                                  '/joint_states',
                                       JointState,
                            self.CytonStateUpdate)

    def CytonStateUpdate(self, JState):
        threading.Thread(target=self._CytonStateUpdate, 
                                args=(JState,)).start()


    def _CytonStateUpdate(self, JState):
        #update current robot state
        states = JState.position

        pose = self.this.action.group.get_current_pose().pose

        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
        Euler = tf.transformations.euler_from_quaternion(
                                            quaternion)

        self.state_info.emit([states, pose, Euler])


class CommandCenter(PyQt4.QtGui.QMainWindow):

    def __init__(self,parent=None):

        # get input for specific robot
        if len(sys.argv) > 1:
            if sys.argv[1] == '300' or sys.argv[1] == '1500':
                self.rob = sys.argv[1]
            else:
                print "Not a robot, please specify 300 or 1500"
                os_exit(1)
        else:
            print "No Robot specified, please specify 300 or 1500"
            os._exit(1)

        #make window and initialize ROS node
        PyQt4.QtGui.QWidget.__init__(self, parent)
        rospy.init_node('front_endBackground',
                    anonymous=True)
       
        self.init()
        self.clock = 0

        self.action = robot_planning_class.CytonMotion()

        t = feedback(self)
        t.state_info.connect(self.state_writer)
        t.start()
        

    def init(self):
        #initilize the window and populate

        PyQt4.QtGui.QToolTip.setFont(PyQt4.QtGui.QFont(
                                        'SansSerif', 10))

        self.init_buttons()     
        self.init_labels()
        self.init_fields()
        
        self.setGeometry(500, 500, 450, 500)
        self.setWindowTitle('Command Center')    
        self.show()


    def init_buttons(self):
        #initialize all the buttons and locations

        self.commandButJ = PyQt4.QtGui.QPushButton(
                                  'CommandJ', self)
        self.commandButJ.setToolTip('Command trajectory')
        self.commandButJ.resize(self.commandButJ.sizeHint())
        self.commandButJ.move(120, 240) 
        self.commandButJ.clicked.connect(self.commandJF)
        
        self.planButtonJ = PyQt4.QtGui.QPushButton('PlanJ', self)
        self.planButtonJ.setToolTip('Plan Trajectory')
        self.planButtonJ.resize(self.commandButJ.sizeHint())
        self.planButtonJ.move(20, 240)  
        self.planButtonJ.clicked.connect(self.planJF)

        self.commandButL = PyQt4.QtGui.QPushButton(
                                   'CommandL', self)
        self.commandButL.setToolTip('Command trajectory')
        self.commandButL.resize(self.commandButL.sizeHint())
        self.commandButL.move(330, 240) 
        self.commandButL.clicked.connect(self.commandLF)

        self.planButtonL = PyQt4.QtGui.QPushButton('PlanL', self)
        self.planButtonL.setToolTip('Plan Trajectory')
        self.planButtonL.resize(self.commandButL.sizeHint())
        self.planButtonL.move(235, 240)  
        self.planButtonL.clicked.connect(self.planLF)

        self.Gripbutton = PyQt4.QtGui.QPushButton('Gripper\n[0 100]', self)

        self.Gripbutton.setToolTip('Close Grippers')
        self.Gripbutton.resize(90,80)
        self.Gripbutton.move(20, 380)  
        self.Gripbutton.clicked.connect(self.moveGripper)

        self.deltaBox = PyQt4.QtGui.QCheckBox(self)
        self.deltaBox.setToolTip("Indicate Delta Moves")
        self.deltaBox.move(100,300)
        self.deltaBox.resize(20,20)

        self.Estop = PyQt4.QtGui.QPushButton('E-STOP', self)
        self.Estop.setStyleSheet("QPushButton {color: red;}")
        self.Estop.setToolTip('Stop robot while executing')
        self.Estop.resize(200,180)
        self.Estop.move(230, 280)  
        self.Estop.clicked.connect(self.stopMotion)

        self.vScale = PyQt4.QtGui.QPushButton('Vel. Scale\n[0.1 5]', self)
        self.vScale.setToolTip('Choose % default speed to move')
        self.vScale.resize(90,80)
        self.vScale.move(120, 380)  
        self.vScale.clicked.connect(self.velScale)


    def init_labels(self):
        #initialize all the labels and locations
        LinearLab = PyQt4.QtGui.QLabel('Delta Moves',self)
        LinearLab.move(70,275)


        LinearLab = PyQt4.QtGui.QLabel(' Linear',self)
        LinearLab.move(265,0)
        LinearLab.resize(60,20)

        JointLab = PyQt4.QtGui.QLabel('    Joint',self)
        JointLab.move(50,0)
        JointLab.resize(80,20)

        JointLab = PyQt4.QtGui.QLabel('   Current',self)
        JointLab.move(340,0)
        JointLab.resize(60,20)

        JointLab = PyQt4.QtGui.QLabel('  Current',self)
        JointLab.move(140,0)
        JointLab.resize(60,20)

        inputXLab = PyQt4.QtGui.QLabel('X',self)
        inputXLab.move(145+90,20)
        inputXLab.resize(60,30)

        inputYLab = PyQt4.QtGui.QLabel('Y',self)
        inputYLab.move(145+90,50)
        inputYLab.resize(60,30)

        inputZLab = PyQt4.QtGui.QLabel('Z',self)
        inputZLab.move(145+90,80)
        inputZLab.resize(60,30)

        inputXLab = PyQt4.QtGui.QLabel('J1',self)
        inputXLab.move(20,20)
        inputXLab.resize(60,30)

        inputYLab = PyQt4.QtGui.QLabel('J2',self)
        inputYLab.move(20,50)
        inputYLab.resize(60,30)

        inputZLab = PyQt4.QtGui.QLabel('J3',self)
        inputZLab.move(20,80)
        inputZLab.resize(60,30)

        inputXLab = PyQt4.QtGui.QLabel('J4',self)
        inputXLab.move(20,110)
        inputXLab.resize(60,30)

        inputYLab = PyQt4.QtGui.QLabel('J5',self)
        inputYLab.move(20,140)
        inputYLab.resize(60,30)

        inputZLab = PyQt4.QtGui.QLabel('J6',self)
        inputZLab.move(20,170)
        inputZLab.resize(60,30)

        inputZLab = PyQt4.QtGui.QLabel('J7',self)
        inputZLab.move(20,200)
        inputZLab.resize(60,30)

        inputRXLab = PyQt4.QtGui.QLabel('RotX',self)
        inputRXLab.move(145+75,125)
        inputRXLab.resize(60,20)

        inputRYLab = PyQt4.QtGui.QLabel('RotY',self)
        inputRYLab.move(145+75,150)
        inputRYLab.resize(60,30)

        inputRZLab = PyQt4.QtGui.QLabel('RotZ',self)
        inputRZLab.move(145+75,180)
        inputRZLab.resize(60,30)

    def paintEvent(self, event):

        qp = PyQt4.QtGui.QPainter()
        qp.begin(self)
        qp.setPen(PyQt4.QtGui.QColor(PyQt4.QtCore.Qt.black))
        qp.drawRect(65,273,94,60)
        qp.end()


    def init_fields(self):
        #initialize all the fields and locations
        self.inputX = PyQt4.QtGui.QLineEdit(self)
        self.inputX.move(145+90+25,20)
        self.inputX.resize(60,30)

        self.inputY = PyQt4.QtGui.QLineEdit(self)
        self.inputY.move(145+90+25,50)
        self.inputY.resize(60,30)

        self.inputZ = PyQt4.QtGui.QLineEdit(self)
        self.inputZ.move(145+90+25,80)
        self.inputZ.resize(60,30)

        self.inputRotX = PyQt4.QtGui.QLineEdit(self)
        self.inputRotX.move(145+90+25,120)
        self.inputRotX.resize(60,30)

        self.inputRotY = PyQt4.QtGui.QLineEdit(self)
        self.inputRotY.move(145+90+25,150)
        self.inputRotY.resize(60,30)

        self.inputRotZ = PyQt4.QtGui.QLineEdit(self)
        self.inputRotZ.move(145+90+25,180)
        self.inputRotZ.resize(60,30)

        self.inputR1 = PyQt4.QtGui.QLineEdit(self)
        self.inputR1.move(50,20)
        self.inputR1.resize(60,30)

        self.inputR2 = PyQt4.QtGui.QLineEdit(self)
        self.inputR2.move(50,50)
        self.inputR2.resize(60,30)

        self.inputR3 = PyQt4.QtGui.QLineEdit(self)
        self.inputR3.move(50,80)
        self.inputR3.resize(60,30)

        self.inputR4 = PyQt4.QtGui.QLineEdit(self)
        self.inputR4.move(50,110)
        self.inputR4.resize(60,30)

        self.inputR5 = PyQt4.QtGui.QLineEdit(self)
        self.inputR5.move(50,140)
        self.inputR5.resize(60,30)

        self.inputR6 = PyQt4.QtGui.QLineEdit(self)
        self.inputR6.move(50,170)
        self.inputR6.resize(60,30)

        self.inputR7 = PyQt4.QtGui.QLineEdit(self)
        self.inputR7.move(50,200)
        self.inputR7.resize(60,30)

        self.inputRG = PyQt4.QtGui.QLineEdit(self)
        self.inputRG.move(37,340)
        self.inputRG.resize(60,30)

        self.currentR1 = PyQt4.QtGui.QLineEdit(self)
        self.currentR1.move(140,20)
        self.currentR1.resize(60,30)

        self.currentR2 = PyQt4.QtGui.QLineEdit(self)
        self.currentR2.move(140,50)
        self.currentR2.resize(60,30)

        self.currentR3 = PyQt4.QtGui.QLineEdit(self)
        self.currentR3.move(140,80)
        self.currentR3.resize(60,30)

        self.currentR4 = PyQt4.QtGui.QLineEdit(self)
        self.currentR4.move(140,110)
        self.currentR4.resize(60,30)

        self.currentR5 = PyQt4.QtGui.QLineEdit(self)
        self.currentR5.move(140,140)
        self.currentR5.resize(60,30)

        self.currentR6 = PyQt4.QtGui.QLineEdit(self)
        self.currentR6.move(140,170)
        self.currentR6.resize(60,30)

        self.currentR7 = PyQt4.QtGui.QLineEdit(self)
        self.currentR7.move(140,200)
        self.currentR7.resize(60,30)

        self.currentX = PyQt4.QtGui.QLineEdit(self)
        self.currentX.move(50+90+90+90+25,20)
        self.currentX.resize(60,30)

        self.currentY = PyQt4.QtGui.QLineEdit(self)
        self.currentY.move(50+90+90+90+25,50)
        self.currentY.resize(60,30)

        self.currentZ = PyQt4.QtGui.QLineEdit(self)
        self.currentZ.move(50+90+90+90+25,80)
        self.currentZ.resize(60,30)

        self.currentRotX = PyQt4.QtGui.QLineEdit(self)
        self.currentRotX.move(50+90+90+90+25,120)
        self.currentRotX.resize(60,30)

        self.currentRotY = PyQt4.QtGui.QLineEdit(self)
        self.currentRotY.move(50+90+90+90+25,150)
        self.currentRotY.resize(60,30)

        self.currentRotZ = PyQt4.QtGui.QLineEdit(self)
        self.currentRotZ.move(50+90+90+90+25,180)
        self.currentRotZ.resize(60,30)

        self.velocityScale = PyQt4.QtGui.QLineEdit(self)
        self.velocityScale.move(133,340)
        self.velocityScale.resize(60,30)


    def commandJF(self):
        threading.Thread(target=self._commandJF,
                       name="commandJF").start()


    def _commandJF(self):
        #send command to the joint task in the planning class
        J1 = radians(float(self.inputR1.text()))
        J2 = radians(float(self.inputR2.text()))
        J3 = radians(float(self.inputR3.text()))
        J4 = radians(float(self.inputR4.text()))
        J5 = radians(float(self.inputR5.text()))
        J6 = radians(float(self.inputR6.text()))
        J7 = radians(float(self.inputR7.text()))

        if self.deltaBox.isChecked():
            self.action.deltaMoveJoint(
           [J1,J2,J3,J4,J5,J6,J7],True)
        else:
            self.action.moveJoint([J1,J2,J3,J4,J5,J6,J7],True)



    def planJF(self):
        threading.Thread(target=self._planJF, 
                       name="planJF").start()


    def _planJF(self):
        #send plan request to the joint task in the planning class
        J1 = radians(float(self.inputR1.text()))
        J2 = radians(float(self.inputR2.text()))
        J3 = radians(float(self.inputR3.text()))
        J4 = radians(float(self.inputR4.text()))
        J5 = radians(float(self.inputR5.text()))
        J6 = radians(float(self.inputR6.text()))
        J7 = radians(float(self.inputR7.text()))

        if self.deltaBox.isChecked():
            self.action.deltaMoveJoint([J1,J2,J3,J4,J5,J6,J7],
                                                        False)
        else:
            self.action.moveJoint([J1,J2,J3,J4,J5,J6,J7],
                                                        False)


    def commandLF(self):
        threading.Thread(target=self._commandLF,
                       name="commandLF").start()


    def _commandLF(self):
        #send command to the linear task in the planning class
        x = float(self.inputX.text())
        y = float(self.inputY.text())
        z = float(self.inputZ.text())
        Ex = radians(float(self.inputRotX.text()))
        Ey = radians(float(self.inputRotY.text()))
        Ez = radians(float(self.inputRotZ.text()))

        if self.deltaBox.isChecked():
            self.action.deltaMoveCartesian([Ex,Ey,Ez],[x,y,z],True)
        else:
            self.action.moveCartesian([Ex,Ey,Ez],[x,y,z],True)


    def planLF(self):
        threading.Thread(target=self._planLF,
                       name="planLF").start()


    def _planLF(self):
        #send planning request to the linear task in the planning class
        x = float(self.inputX.text())
        y = float(self.inputY.text())
        z = float(self.inputZ.text())
        Ex = radians(float(self.inputRotX.text()))
        Ey = radians(float(self.inputRotY.text()))
        Ez = radians(float(self.inputRotZ.text()))

        if self.deltaBox.isChecked():
            self.action.deltaMoveCartesian([Ex,Ey,Ez],
                                                 [x,y,z],False)
        else:
            self.action.moveCartesian([Ex,Ey,Ez],[x,y,z],False)


    def moveGripper(self):
        threading.Thread(target=self._moveGripper,
                       name="moveGripper").start()


    def _moveGripper(self):
        #send command to the gripper task in the hardware
        move = float(self.inputRG.text())

        if self.rob == '1500':
            move = ((move/100.)*2.4)-.5
        else:
            move = ((move/100.)*1.3)-.6

        self.action.moveGripper(move)


    def state_writer(self, info):
        states = info[0]
        pose = info[1]
        Euler = info[2]

        if self.rob == '1500':
            #ordering of joints recieved for 300 and 1500

            self.currentR1.setText(str(round(degrees(states[4]),2)))
            self.currentR2.setText(str(round(degrees(states[0]),2)))
            self.currentR3.setText(str(round(degrees(states[6]),2)))
            self.currentR4.setText(str(round(degrees(states[5]),2)))
            self.currentR5.setText(str(round(degrees(states[3]),2)))
            self.currentR6.setText(str(round(degrees(states[1]),2)))
            self.currentR7.setText(str(round(degrees(states[2]),2)))

        else:
  
            self.currentR1.setText(str(round(degrees(states[5]),2)))
            self.currentR2.setText(str(round(degrees(states[0]),2)))
            self.currentR3.setText(str(round(degrees(states[4]),2)))
            self.currentR4.setText(str(round(degrees(states[6]),2)))
            self.currentR5.setText(str(round(degrees(states[3]),2)))
            self.currentR6.setText(str(round(degrees(states[1]),2)))
            self.currentR7.setText(str(round(degrees(states[2]),2)))

        self.currentX.setText(str(round(pose.position.x,2)))
        self.currentY.setText(str(round(pose.position.y,2)))
        self.currentZ.setText(str(round(pose.position.z,2)))

        self.currentRotX.setText(str(round(
                                 degrees(Euler[0]),2)))
        self.currentRotY.setText(str(round(
                                 degrees(Euler[1]),2)))
        self.currentRotZ.setText(str(round(
                                 degrees(Euler[2]),2)))


    def stopMotion(self):
        threading.Thread(target=self._stopMotion,
                        name="stopMotion").start()

    def _stopMotion(self):
        self.action.stopMotion()

    def velScale(self):
    # change the velocity scale for the robot's motion
        scale = float(self.velocityScale.text())
        if scale > 5:
            scale = 5
        if scale < 0.1:
            scale = 0.1

        self.action.changeVelocityScaling(scale)


    def closeEvent(self, event):
        #close all threads cleanly
        os._exit(1)


if __name__ == "__main__":
    app = PyQt4.QtGui.QApplication(sys.argv)
    myapp = CommandCenter()
    myapp.show()
    sys.exit(app.exec_())
