#!/usr/bin/env python3
#-*- coding:utf-8 -*- 
import rospy
from tf.transformations import *
from math import tau
import sys
import cv2 as cv

# write your path on here 
sys.path.insert(0, '/home/chan/catkin_ws/src/indy_driver/src')

from indy_driver.msg import flag_data, robot_state
from move_group_python_interface import MoveGroupPythonInterface

class Indy10_Move_With_Camera():
    def __init__(self):

        self.indy10_interface           = MoveGroupPythonInterface(real=True, gripper="Vaccum")

        self.sub_object_info            = rospy.Subscriber("robot_flag", flag_data, self.detection_callback)
        self.sub_robot_state            = rospy.Subscriber("robot_state", robot_state, self.robot_state_callback)


        # init_pose_joints = [0, 0, tau/4, 0, tau/4, 0]          # tau = 2 * pi
        # self.indy10_interface.go_to_joint_state(init_pose_joints)

        deg = tau/360  # For example, entering 90*deg will output 90 degrees.
            
        # Joint 각도 선언
        # self.target_joints_1 = [-10*deg, -11.23*deg, 120*deg,0*deg,-17.7*deg, 0*deg ]     # Flag1
        # self.target_joints_2 = [0*deg, -11.23*deg, 120*deg,0*deg,-17.7*deg, 0*deg ]       # Flag2
        # self.target_joints_3 = [10*deg, -11.23*deg, 120*deg,0*deg,-17.7*deg, 0*deg  ]     # Flag3

        # self.target_joint_4_safe= [0*deg,-18.82*deg,79.01*deg,0*deg,119.15*deg,0*deg]     # @Flag4 --> Move safe loc
        # self.target_joint_4_grip= [0*deg, -31.35*deg,135.52*deg, 0*deg, 64.98*deg, 0*deg] # @Flag4 --> Grip loc
        # self.target_joint_end   = [0, 0, 45*deg, 0, 45*deg, 0] # @Flag4 --> Grip loc

       
        self.target_joints_1 = [-10.0*deg,  -14.18*deg, 121.82*deg,   -1.07*deg,  -16.58*deg,  1.89*deg ]     # Flag2
        self.target_joints_2 = [  0.0*deg,  -14.18*deg, 121.82*deg,   -1.07*deg,  -16.58*deg,  1.89*deg ]     # Flag3
        self.target_joints_3 = [ 10.0*deg,  -14.18*deg, 121.82*deg,   -1.07*deg,  -16.58*deg,  1.89*deg ]       # Flag4
        

        self.target_joint_goal_safe1 = [0.0*deg,-18.60*deg, 66.45*deg, -1.35*deg, 47.18*deg, 0.10*deg]
        self.target_joint_goal_safe2 = [0.0*deg,-28.38*deg, 107.98*deg, -1.30*deg, 101.33*deg, 0.10*deg]# @Flag4 --> Move safe loc
        self.target_joint_goal_grip = [-0.01*deg, -26.51*deg, 132.48*deg, -0.06*deg, 76.38*deg, 1.89*deg] # @Flag4 --> Grip loc
        self.target_joint_end       = [0, 0, 45*deg, 0, 45*deg, 0] # @Flag4 --> Grip loc

        self.flag = 0
        self.pre_flag = 0
        self.vaccum_flag = 0
        self.robot_state = 0
        self.count = 0
  
        print("Initialization is completed!")

    def detection_callback(self, data):
        # print(f"message received: {data.x}, {data.y}")
        self.flag = data.flag

    def robot_state_callback(self, data):

        self.robot_state = data.move


    def run(self):
        self.indy10_interface.grip_off() 
        self.indy10_interface.go_to_joint_state_True(self.target_joints_2)

        while not rospy.is_shutdown():                  # ROS가 종료되지 않은 동안

            if self.pre_flag != self.flag:

                print(f"flag: {self.flag: d}")
                self.pre_flag = self.flag

                if self.flag == 1   :  ## Flag1에 대한 위치로 이동                            
                    self.indy10_interface.go_to_joint_state_False(self.target_joints_1)
                
                elif self.flag == 2 :  ## Flag2에 대한 위치로 이동                            
                    self.indy10_interface.go_to_joint_state_False(self.target_joints_2)
                
                elif self.flag == 3 :  ## Flag3에 대한 위치로 이동                            
                    self.indy10_interface.go_to_joint_state_False(self.target_joints_3)   
                
                elif self.flag == 4:
                    self.indy10_interface.go_to_joint_state_True(self.target_joint_goal_safe1)
                    self.indy10_interface.go_to_joint_state_True(self.target_joint_goal_safe2)# 테이블에 닿지 않는 안전한 위치로 로봇 팔 이동(For Grip)
                    self.indy10_interface.go_to_joint_state_True(self.target_joint_goal_grip) # Grip할 위치로 이동
                    self.indy10_interface.grip_on()  
                    cv.waitKey(500)
                    self.indy10_interface.go_to_joint_state_True(self.target_joint_goal_safe2)
                    self.indy10_interface.go_to_joint_state_True(self.target_joint_goal_safe1)
                    self.indy10_interface.go_to_joint_state_True(self.target_joints_2)
                    self.indy10_interface.grip_off()
                    self.flag == 2 
        
        self.indy10_interface.go_to_joint_state_True(self.target_joint_end)
        

def main():
    try:
        
        Indy10 = Indy10_Move_With_Camera()
        Indy10.run()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()