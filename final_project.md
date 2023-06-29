# Byung-Ji Robot (Air-Hockey penalty kick)

**Instructor :** prof. Young-Keun Kim

**Team member :** Dong-min Kim, Eun-chan Kim, Seung-eun Hwang

**Date :** Spring semester, 2023

**Part :** Automation part in Industrial AI & Automation class

**Project :** #2

-----

## 1. Introduction

This Repository includes a **Tutorial** for the final project of the Industrial AI & Automation course conducted in the first semester of 2023 at Handong Global University, namely, the **Robot Byeong-Gi (Airhockey Robot)**.

The Automation project is designed to gain experience in controlling a robotic arm by configuring a scenario where the Indy-10(Neuromeka)'s 6-axis collaborative robotic arm performs a specific task, and aims to promote the Department of Mechanical Control Engineering by providing a positive experience through robot experiences for lower-grade students.

We have selected the project theme with a focus on **"participation"** and **"feasibility"**. We judged that an **Air Hockey Robot** would allow participants to easily play the game and could be implemented with a collaborative robot.

The primary objective was set to predict the moving path of an object using image processing techniques, and move the robotic arm to the predicted location to block the object.

## 2. Requirements

### 2.1. Software

- OS: ubuntu 20.04 version

- ROS noetic

- python 3.9
- opencv 4.7.0

### 2.2. Hardware

- Camera  sensor : Logitech Brio 4K (EA : 1)

<img src="https://github.com/DongminKim21800064/IAIA_Project2_RobotSonny/assets/91474647/4fba1e99-0299-42ec-b9b3-9479097881b8" alt="image" style="zoom: 50%;" />

<center><strong>Figure 1. Logitech Brio 4K</strong></center>

- Robot arm : indy 10 (EA : 1)

<img src="https://github.com/DongminKim21800064/IAIA_Project2_RobotSonny/assets/91474647/db905399-5634-4a32-9872-03d43c2cc7a8" alt="image" style="zoom: 33%;" />

<center><strong>Figure 2. Indy 10 Robot Arm</strong></center>

- Specifications of Indy 10 robot
  - 6 degree of freedom
  - 10 [kg] payload
  - 175 deg
  - joint 1,2 : 60 deg/s, joint 3,4,5,6 : 90 deg/s
  - Maximum reach : 1000mm

- Air hockey peddle

3D modeling was conducted using Fusion 360. The size of the robot arm's bracket was measured in real world, and modeling was done accordingly to create the handle.

<img src="https://github.com/DongminKim21800064/IAIA_Project2_RobotSonny/assets/91419683/556c4dad-de95-46b4-a35b-26ed7ac3c39c" alt="image" style="zoom: 67%;" />

<center><strong>Figure 3. Air Hockey Peddle 3D Model</strong></center>

[3D Model Download link](https://a360.co/3CPhVSU)

- AirHockey Table

We purchased and used an air hockey table in the 300,000 won range. A table appropriate for the height of Indy 10 was selected. Considering the speed of the robot, we purchased a product with a vertical length of more than 150cm.

<img src="https://github.com/DongminKim21800064/IAIA_Project2_RobotSonny/assets/91419683/4e3ffd42-2178-48a4-9c35-9f479365e358" alt="image" style="zoom: 67%;" />

<center><strong>Figure 4. Air Hockey Table</strong></center>

[Purchase link](https://www.11st.co.kr/products/5729217404?NaPm=ct=lhk3yb9k|ci=01f4068eac4029dc1c366ec30dee720cb879cad5|tr=slct|sn=17703|hk=6d9d370784625290fee06e082f7238cd3c822bf0&utm_term=&utm_campaign=%B3%D7%C0%CC%B9%F6pc_%B0%A1%B0%DD%BA%F1%B1%B3%B1%E2%BA%BB&utm_source=%B3%D7%C0%CC%B9%F6_PC_PCS&utm_medium=%B0%A1%B0%DD%BA%F1%B1%B3)

- Vacuum gripper

<img src="https://github.com/DongminKim21800064/IAIA_Project2_RobotSonny/assets/91474647/61b24f2d-f939-4d01-a2ca-20f4eed9da9a" alt="image" style="zoom: 50%;" />

<center><strong>Figure 5. Vacuum Gripper</strong></center>



## 3. **Process**

### 3.1. Process structure

 The structure of the program consists of two parts: image processing and robot operation. In the camera, segmentation was carried out using HSV, and the expected path was calculated based on the position change according to the frame change of the detected object. The average slope of the object's position change was used to predict the angle of incidence and reflection of the Puck. The robot area is divided into three parts: left, center, and right. Depending on which area the end point of the predicted path is located, a movement Flag is passed to the robot. If an object crosses the goal line and is not detected for a certain number of frames, it is judged that a goal has been scored, and a Flag for the robot to pick up the Puck is passed.

 Through the Flag received from the camera part, the robot moves to the expected position where the Puck will arrive. The positions of the three areas were pre-set through the robot's absolute coordinates, and the action of picking up the Puck was also set using absolute coordinates. By repeating this process, the code was constructed to sustain the game of sudden death.



### 3.2. System flow chart

<img src="https://github.com/DongminKim21800064/IAIA_Project2_RobotSonny/assets/91474647/72a88420-0398-4e77-be71-06e181436dbf" alt="image" style="zoom: 33%;" />

<center><strong>Figure 6. Flow Chart</strong></center>

### 3.3. System process

<img src="https://github.com/DongminKim21800064/IAIA_Project2_RobotSonny/assets/91474647/704f67b4-ded5-4567-948b-3e47e3565c40" alt="image" style="zoom: 33%;" />

- The camera continuously calculates the segmentation information of the puck.
- The destination the robot should go to is transmitted via a publisher flag.
- When the robot receives location information via a subscriber, the robot moves.
- When the robot moves, once it receives a flag indicating the movement is complete, the robot is ready to receive the next location.

### 3.4. Robot manipulation

![image](https://github.com/DongminKim21800064/IAIA_Project2_RobotSonny/assets/91474647/adc91cd9-eb17-4d6c-85a3-9771d0d5a60d)

<center><strong>Figure 7. Robot Axis</strong></center>

 There are three types of movements for the robot: absolute coordinate commands (x, y, z, roll, pitch, yaw), relative coordinate commands (x, y, z, roll, pitch, yaw), and joint commands (limit radian for each joint) which were used in our experiment.

 A problem with using absolute and relative coordinates is that ROS calculates the path when moving along a single path. At this point, instead of taking the optimal path, it tends to slow down by using all joints.

 Therefore, in our robot where speed is important, we only utilized the rotation of the z-axis joint on the 1-axis side and used the minimum number of axes. The reason for using the minimum number of axes is because all joint movements do not occur simultaneously, but move in series, so only one axis was used.

 Also, for quick movements, we set the stopping part in the go_to_joint_state function to False to move with the minimum delay when the next command comes in.

 However, after the goal, which is flag 4, the stop had to be set to True, so different function commands were used.

- path planning 

```python
self.target_joints_1 = [-10.0*deg,  -14.18*deg, 121.82*deg,   -1.07*deg,  -16.58*deg,  1.89*deg ]     # Flag2
self.target_joints_2 = [  0.0*deg,  -14.18*deg, 121.82*deg,   -1.07*deg,  -16.58*deg,  1.89*deg ]     # Flag3
self.target_joints_3 = [ 10.0*deg,  -14.18*deg, 121.82*deg,   -1.07*deg,  -16.58*deg,  1.89*deg ]       # Flag4
        

self.target_joint_goal_safe1 = [0.0*deg,-18.60*deg, 66.45*deg, -1.35*deg, 47.18*deg, 0.10*deg]
self.target_joint_goal_safe2 = [0.0*deg,-28.38*deg, 107.98*deg, -1.30*deg, 101.33*deg, 0.10*deg]# @Flag4 --> Move safe loc
self.target_joint_goal_grip = [-0.01*deg, -26.51*deg, 132.48*deg, -0.06*deg, 76.38*deg, 1.89*deg] # @Flag4 --> Grip loc
self.target_joint_end       = [0, 0, 45*deg, 0, 45*deg, 0] # @Flag4 --> Grip loc
```



- robot manipulation

```python
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
```



### 3.5. Robot speed control

Due to Indy 10 being a cooperative robot, the speed of the robot's actuator is slow. Therefore, speed configuration is required. As the speed function within ROS does not allow for changes in speed, we referred to the corresponding code in the documentation of the robot manufacturer, Neuromeka, on their website.

[neuromeka docs](http://docs.neuromeka.com/3.0.0/kr/Python/section1/)

The robot's speed can be set from 1 to 9. We set it to the fastest speed, and after setting it to 9, we were able to proceed with a simple game.



**indy_set_velocity.py**

```python
#!/usr/bin/env python3
#-*- coding:utf-8 -*- 

import indydcp_client as client
import copy


def main():
    robot_ip = "192.168.0.8"    # 예시 STEP IP 주소
    robot_name = "NRMK-Indy10"   # IndyRP2의 경우 "NRMK-IndyRP2"
    indy = client.IndyDCPClient(robot_ip, robot_name) # indy 객체 생성

    indy.connect() # 로봇 연결
    vel  : int   = 9 # 1 ~ 9

    print(f'setting velocity: {vel:.2f}')
    # Setting 
    indy.set_joint_vel_level(vel)     # 1 ~ 9
    indy.set_task_vel_level(vel)     # 1 ~ 9

    indy.disconnect() # 연결 해제

if __name__ == '__main__':
    try:
        main()

    except Exception as e:
        print("[ERROR]", e)
```



## 4. Result (Demo video)

Demo Video : [Click here](https://www.youtube.com/watch?v=lsEivK4yrS4)

<img src="https://github.com/HanMinung/Robotarm_Automation/assets/91367451/4e2c83c3-3102-46e3-912f-824f06262b65" alt="image" style="zoom:67%;" />

<center><strong>Figure 8. Example of operation</strong></center>

<strong>Table 1. Attempt Result</strong>

| Attempt | 1    | 2    | 3    | 4    | 5    | 6    | 7    | 8    | 9    | 10   | 11   | 12   | 13   | 14   |
| ------- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| Success | O    | O    | O    | O    | O    | O    | O    | O    | O    | X    | O    | O    | O    | O    |

 As can be seen in the video above, it showed a successful defense result in 13 out of 14 attempts. The Flag 4 command, an action to pick up the Puck, was also confirmed to be operating normally.

**Accuracy : 92.85%** 



## 5. Multi process shell script management

This project involves the inconvenience of running multiple files simultaneously. Even if it is a single program, difficulties arise in the progression of the program if the order is mixed up or if certain files are not executed. Therefore, the aim is to resolve this inconvenience by executing only one shell script file.

- Things to do before running the file:
  - Place the sh file in the **catkin_ws** folder.
  - Type "chmod +x airhockey.sh" in the terminal.

- Shell script execute 

```bash
# Run the Python script

source devel/setup.bash
python3 /home/"user name"/catkin_ws/src/indy_driver/src/indy_set_velocity.py

# Define a function to run a command in a new Terminator window and arrange it
function run_in_terminator {
    local command=$1
    # Use Terminator's --geometry option to specify the window position and size (format: WIDTHxHEIGHT+X+Y)
    # Adjust values as needed
    terminator --geometry=800x600+0+0 -e "$command" &
    sleep 5
}

# Run the commands
run_in_terminator "bash -c 'source devel/setup.bash; roslaunch indy10_moveit_config moveit_planning_execution.launch robot_ip:=192.168.0.8; exec bash'"
run_in_terminator "bash -c 'source devel/setup.bash; rosrun indy_driver camera.py; exec bash'"
run_in_terminator "bash -c 'source devel/setup.bash; rosrun indy_driver move_robot.py; exec bash'"

# Wait for user input to close all windows
read -p "Press Enter to close all Terminator windows"

# Close all Terminator windows
killall terminator
```



And execute the following in the terminal window.

```bash
cd catkin_ws
./airhockey.sh
```

<img src="https://github.com/DongminKim21800064/IAIA_Project2_RobotSonny/assets/91474647/561a726c-e7da-42c0-a24b-4ff06e609197" alt="image" style="zoom:67%;" />

<center><strong>Figure 9. Program Excute</strong></center>

A total of four windows and a camera window will be opened. The provided image shows the two important terminals and the displayed camera.



## 6. Discussion and analysis

1. **Issues with Robots and ROS**

There are several issues and limitations in the progression of this project. One of the most significant problems and limitations is that despite the robot's speed being set at its maximum, it showed inadequacy in blocking the Puck. As a result, the initial plan of continuing the rally had to be altered to a game of sudden death, and instead of real-time path prediction, it was necessary to derive the predicted path through the average slope within 5 frames after hitting the Puck. Consequently, discrepancies between the anticipated and actual path of the Puck occurred. In order to resolve this, it is deemed appropriate to use a robot capable of faster action.

Furthermore, when controlling the robot using ROS, an issue arose where the robot's movements slowed down as time elapsed, causing delayed reactions to input commands. Testing showed that the robot takes an additional second to process each multiple of ten commands. This suggests a delay in processing speed over time in ROS, as the same issue occurs in other robots of the same model.

2. **Issues with Camera Perception and Path Prediction**

The predicted path was linearly assumed by setting the angle of incidence and reflection with the slope according to the change in the position of the Puck. However, it was confirmed that the actual movement of the Puck was nonlinear due to the wind coming from the floor of the game court and friction. Potential solutions to the Puck's nonlinear movement include using a faster-acting robot to accommodate real-time path prediction, or using methods such as multiple regression or the Kalman Filter to enable nonlinear path prediction.

The original plan was to use a deep learning model to segment the Puck and predict its path, but despite using a camera supporting 60 FPS, the segmentation was interrupted due to afterimages when the object moved. Therefore, instead of using a deep learning model, the segmentation of the blue Puck was carried out by setting the HSV channel value area. This resulted in a susceptibility to changes in illumination. There were no significant issues when playing the game under constant lighting, but problems occurred when shadows were formed or the lighting changed. It is expected that by using a higher FPS camera based on a deep learning model, issues of segmentation interruption can be prevented.

  

## 7. Tutorial

### 7.1. Download the Source File from GitHub

1.  Download the file from the provided GitHub repository [git-hub link](https://github.com/HwangSeungEun/IAIA_Final_Project_AirhockeyRobot.git).

2. After decompressing the downloaded file, move the **airhockey.sh** file located in the **airhoceky** folder to **catkin_ws**. Move folders with the prefix indy to **/catkin_ws/src**.

3. Proceed with the catkin_make command.

   1. If the cmake build is not successful, read the error message carefully and resolve the issue.
   2. The most common issue we encountered was the need to install packages that do not automatically get installed with ROS. Below are the commands to install the necessary packages for resolving build errors encountered during setup in a new environment.

   ```bash
   sudo apt install ros-noetic-industrial-robot-client
   sudo apt install ros-noetic-moveit-visual-tools
   sudo apt install ros-noetic-moveit-commander
   ```

   

### 7.2. Preset

- Hardware Settings

1. **Table Settings:**

   1. Use a tablet that can configure indy10 to manually move the Indy-10 robot to the reference position through direct teaching (e.g., [0, 0, tau/4, 0, tau/4, 0]).
   2. Align the center of the robot with the center of the table.
   3. Use a digital protractor to adjust the table's tilt close to 0 degrees. Proper alignment improves puck trajectory prediction accuracy.
   4. If the table is tilted, place an object under the table legs to level it.
   5. Connect the power to the air hockey table and turn the switch on.

   

2. **Robot Settings:**

   Robot settings need to be adjusted if the position is misaligned. Use the tablet connected to the robot to check the absolute coordinates.

   1. Specify the right, center, and left positions relative to the robot. If the paddle is too close to the table, the camera may shake along with the table during robot arm movement, so finding the proper distance is crucial.

   2. When choosing the robot's position after a goal is scored, set it to a safe location so that the robot doesn’t collide with the table during movement.

   3. Apply the absolute coordinates obtained in the above process to the following positions in **move_robot.py**.

      ```python
      self.target_joints_1 = [-10.0*deg,  -14.18*deg, 121.82*deg,   -1.07*deg,  -16.58*deg,  1.89*deg ] # Flag2
      self.target_joints_2 = [  0.0*deg,  -14.18*deg, 121.82*deg,   -1.07*deg,  -16.58*deg,  1.89*deg ] # Flag3
      self.target_joints_3 = [ 10.0*deg,  -14.18*deg, 121.82*deg,   -1.07*deg,  -16.58*deg,  1.89*deg ] # Flag4
              
      
      self.target_joint_goal_safe1 = [0.0*deg,-18.60*deg, 66.45*deg, -1.35*deg, 47.18*deg, 0.10*deg]
      self.target_joint_goal_safe2 = [0.0*deg,-28.38*deg, 107.98*deg, -1.30*deg, 101.33*deg, 0.10*deg]# @Flag4 --> Move safe loc
      self.target_joint_goal_grip = [-0.01*deg, -26.51*deg, 132.48*deg, -0.06*deg, 76.38*deg, 1.89*deg] # @Flag4 --> Grip loc
      self.target_joint_end       = [0, 0, 45*deg, 0, 45*deg, 0] # @Flag4 --> Grip loc
      
      ```

      

   4. Modify the path in **move_robot.py** at the location below.

      ```python
      sys.path.inser(0, 'indy_driver/src 전체 경로로 설정')
      ```

3. **Camera Settings:**
   1. Install the camera on the profile fixed to the table. If the camera is not fixed, accurate object recognition cannot be achieved.

   2. In **camera.py**, apply the camera index number between 0 and 2 in the corresponding section to select the camera to use. If a flickering camera image appears, it means that it is an IR camera, not the main camera, and you should change to a different index.

      ```python
      def __init__(self, cameraNumber = 0):
      ```

   3. When running the code, adjust the camera angle so that the lines representing the perimeter of the playing field match the actual game area.****



### 7.3. Running the Program

1. Open the terminal and navigate to the catkin_ws directory.

```bash
cd catkin_ws
```

2. Modifying the sh file:

If it's your first time downloading the sh file, you need to modify the "user name" in the code within the file to match your username.

**airhockey.sh**

```
python3 /home/"user name"/catkin_ws/src/indy_driver/src/indy_set_velocity.py
```

3. Execute the sh file by entering the following in the terminal.

```bash
./airhockey.sh
```

If the sh file is not recognized and does not execute, enter the following to make it executable and then run it.

```bash
chmod +x airhockey.sh
./airhockey.sh
```



### 7.4. Playing the Game

1. When the camera window appears, left-click and drag the blue puck area visible in the camera. Slide the puck to the right of the green line to check if it's being recognized properly. If a continuous green box appears around the puck, it is being recognized correctly.

<img src="https://github.com/DongminKim21800064/IAIA_Project2_RobotSonny/assets/91474647/88666fcb-dfc7-412c-b467-83e66c5ca9ef" alt="image" style="zoom:30%;" /><img src="https://github.com/DongminKim21800064/IAIA_Project2_RobotSonny/assets/91474647/ab57c561-4b83-4514-b7fb-9f56894f177d" alt="image" style="zoom:30%;" />

<center><strong>Figure 10. Game Setting 1</strong></center>

2. Place the puck in the circle in the center and enjoy the game by hitting it.

<img src="https://github.com/DongminKim21800064/IAIA_Project2_RobotSonny/assets/91474647/27dac51e-e8ad-432f-93a7-e3969d7934bc" alt="image" style="zoom: 30%;" /><img src="https://github.com/DongminKim21800064/IAIA_Project2_RobotSonny/assets/91474647/adefeaef-80f6-4bc7-ba92-d95f9bac2662" alt="image" style="zoom:30%;" />

<center><strong>Figure 10. Game Setting 2</strong></center>