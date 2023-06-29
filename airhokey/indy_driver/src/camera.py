import rospy
from sensor_msgs.msg import Image   # sensor_msgs 패키지로부터 Image 메시지 타입을 import
from cv_bridge import CvBridge, CvBridgeError     # cv_bridge 라이브러리 : OpenCV 이미지와 ROS 메시지 간의 변환 가능
import cv2 as cv                
import numpy as np
import os
import time
from indy_driver.msg import flag_data
import datetime

class Predict:

    def __init__(self, cameraNumber=0):
        
        self.drawing = False
        self.ix, self.iy = -1, -1
        self.fx, self.fy = -1, -1
        self.frame_hsv = None
        self.roi_hist = None
        self.user_font = cv.FONT_HERSHEY_SIMPLEX 
        self.x = None
        self.y = None
        
        self.previous_position = (0, 0)  # Initialize previous_position variable
        self.current_position = None
        self.previous_velocity = None
        self.move_right = False
        self.path_drawn_time = None
        self.path_line                 : list = None
        self.reflected_path_line       : list = None
        self.extra_reflected_path_line : list = None
        self.reflected_x2 = None
        self.reflected_y2 = None
        self.slope = None
        self.previous_slope = None
        self.center_x = 0
        self.center_y = 0

        # Count the number of frames without contours
        self.no_contour = 0
        self.no_contour_flag = 0

        # slope calculate
        self.slope_list = [] 
        self.stored_position = None 
        self.flag = 0
        self.path_drawn = False

        self.is_goal = False
        self.shoot = 0

        # camera setting
        self.cap        = cv.VideoCapture(cameraNumber)          # 카메라 연결을 위한 VideoCapture 객체 생성
        fourcc          = cv.VideoWriter.fourcc('M','J','P','G')
        self.cap.set(cv.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv.CAP_PROP_FPS, 60.0) 
        self.width      = int(self.cap.get(cv.CAP_PROP_FRAME_WIDTH))
        self.height     = int(self.cap.get(cv.CAP_PROP_FRAME_HEIGHT))
        self.fps        = int(self.cap.get(cv.CAP_PROP_FPS))

        print('get cam fps: ', self.fps)

        self.output_width = self.width
        self.output_height = self.height
        self.output_fps = self.fps
        
        date = datetime.datetime.now().strftime('%y%m%d_%H%M%S')
        self.output_filename = str(date) + '.mp4'
        self.output_video = cv.VideoWriter(self.output_filename, fourcc, self.output_fps, (self.output_width, self.output_height))

        cv.namedWindow('Video', cv.WINDOW_NORMAL)
        cv.resizeWindow('Video', self.width, self.height)
        cv.setMouseCallback('Video', self.select_roi)

        # field boundary
        self.line_x1 = 90
        self.line_y1 = 93
        self.line_x2 = self.width - 45
        self.line_y2 = self.height - 105  
          

        if not self.cap.isOpened():
            print("Unable to open video file")
            
            exit()

        # ros communicate setting
        self.pub_flag   = rospy.Publisher("robot_flag", flag_data, queue_size=1)
        self.flag       = flag_data()


    def select_roi(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.ix, self.iy = x, y
            self.fx, self.fy = x, y
            
        elif event == cv.EVENT_MOUSEMOVE:
            if self.drawing == True:
                self.fx, self.fy = x, y
                
        elif event == cv.EVENT_LBUTTONUP:
            self.drawing = False
            self.fx, self.fy = x, y
            
            hsv_roi = self.frame_hsv[self.iy:self.fy, self.ix:self.fx]
            mask = cv.inRange(hsv_roi, np.array((0., 40., 22.)), np.array((180., 255., 255.)))
            self.roi_hist = cv.calcHist([hsv_roi], [0], mask, [180], [0, 180])
            cv.normalize(self.roi_hist, self.roi_hist, 0, 255, cv.NORM_MINMAX)

    def slope_avg(self, new_slope):
        # Append new_slope to the list
        if len(self.slope_list) <= 3:
            self.slope_list.append(new_slope)

        if len(self.slope_list) == 3:
            self.stored_position = self.current_position
            self.path_drawn = False  
            
        # Compute the average of the slopes
        self.slope = sum(self.slope_list) / len(self.slope_list) 
        
    def predict_path(self, contours):
        # initialize variables
        roi_x1 = self.line_x1
        roi_y1 = self.line_y1
        roi_x2 = self.line_x2
        roi_y2 = self.line_y2
        
        hit_border = None
        hit_border2 = None
        reflected_y2 = None
        reflected_x2 = None
        
        if contours:
            for cnt in contours:
                area = cv.contourArea(cnt)
                # filter outlier contours
                
                if 200 <= area <= 380:
                    self.x, self.y, w, h = cv.boundingRect(cnt)
                    # check object in roi
                    if (roi_x1 <= self.x <= roi_x2) and (roi_y1 <= self.y <= roi_y2): 
                        self.no_contour_flag = 0
                        # Calculate the center of the bounding box
                        self.center_x = self.x + w // 2
                        self.center_y = self.y + h // 2
                        self.current_position = (self.center_x, self.center_y)
                        # object target
                        cv.rectangle(frame, (self.x, self.y), (self.x + w, self.y + h), (0, 255, 0), 2)
                        
                        x_move = self.current_position[0] - self.previous_position[0]
                        move_distance = float(np.sqrt((self.current_position[0] - self.previous_position[0])**2+(self.current_position[1] - self.previous_position[1])**2))
                        
                        # Reset slope list and drawn paths if object moves left more than -3
                        if x_move < -3:
                            self.slope_list = []
                            self.path_line = None
                            self.reflected_path_line = None
                            self.path_drawn = False
                            self.extra_reflected_path_line = None 
                        
                        # If puck is moving to the right
                        if self.previous_position is not (0,0) and x_move >= 3 and move_distance >= 4.5:
                            if self.x > 120:
                                self.puck_move = True
                                self.slope_avg((self.current_position[1] - self.previous_position[1]) / (self.current_position[0] - self.previous_position[0]))
                        else:
                            self.puck_move = False
                           
                        # If puck is moving
                        if self.puck_move == True and len(self.slope_list) > 0 and self.path_drawn == False: 
                 
                            if self.slope != 0 and self.slope < 0:
                                predicted_x = self.previous_position[0] + (roi_y1 - self.previous_position[1]) / self.slope 
                            elif self.slope != 0 and self.slope > 0:
                                predicted_x = self.previous_position[0] + (roi_y2 - self.previous_position[1]) / self.slope 
                            else:
                                predicted_x = self.previous_position[0]

                            # Ensure that predicted_x does not exceed the boundaries.
                            predicted_x = min(max(predicted_x, roi_x1), roi_x2)
                                
                            # Predicted path line (in boundary)
                            predicted_y1 = self.current_position[1] + self.slope * (roi_x1 - self.current_position[0])
                            predicted_y2 = self.current_position[1] + self.slope * (roi_x2 - self.current_position[0])
                            predicted_y1 = max(roi_y1, min(roi_y2, predicted_y1))
                            predicted_y2 = max(roi_y1, min(roi_y2, predicted_y2))
                          
                            # Remember the border (upper or lower) that was hit
                            self.hit_border = roi_y1 if predicted_y2 == roi_y1 else roi_y2
                            
                            if self.hit_border is not None and predicted_x != roi_x2:
                                if self.slope != 0:
                                    if 0 <= abs(self.slope) <= 1.5:
                                       self.slope = self.slope * 0.4 
                                    if 1.5 < self.slope <= 5:
                                       self.slope = self.slope * 0.7
                                    if -5 < self.slope <= -1.5:
                                       self.slope = self.slope * 0.8
                                      
                                    self.reflected_y2 = self.hit_border - self.slope  * (roi_x2 - self.current_position[0])
                                    self.reflected_x2 = self.current_position[0] + (self.reflected_y2 - self.hit_border) / -self.slope 
                                  
                                    # when hit the borders                                    
                                    if self.reflected_y2 >= roi_y2 and self.reflected_x2 <= roi_x2:
                                        self.reflected_y2 = roi_y2
                                        self.reflected_x2 = self.current_position[0] + (self.reflected_y2 - self.hit_border) / -self.slope # * 1.2 

                                    if self.reflected_y2 <= roi_y1 and self.reflected_x2 <= roi_x2:
                                        self.reflected_y2 = roi_y1
                                        self.reflected_x2 = self.current_position[0] + (self.reflected_y2 - self.hit_border) / -self.slope
                                    
                                if self.reflected_x2 is not None and self.reflected_y2 is not None:
                                    self.reflected_path_line =[[int(predicted_x), int(predicted_y2)], [int(self.reflected_x2), int(self.reflected_y2)]]
                                    # print(self.reflected_path_line)

                                    self.reflected_path_line = list(self.reflected_path_line)
                                    # print(self.reflected_path_line)
                                    
                                else:
                                    self.reflected_path_line = None
                            
                            if self.reflected_x2 is not None:   
                                self.reflected_x2 = round(self.reflected_x2,3)  
                              
                            else:
                                self.reflected_path_line = None
                            
                            if self.reflected_y2 is not None:
                                self.hit_border2 = roi_y1 if self.reflected_y2 == roi_y1 else roi_y2
                    
                            if self.hit_border2 is not None and self.reflected_x2 != roi_x2:
                                self.reflected_x3 = roi_x2
                                self.reflected_y3_temp = self.hit_border2 + self.slope *  (roi_x2 - self.reflected_x2)  # reflection
                                
                                # self.reflected_y3_temp will be bounded by the ROI borders
                                self.reflected_y3 = max(roi_y1, min(roi_y2, self.reflected_y3_temp))

                                self.extra_reflected_path_line = [[int(self.reflected_x2), int(self.reflected_y2)], [int(self.reflected_x3), int(self.reflected_y3)]]
                                print(self.path_line)
                                print(self.reflected_path_line)
                                print(self.extra_reflected_path_line)

                            else:
                                self.extra_reflected_path_line = None

                            self.path_line = [[self.current_position[0], self.current_position[1]], [int(predicted_x), int(predicted_y2)]]
                            
                        else:
                            # Reset paths
                            self.predicted_path = None
                            self.reflected_path = None
                            #self.extra_reflected_path_line = None
                            

                        self.previous_position = self.current_position
                        self.path_drawn_time = time.time()
                        self.path_drawn = True
           
    def goal(self):
        goal_x = self.line_x2 - 50
        goal_y1 = 0
        goal_y2 = self.height   

        # Check if self.center_x has passed the goal line
        if self.center_x >= goal_x:

            if self.no_contour > 200:
                self.path_line = None
                self.relfected_path_line = None
                self.extra_reflected_path_line = None
                self.flag = 4
                self.is_goal = True
                cv.putText(frame, "Flag: " + str(self.flag), (450,80), self.user_font, 0.7, (255, 255, 0), 2)

            if self.no_contour > 400:
                self.no_contour = 0
                self.no_contour_flag = 1
                self.flag = 0
               
        else:
            # Reset the no_contour counter if the object is not past the goal line
            self.no_contour = 0
            self.no_contour_flag = 1
            
        if self.flag == 4:
            cv.putText(frame, "---- YOU WIN ----", (220,200), self.user_font, 0.7, (255, 255, 255), 3)
            cv.putText(frame, "---- YOU WIN ----", (220,200), self.user_font, 0.7, (0, 0, 0), 2)

    def decide_flag(self):
        roi_width = self.line_y2 - self.line_y1
        y_div = roi_width // 5
        
        # set the flag area
        robot_area1 = [int(self.width / 10 * 9), self.line_y1, self.width, self.line_y1 + y_div * 2]
        robot_area2 = [int(self.width / 10 * 9), self.line_y1 + y_div * 2, self.width, self.line_y1 + 3 * y_div]
        robot_area3 = [int(self.width / 10 * 9), self.line_y1 + 3 * y_div, self.width, self.line_y2]
        
        cv.rectangle(frame, (robot_area1[0], robot_area1[1]), (robot_area1[2], robot_area1[3]), (255, 255, 255), 2)
        cv.rectangle(frame, (robot_area2[0], robot_area2[1]), (robot_area2[2], robot_area2[3]), (255, 255, 255), 2)
        cv.rectangle(frame, (robot_area3[0], robot_area3[1]), (robot_area3[2], robot_area3[3]), (255, 255, 255), 2)

        self.final_point = []
        
        # set final point 
        if self.path_line is not None and self.reflected_path_line is None:
            self.final_point = self.path_line[1]  

        if self.reflected_path_line is not None:
            if (self.path_line != None and self.reflected_path_line != None):
                self.final_point = self.reflected_path_line[1]  

        if self.extra_reflected_path_line is not None:
            self.final_point = self.extra_reflected_path_line[1]  

        # check final point position in robot area    
        if self.final_point and self.flag != 4:
            
            if robot_area1[1] <= self.final_point[1] <= robot_area1[3]:
                self.flag = 1
                cv.rectangle(frame, (robot_area1[0], robot_area1[1]), (robot_area1[2], robot_area1[3]), (255, 151, 153), 5)
            if robot_area2[1] <= self.final_point[1] <= robot_area2[3]:
                self.flag = 2
                cv.rectangle(frame, (robot_area2[0], robot_area2[1]), (robot_area2[2], robot_area2[3]), (255, 151, 153), 5)
            if robot_area3[1] <= self.final_point[1] <= robot_area3[3]:
                self.flag = 3
                cv.rectangle(frame, (robot_area3[0], robot_area3[1]), (robot_area3[2], robot_area3[3]), (255, 151, 153), 5)
            if self.flag != 4:    
                cv.putText(frame, "Flag: " + str(self.flag), (450,50), self.user_font, 0.7, (255, 255, 0), 2)
       
        self.pub_flag.publish(self.flag)
            
    def run(self):

        rospy.init_node('detect', anonymous=True)  # 노드 이름 "detect"로 초기화
        rate = rospy.Rate(self.fps)                           # 루프 실행 주기 : 60hz

        while not rospy.is_shutdown():                  # ROS가 종료되지 않은 동안
            global frame
            self.cur_time = time.time()	# 현재 시간을 
            self.prev_time = self.cur_time # use self.start_time instead of self.start_time_sub

            ret, frame = self.cap.read()

            if not ret:
                print("End of video")
                break

            #frame = cv.flip(frame, 1)
            self.frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

            #cv.line(frame, (self.line_x1, self.line_y1), (self.line_x2, self.line_y1), (153, 0, 204), 2)
            #cv.line(frame, (self.line_x1, self.line_y2), (self.line_x2, self.line_y2), (153, 0, 204), 2)
            
            # show filed area
            overlay = frame.copy()
            cv.rectangle(overlay, (25, 93), (self.width - 20, self.height - 105), (204, 153, 255), -1)
            cv.rectangle(frame, (25, 93), (self.width - 20, self.height - 105), (102, 51, 204), 2)
            transparency = 0.2
            frame = cv.addWeighted(overlay, transparency, frame, 1 - transparency, 0)

            start_line = np.array([(110, self.line_y1), (110, self.line_y2)])      
            
            cv.putText(frame, "Shoot Line", (60,75), self.user_font, 0.7, (255, 255, 255), 3)
            cv.putText(frame, "Shoot Line", (60,75), self.user_font, 0.7, (0, 0, 0), 2)
            cv.line(frame, (start_line[0]), (start_line[1]), (100, 255, 40), 1)
            
            if self.drawing:
                cv.rectangle(frame, (self.ix, self.iy), (self.fx, self.fy), (255, 0, 0), 2)

            if self.roi_hist is not None:
                back_proj = cv.calcBackProject([self.frame_hsv], [0], self.roi_hist, [0, 180], 1)
                _, threshold = cv.threshold(back_proj, 1, 255, cv.THRESH_BINARY)
                contours, _ = cv.findContours(threshold, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

                # puck detection 
                detected = False
                for contour in contours:
                    area = cv.contourArea(contour)
                    if 200 < area < 380:
                        if self.x is not None and self.y is not None:
                            if (self.line_x1 <= self.x <= self.line_x2) and (self.line_y1 <= self.y <= self.line_y2):
                                detected = True
                                self.no_contour_flag == 1

                # no contour count
                if detected == True:
                    self.no_contour_flag == 1
                    
                if self.no_contour_flag == 0 and detected == False:
                    self.no_contour += 1
                    print(self.no_contour)
          
                else:
                    self.no_contour = 0
                    self.no_contour_flag == 1
    
                self.predict_path(contours)
            
            self.goal()

            if self.path_line is not None and self.flag != 4 and self.flag != 0: 
                cv.line(frame, *self.path_line, (204, 102, 000), 2)
                self.path_drawn = True
                # self.shoot += 1
                # if self.shoot == 6:
                #     self.shoot = 0
            if self.reflected_path_line is not None and self.flag != 4 and self.flag != 0:
                cv.line(frame, *self.reflected_path_line, (204, 102, 000), 2)
                self.path_drawn = True

            if self.extra_reflected_path_line is not None and self.flag != 4 and self.flag != 0:
                cv.line(frame, *self.extra_reflected_path_line, (204, 102, 000), 2)
                self.path_drawn = True

            # if self.is_goal == True and self.shoot == 1:
            #     cv.circle(frame,(20.20), 10, (0,255,0), 2)
            # elif self.is_goal == False and self.shoot == 1:
            #     cv.circle(frame,(20.20), 10, (0,0,255), 2)
            # if self.is_goal == True and self.shoot == 2:
            #     cv.circle(frame,(30.20), 10, (0,255,0), 2)
            # elif self.is_goal == False and self.shoot == 2:
            #     cv.circle(frame,(30.20), 10, (0,0,255), 2)
            # if self.is_goal == True and self.shoot == 3:
            #     cv.circle(frame,(40.20), 10, (0,255,0), 2)
            # elif self.is_goal == False and self.shoot == 3:
            #     cv.circle(frame,(40.20), 10, (0,0,255), 2)  
            # if self.is_goal == True and self.shoot == 4:
            #     cv.circle(frame,(50.20), 10, (0,255,0), 2)
            # elif self.is_goal == False and self.shoot == 4:
            #     cv.circle(frame,(50.20), 10, (0,0,255), 2)   
            # if self.is_goal == True and self.shoot == 5:
            #     cv.circle(frame,(60.20), 10, (0,255,0), 2)
            # elif self.is_goal == False and self.shoot == 5:
            #     cv.circle(frame,(60.20), 10, (0,0,255), 2)       

            # if self.shoot == 5:
            #     self.shoot = 0


            if self.x is not None and self.x < 120:
                self.path_line = None
                self.reflected_path_line = None
                self.extra_reflected_path_line = None
                   
                
            # print("Flag: " + str(self.flag))
            
            self.decide_flag()    

            diff_time = time.time() - self.prev_time

            fps = 0
            if diff_time > 0:
                fps = 1 / diff_time

            # cv.putText(frame, f'FPS : {fps:.2f}', (20, 90), self.user_font, 2,  (100, 255, 0), 2)
            cv.imshow("Video", frame) 
            self.output_video.write(frame)

            key = cv.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                cv.waitKey(0)

            elif key == ord('1'):
                self.flag = 1
                self.pub_flag.publish(self.flag)
            elif key == ord('2'):
                self.flag= 2
                self.pub_flag.publish(self.flag)
            elif key == ord('3'):
                self.flag = 3
                self.pub_flag.publish(self.flag)
            elif key == ord('4'):
                self.flag = 4
                self.pub_flag.publish(self.flag)

                # 1ms 동안 키보드 입력 대기

            rate.sleep()                                # 지정된 루프 실행 주기에 따라 대기  

        self.cap.release()
        self.output_video.release()
        cv.destroyAllWindows()

if __name__ == '__main__':
    try:
        display = Predict(0)     # DisplayNode 클래스의 인스턴스 생성
        display.run()               # 노드 실행

    except rospy.ROSInterruptException:
        pass
