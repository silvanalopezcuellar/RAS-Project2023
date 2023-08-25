import base64 
import cv2 as cv
import numpy as np
import imutils

import rclpy
from rclpy.node import Node
import sys
from sensor_msgs.msg import Image

from djitellopy import Tello
import cv2, math, time

gate = 2
flag=1
first_time=1

tello = Tello()
tello.connect()

tello.streamon()
frame_read = tello.get_frame_read()

tello.takeoff()
  
while flag == 1:

    
    def get_center(center):
        x = int(np.sum(np.array(center).T[0])/len(center))
        y = int(np.sum(np.array(center).T[1])/len(center))
        return x,y

    def get_corner_center(corners):
        center = []
        for i in range(len(corners)):
            x = int(np.sum(corners[i][0].T[0])/4.0)
            y = int(np.sum(corners[i][0].T[1])/4.0)
            center.append([x,y])
        #print(np.matrix(center))
        return center
    

    def get_arauco_img(frame):
        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_250)
        parameters =  cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(aruco_dict,parameters)
        corners,ids,rejectCandidates = detector.detectMarkers(frame)
        return corners,ids
    

    def image_sub_callback():
        img = frame_read.frame
        img = imutils.resize(img, width=600)
        flag1=0
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask_green = cv.inRange(hsv, (30,43,35),(90,255,255))

        contours, hierarchy = cv.findContours(mask_green, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        contours_index_list=[1,1,1,1]
        area_list=[1,1,1,1]
        if len(contours) <= 1:
            flag1 = -1
            return img, None, None, flag1, None, None 
        else:
            for c in range(len(contours)):
                area = cv.contourArea(contours[c])
                if area > min(area_list):
                    contours_index_list[area_list.index(min(area_list))]=c
                    area_list[area_list.index(min(area_list))]=area
            area_list_new=[]
            contours_index_list_new=[]
            for i in range(len(area_list)):
                if area_list[i]<max(area_list)*0.3:
                    pass
                else:
                    area_list_new.append(area_list[i])
                    contours_index_list_new.append(contours_index_list[i])
            x_list=[]
            y_list=[]
            xplusy_list=[]
            for index in contours_index_list_new:
                rect = cv.minAreaRect(contours[index])
                box = np.int0(cv.boxPoints(rect))
                x_list.append(np.int32(rect[0][0]))
                y_list.append(np.int32(rect[0][1]))
                xplusy_list.append(np.int32(rect[0][0])+np.int32(rect[0][1]))
                cv.drawContours(img,[box],0,(0,255,0),3)

            center_x = int(x_list[xplusy_list.index(max(xplusy_list))]/2 + x_list[xplusy_list.index(min(xplusy_list))]/2)
            center_y = int(y_list[xplusy_list.index(max(xplusy_list))]/2 + y_list[xplusy_list.index(min(xplusy_list))]/2)
            height = y_list[xplusy_list.index(max(xplusy_list))]-y_list[xplusy_list.index(min(xplusy_list))]


            x1 = x_list[xplusy_list.index(min(xplusy_list))]
            x2 = x_list[xplusy_list.index(max(xplusy_list))]
            y1 = y_list[xplusy_list.index(min(xplusy_list))]
            y2 = y_list[xplusy_list.index(max(xplusy_list))]

            cv2.imshow("drone", img)
            cv.waitKey(1)
            return area_list_new, center_x, center_y, x1, x2, y1, y2
        

    def circle_gate (frame,gate,first_time):
        print ("first_time before gate: ", first_time)

        
        frame = imutils.resize(frame, width=600)


        corners,ids=get_arauco_img(frame)

        
        count =0
        clock_complete=0
        while ids is None:
            
            print("rotate")
            


            
            frame = frame_read.frame
            corners,ids=get_arauco_img(frame)
            print ("Count hight:", count)
            
            if first_time == 1:
                tello.move_up(20)
                first_time=0
                count=count+1
            print("clock_complete: ",clock_complete)
            if (clock_complete > 32):
                clock_complete=clock_complete=1

                
                frame = frame_read.frame
                corners,ids=get_arauco_img(frame)
                
            
            frame = frame_read.frame
            corners,ids=get_arauco_img(frame)

            if ids is None:
                tello.rotate_clockwise(5)
                clock_complete=clock_complete+1
            
            frame = frame_read.frame
            corners,ids=get_arauco_img(frame)

            if ids is None:
                tello.rotate_clockwise(10)
                clock_complete=clock_complete+1    


        
        cc = get_corner_center(corners)
        ccc = get_center(cc)
        cv.circle(frame, ccc, 70, (0, 0, 255), 3, 8, 0)

        print("ccc=", np.matrix(ccc))

        cv.imshow("frame",frame)
        cv.waitKey(1)

        #missing=0
        markerref = [1,4,3,2]
        k=0

        print("ids=", np.matrix(ids))

        if len(ids) == 3:
            tello.move_back(20)

        
        missing = 0
        if len(ids) < 4:
            for i in (0,3):
                if markerref[i] not in ids:
                    missing = markerref[i]
                    print("Missing ids:", np.matrix(missing))

        if missing == 1:
            print("down")
            tello.move_down(20)
        elif (missing == 2) and (len(ids>2)):
            print("up")
            tello.move_up(20)
        elif missing == 3:
            print("right")
            tello.move_right(30)
        elif missing == 4:
            print("left")
            tello.move_left(30)
        else:
            
            if ccc[0] is not range (200, 340) :
                if ccc[0] < 200:
                    print("X=", ccc[0])
                    print("left")
                    tello.move_left(20)
                elif ccc[0] > 340:
                    print("X=", ccc[0])
                    print("right")
                    tello.move_right(20)

                frame = frame_read.frame
                frame = imutils.resize(frame, width=600)
                corners,ids=get_arauco_img(frame)
                

                time.sleep(2)
                print("sleep 2S")

                cc = get_corner_center(corners)
                ccc = get_center(cc)
                print("CCC=", np.matrix(ccc))


            if ccc[1] is not range (200, 350) :
                if ccc[1] < 200:
                    print("Y=", ccc[1])  
                    print("UP")
                    tello.move_up(20)
                elif ccc[1] > 350:
                    print("Y=", ccc[1])
                    print("DOWN")
                    tello.move_down(20)

                frame = frame_read.frame
                frame = imutils.resize(frame, width=600)
                time.sleep(2)
                print("sleep 2s")

                corners,ids=get_arauco_img(frame)


                time.sleep(2)
                print("sleep 2S")

                cc = get_corner_center(corners)
                ccc = get_center(cc)
                print("CCC=", np.matrix(ccc))

            if len(ids) == 4:    
                tello.move_forward(20)
                tello.move_down(20)
                print("forward 150-1")
                tello.move_forward(130)
                print("forward 150-2")
                tello.move_forward(100)
                gate=3
                time.sleep(2)
        return gate, first_time
    

    def coners_gate (gate):
        
        area_list,ccc2_x,ccc2_y, x1, x2, y1, y2 = image_sub_callback()
        print("area_list, ccc2_x,ccc2_y, x1, x2, y1, y2 = ", area_list,ccc2_x,ccc2_y, x1, x2, y1, y2)

        
        if y1 is None:
            print("down")
            tello.move_down(30)
        elif y2 is None:
            print("up")
            tello.move_up(30)
        elif x2 is None:
            print("right")
            tello.move_right(30)
        elif x1 is None:
            print("left")
            tello.move_left(30)
        else:
            print("ELSE TEST!")
            if ccc2_x is not range (200, 300):
                if ccc2_x < 200:
                    print("X=", ccc2_x)
                    print("left")
                    tello.move_left(30)
                elif ccc2_x > 300:
                    print("X=", ccc2_x)
                    print("right")
                    tello.move_right(30)

                area_list,ccc2_x,ccc2_y, x1, x2, y1, y2 = image_sub_callback()
                print("ccc2_x,ccc2_y, x1, x2, y1, y2 = ", ccc2_x,ccc2_y, x1, x2, y1, y2)
            if ccc2_y is not range (100, 200) :
                if ccc2_y < 100:
                    print("Y=", ccc2_y)  
                    print("UP")
                    tello.move_up(30)
                elif ccc2_y > 200:
                    print("Y=", ccc2_y)
                    print("DOWN")
                    tello.move_down(40)

                area_list,ccc2_x,ccc2_y, x1, x2, y1, y2 = image_sub_callback()
                print("ccc2_x,ccc2_y, x1, x2, y1, y2 = ", ccc2_x,ccc2_y, x1, x2, y1, y2)

            if len(area_list) == 4 and max(area_list):
                print("forward 150-1")
                tello.move_forward(150)
                print("forward 150-2")
                tello.move_forward(150)
                gate=0
                time.sleep(5)

            else:   
                tello.move_down(20)
                tello.move_back(30)
                if len(area_list) < 3:
                    tello.move_up(40)
        return gate
    

    def sq_gate (gate):
        
        area_list,ccc2_x,ccc2_y, x1, x2, y1, y2 = image_sub_callback()
        print("area_list, ccc2_x,ccc2_y, x1, x2, y1, y2 = ", area_list,ccc2_x,ccc2_y, x1, x2, y1, y2)

        
        if y1 is None:
            print("down")
            tello.move_down(30)
        elif y2 is None:
            print("up")
            tello.move_up(30)
        elif x2 is None:
            print("right")
            tello.move_right(30)
        elif x1 is None:
            print("left")
            tello.move_left(30)
        else:
            print("ELSE TEST!")
            if ccc2_x is not range (200, 400):
                if ccc2_x < 200:
                    print("X=", ccc2_x)
                    print("left")
                    tello.move_left(30)
                elif ccc2_x > 400:
                    print("X=", ccc2_x)
                    print("right")
                    tello.move_right(30)

                area_list,ccc2_x,ccc2_y, x1, x2, y1, y2 = image_sub_callback()
                print("ccc2_x,ccc2_y, x1, x2, y1, y2 = ", ccc2_x,ccc2_y, x1, x2, y1, y2)
            if ccc2_y is not range (150, 200) :
                if ccc2_y < 150:
                    print("Y=", ccc2_y)  
                    print("UP")
                    tello.move_up(30)
                elif ccc2_y > 200:
                    print("Y=", ccc2_y)
                    print("DOWN")
                    tello.move_down(40)

                area_list,ccc2_x,ccc2_y, x1, x2, y1, y2 = image_sub_callback()
                print("ccc2_x,ccc2_y, x1, x2, y1, y2 = ", ccc2_x,ccc2_y, x1, x2, y1, y2)

            if max(area_list) > 50000:
                print("forward 150-1")
                tello.move_forward(150)
                print("forward 150-2")
                tello.move_forward(150)
                gate=2
                time.sleep(5)

        return gate




    frame = frame_read.frame
    time.sleep(5)

    

    print("Gate: ",gate)
    if (gate == 1):
        gate,first_time=circle_gate (frame,gate,first_time)
        print ("first_time after gate: ", first_time)
    elif(gate == 2):
        gate=coners_gate(gate)
    elif(gate == 3):
        gate=sq_gate(gate)
    else:
        tello.land()


