import numpy as np                      #necessary imports
import cv2
import time 
import math
import serial


color=(255,0,0)                         #variable for contour color and thickness
thickness=2
cX = cY = 0                             #centroid of ball contour
cap = cv2.VideoCapture(1)               #capture from video camera 
j=0
prev_ang1,prev_ang2 = 0,0     #previous co-ordinates of ball contour centriod
x_cor,y_cor,i = 0,0,0                   #x,y co-ordinate of edge of platform initialize
s = serial.Serial("COM3",9600)          #Establish Serial Communication
s.baudrate = 9600
portion = 0

def Platform(c):

    global x_cor,y_cor,img2,Left,Right,Top,Bottom,frame,Q
    
    Left = tuple(c[c[:, :, 0].argmin()][0])     #This is creating a tuple of x,y cordinates of extreme points
    Right = tuple(c[c[:, :, 0].argmax()][0])    #Minimum along X-Axis is Left and similar logic for others
    Top = tuple(c[c[:, :, 1].argmin()][0])
    Bottom = tuple(c[c[:, :, 1].argmax()][0])

    x_cor = int(((Right[0] - Left[0])**2 + (Right[1] - Left[1])**2 )**0.5)  #Sides of the platform (dynamically)
    y_cor = int(((Bottom[0] - Top[0])**2 + (Bottom[1] - Top[1])**2 )**0.5)
    
    pts1 = np.float32([(list(Top),list(Right),list(Bottom),list(Left))])    #List of all 4 corners
    pts2 = np.float32([[0,0],[x_cor,0],[x_cor,y_cor],[0,y_cor]])            #List of 4 points we want to map it to
    Q = cv2.getPerspectiveTransform(pts1,pts2)                              #Get the Transformation Matrix
    


def Ball_Track():

    global dst,x_cor,y_cor,thresh1,frame,Q,i

    dst = cv2.warpPerspective(frame,Q,(x_cor,y_cor))              #Trsansform and view in orthogonal perspective
    
    gray1 = cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)
    ret,thresh1 = cv2.threshold(gray1,170,255,cv2.THRESH_BINARY) 
    (_,cont_bw,hierarchy)=cv2.findContours(thresh1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)   #contours of ball

    cv2.circle(dst, (x_cor//2,y_cor//2), 8, (255, 255, 0), -1)
    
    if len(cont_bw) != 0:
        #l = max(cont_bw, key = cv2.contourArea)
        for q in range(len(cont_bw)):
            peri = cv2.arcLength(cont_bw[q], True)
            approx = cv2.approxPolyDP(cont_bw[q], 0.01 * peri, True)
            area = cv2.contourArea(cont_bw[q])
            #print(len(approx))
            if peri != 0 :
                #print(area/peri)
                if (len(approx)>=7 and area/peri > 8):      # circle will have more than 7 sides and also area/peri is Radius/2
                    print(area/peri)
                    dst=cv2.drawContours(dst, cont_bw[q], -1, [0,255,0], thickness) #Draw contours of the ball
                    M = cv2.moments(cont_bw[q])
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])           #Centroid of ball
                        cY = int(M["m01"] / M["m00"])
                        i = [cX,cY]                             #List of centroid
                        print(i)
                        data = PID()                            #Get Servo Angles to send by PID
                        Serial_C(data)                          #Send data to Arduino

def Translate_X():
    while(True):
        time.sleep(1)
        Serial_C("110070070110110110*")
        time.sleep(1)
        Serial_C("070110110070070070*")
        
def Translate_Y():
    while(True):
        time.sleep(1)
        Serial_C("110090090070070110*")
        time.sleep(1)
        Serial_C("070090090110110070*")


        
def Translate_Z():
    while(True):
        time.sleep(1)
        Serial_C("110110110110110110*")
        time.sleep(1)
        Serial_C("070070070070070070*")
        
def Rotate_X():
    while(True):
        time.sleep(1)
        Serial_C("090090090090110110*")
        time.sleep(1)
        Serial_C("090090090090070070*")
        
def Rotate_Y():
    while(True):
        time.sleep(1)
        Serial_C("110110070070090090*")
        time.sleep(1)
        Serial_C("070070110110090090*")
        
def Rotate_Z():
    while(True):
        time.sleep(1)
        Serial_C("110070110070110070*")
        time.sleep(1)
        Serial_C("070110070110070110*")


def PID():

    global x_cor,y_cor,i
    global int_x,int_y,prev_x,prev_y
    global portion,prev_ang1,prev_ang2

    
    dx = 15*(i[0]-x_cor/2)//x_cor                           #Co-ordinates of Ball maped in cm
    dy = 15*(i[1]-y_cor/2)//y_cor
    
    
    Kp = 1       #1
    Kd = -40     #-35   
    Ki = 0.05   #-0.01                                          #PID co-efficients

    distance = (dx*dx + dy*dy )**0.5
    theta= np.degrees(math.atan2(dy,dx))

    if(theta < 0):
        theta = 360 + theta
    
    if(theta < 60):
        if(portion != 1 and portion != 0):
            temp = prev_ang1
            prev_ang1 = prev_ang2
            prev_ang2 = temp 

        Angle_1 = (30*distance*math.sin((60-theta)*(math.pi/180)))/(7)
        Angle_2 = (30*distance*math.sin(theta*(math.pi/180)))/(7)

        Data_1 = max(-15,min(15,int(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))))
        Data_2 = max(-15,min(15,int(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))))

        Servo_1 = str(90-Data_2).zfill(3)
        Servo_2 = str(90-Data_2).zfill(3)
        Servo_3 = str(90).zfill(3)
        Servo_4 = str(90).zfill(3)
        Servo_5 = str(90+Data_1).zfill(3)
        Servo_6 = str(90+Data_1).zfill(3)
                             
        portion = 1

    elif(theta < 120):
        theta = theta - 60
        if(portion!=2 and portion !=0):
            temp = prev_ang1
            prev_ang1 = prev_ang2
            prev_ang2 = temp
            
        Angle_1 = (30*distance*math.sin((60-theta)*(math.pi/180)))/(7)
        Angle_2 = (30*distance*math.sin(theta*(math.pi/180)))/(7)

        Data_1 = max(-15,min(15,int(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))))
        Data_2 = max(-15,min(15,int(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))))

        Servo_1 = str(90-Data_1).zfill(3)
        Servo_2 = str(90-Data_1).zfill(3)
        Servo_3 = str(90+Data_2).zfill(3)
        Servo_4 = str(90+Data_2).zfill(3)
        Servo_5 = str(90).zfill(3)
        Servo_6 = str(90).zfill(3)
        portion = 2

    elif(theta < 180):
        theta = theta - 120
        if(portion!=3 and portion !=0):
            temp = prev_ang1
            prev_ang1 = prev_ang2
            prev_ang2 = temp 
    
        Angle_1 = (30*distance*math.sin((60-theta)*(math.pi/180)))/(7)
        Angle_2 = (30*distance*math.sin(theta*(math.pi/180)))/(7)

        Data_1 = max(-15,min(15,int(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))))
        Data_2 = max(-15,min(15,int(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))))

        Servo_1 = str(90).zfill(3)
        Servo_2 = str(90).zfill(3)
        Servo_3 = str(90+Data_1).zfill(3)
        Servo_4 = str(90+Data_1).zfill(3)
        Servo_5 = str(90-Data_2).zfill(3)
        Servo_6 = str(90-Data_2).zfill(3)
        
        portion = 3

    elif(theta < 240):
        theta = theta - 180
        if(portion!=4 and portion !=0):
            temp = prev_ang1
            prev_ang1 = prev_ang2
            prev_ang2 = temp 
    
        Angle_1 = (30*distance*math.sin((60-theta)*(math.pi/180)))/(7)
        Angle_2 = (30*distance*math.sin(theta*(math.pi/180)))/(7)

        Data_1 = max(-15,min(15,int(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))))
        Data_2 = max(-15,min(15,int(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))))

        Servo_1 = str(90+Data_2).zfill(3)
        Servo_2 = str(90+Data_2).zfill(3)
        Servo_3 = str(90).zfill(3)
        Servo_4 = str(90).zfill(3)
        Servo_5 = str(90-Data_1).zfill(3)
        Servo_6 = str(90-Data_1).zfill(3)
                             
        portion = 4

    elif(theta < 300):
        theta = theta - 240
        if(portion!=5 and portion !=0):
            temp = prev_ang1
            prev_ang1 = prev_ang2
            prev_ang2 = temp 
    
        Angle_1 = (30*distance*math.sin((60-theta)*(math.pi/180)))/(7)
        Angle_2 = (30*distance*math.sin(theta*(math.pi/180)))/(7)

        Data_1 = max(-15,min(15,int(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))))
        Data_2 = max(-15,min(15,int(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))))

        Servo_1 = str(90+Data_1).zfill(3)
        Servo_2 = str(90+Data_1).zfill(3)
        Servo_3 = str(90-Data_2).zfill(3)
        Servo_4 = str(90-Data_2).zfill(3)
        Servo_5 = str(90).zfill(3)
        Servo_6 = str(90).zfill(3)

        portion = 5
    
    elif(theta < 360):
        theta = theta - 300
        if(portion!=6 and portion !=0):
            temp = prev_ang1
            prev_ang1 = prev_ang2
            prev_ang2 = temp 
   
        Angle_1 = (30*distance*math.sin((60-theta)*(math.pi/180)))/(7)
        Angle_2 = (30*distance*math.sin(theta*(math.pi/180)))/(7)

        Data_1 = max(-15,min(15,int(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))))
        Data_2 = max(-15,min(15,int(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))))

        Servo_1 = str(90).zfill(3)
        Servo_2 = str(90).zfill(3)
        Servo_3 = str(90-Data_1).zfill(3)
        Servo_4 = str(90-Data_1).zfill(3)
        Servo_5 = str(90+Data_2).zfill(3)
        Servo_6 = str(90+Data_2).zfill(3)

        portion = 6
 
    arduino = Servo_1 + Servo_2 + Servo_3 + Servo_4 + Servo_5 + Servo_6 + "*"              #End of command character

    prev_ang1 = Angle_1
    prev_ang2 = Angle_2
    
    return arduino    

def Serial_C(data):

    global s
    s.write(data.encode())          #Send Data to Arduino
    
    
if __name__ == "__main__":
    global j,img2,Left,Right,Top,Bottom,dst,thresh1,frame,Points,m,x_cor,y_cor,i

    while(True):
        print("Press 1 for Ball Balncing")
        print("Press 2 for X-Translation")
        print("Press 3 for Y-Translation")
        print("Press 4 for Z-Translation")
        print("Press 5 for X-Rotation")
        print("Press 6 for Y-Rotation")
        print("Press 7 for Z-Rotation")
        print("Press 9 to Leave")

        choice = int(input("Your Choice Please :"))
        if(choice == 2):
            Translate_X()
            
        elif(choice == 3):
            Translate_Y()
            
        elif(choice == 4):
            Translate_Z()

        elif(choice == 5):
            Rotate_X()

        elif(choice == 6):
            Rotate_Y()
        
        elif(choice == 7):
            Rotate_Z()

        elif(choice == 9):
            break
            
        elif(choice == 1):
            while(True):
                j=j+1
          
                # Capture frame-by-frame
                ret, frame = cap.read()  # ret = 1 if the video is captured; frame is the image

                gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
                ret,thresh = cv2.threshold(gray,100,255,cv2.THRESH_BINARY_INV)
                (_,contour,hierarchy)=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  #Contours for Platform  
                
                if len(contour) != 0:
                        c = max(contour, key = cv2.contourArea) # find the largest contour
                        img2=cv2.drawContours(frame, c, -1, color, thickness) # draw largest contour

                        if(j>=25):          #From 25th Frame for settling the image
                
                            Platform(c)     #Make Platform Contours
            
                        if(j>=25):

                            Ball_Track()    #Make Ball Track Contours
                            
                            cv2.circle(img2, Left, 8, (0, 0, 255), -1)        #Points (Extreme Display)
                            cv2.circle(img2, Right, 8, (0, 255, 0), -1)
                            cv2.circle(img2, Top, 8, (255, 0, 0), -1)
                            cv2.circle(img2, Bottom, 8, (0, 255, 0), -1)

                            #cv2.circle(dst,(i[0],i[1]), 8, (0, 255, 0), -1)
                                
                            cv2.imshow('Original View',img2)                  #Display all 3 views
                            cv2.imshow('B&W',thresh1)
                            cv2.imshow('Tracking',dst)

                            
                # Display the resulting image
                #cv2.imshow('Contour',img3)
                if cv2.waitKey(1) & 0xFF == ord('q'):  # press q to quit    
                   break
                    
            # When everything done, release the capture
            cap.release()
            cv2.destroyAllWindows()
            
        else: print("Invalid Choice")
