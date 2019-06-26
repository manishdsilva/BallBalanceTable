import numpy as np
import math
portion = 1
Kp=Kd=Ki=1
prev_ang1=prev_ang2=0
def abc(dy,dx):
    global portion,Kp,Kd,Ki,prev_ang1,prev_ang2
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
        Servo_2 = str(90+Data_2).zfill(3)
        Servo_3 = str(90).zfill(3)
        Servo_4 = str(90).zfill(3)
        Servo_5 = str(90-Data_1).zfill(3)
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
        Servo_2 = str(90+Data_1).zfill(3)
        Servo_3 = str(90-Data_2).zfill(3)
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
        Servo_3 = str(90-Data_1).zfill(3)
        Servo_4 = str(90+Data_1).zfill(3)
        Servo_5 = str(90+Data_2).zfill(3)
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
        Servo_2 = str(90-Data_2).zfill(3)
        Servo_3 = str(90).zfill(3)
        Servo_4 = str(90).zfill(3)
        Servo_5 = str(90+Data_1).zfill(3)
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
        Servo_2 = str(90-Data_1).zfill(3)
        Servo_3 = str(90+Data_2).zfill(3)
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
        Servo_3 = str(90+Data_1).zfill(3)
        Servo_4 = str(90-Data_1).zfill(3)
        Servo_5 = str(90-Data_2).zfill(3)
        Servo_6 = str(90+Data_2).zfill(3)

        portion = 6
 
    arduino = Servo_1 + Servo_2 + Servo_3 + Servo_4 + Servo_5 + Servo_6 + "*"              #End of command character

    prev_ang1 = Angle_1
    prev_ang2 = Angle_2
    
    return arduino
print(abc(-3,10))
    
