function sysCall_init()

    Servo1=sim.getObjectHandle('Servo1_rotor') 
    Servo2=sim.getObjectHandle('Servo2_rotor') 
 
    Servo3=sim.getObjectHandle('Servo3_rotor')
    Servo4=sim.getObjectHandle('Servo4_rotor') 

    Servo5=sim.getObjectHandle('Servo5_rotor')  
    Servo6=sim.getObjectHandle('Servo6_rotor')
    sphere  =sim.getObjectHandle('Sphere')

    pf    =sim.getObjectHandle('Platform')
    Dirn =1 
    i=1
    
end

function sysCall_actuation()
    
    if((math.floor(i/100)%2)==1) then 
    Dirn = -1
    else
    Dirn = 1
    end
    Angle= 20
    sim.setJointTargetPosition(Servo5,Dirn*Angle*math.pi/180)
    sim.setJointTargetPosition(Servo6,Dirn*Angle*math.pi/180)
    sim.setJointTargetPosition(Servo1,-Dirn*Angle*math.pi/180)
    sim.setJointTargetPosition(Servo2,-Dirn*Angle*math.pi/180)
    sim.setJointTargetPosition(Servo3,Dirn*Angle*math.pi/180)
    sim.setJointTargetPosition(Servo4,Dirn*Angle*math.pi/180)
    i=i+2
end

