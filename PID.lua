
function sysCall_init()

    Servo1=sim.getObjectHandle('Servo1_rotor') 
    Servo2=sim.getObjectHandle('Servo2_rotor') 
 
    Servo3=sim.getObjectHandle('Servo3_rotor')
    Servo4=sim.getObjectHandle('Servo4_rotor') 

    Servo5=sim.getObjectHandle('Servo5_rotor')  
    Servo6=sim.getObjectHandle('Servo6_rotor')
    sphere  =sim.getObjectHandle('Sphere')

    pf    =sim.getObjectHandle('Platform')
    
    prev_ang1 = 0
    prev_ang2 = 0

    Kp = 1
    Kd = -40
    Ki = 0.1
    portion = 0
    
    
end

function sysCall_actuation()
    
    xyz_cor = sim.getObjectPosition(sphere,pf)
    --xyz_tar = sim.getObjectPosition(target,pf)

    dx = ((xyz_cor[2])*100) 
    dy = ((xyz_cor[3])*100) 
    print(dx)
    print(dy)
    distance = math.sqrt ( dx * dx + dy * dy )
    theta= math.deg(math.atan2(dy,dx))
    
    
    if(theta < 0) then
    theta = 360 + theta
    end

    --print(distance)

    if(theta < 60) then
    if(portion ~= 1 and portion ~= 0) then
    temp = prev_ang1
    prev_ang1 = prev_ang2
    prev_ang2 = temp 
    end
    Angle_1 = (30*distance*math.sin((60-theta)*(math.pi/180)))/(7)
    Angle_2 = (30*distance*math.sin(theta*(math.pi/180)))/(7)
    sim.setJointTargetPosition(Servo5,-(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))*math.pi/180)
    sim.setJointTargetPosition(Servo6,(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))*math.pi/180)
    sim.setJointTargetPosition(Servo1,-(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))*math.pi/180)
    sim.setJointTargetPosition(Servo2,(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))*math.pi/180)
    portion = 1

    elseif(theta < 120) then
    theta = theta - 60
    if(portion~=2 and portion ~=0) then
    temp = prev_ang1
    prev_ang1 = prev_ang2
    prev_ang2 = temp 
    end
    Angle_1 = (30*distance*math.sin((60-theta)*(math.pi/180)))/(7)
    Angle_2 = (30*distance*math.sin(theta*(math.pi/180)))/(7)
    sim.setJointTargetPosition(Servo1,-(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))*math.pi/180)
    sim.setJointTargetPosition(Servo2,(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))*math.pi/180)
    sim.setJointTargetPosition(Servo3,-(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))*math.pi/180)
    sim.setJointTargetPosition(Servo4,(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))*math.pi/180)
    portion = 2

    elseif(theta < 180) then
    theta = theta - 120
    if(portion~=3 and portion ~=0) then
    temp = prev_ang1
    prev_ang1 = prev_ang2
    prev_ang2 = temp 
    end
    Angle_1 = (30*distance*math.sin((60-theta)*(math.pi/180)))/(7)
    Angle_2 = (30*distance*math.sin(theta*(math.pi/180)))/(7)
    sim.setJointTargetPosition(Servo3,-(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))*math.pi/180)
    sim.setJointTargetPosition(Servo4,(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))*math.pi/180)
    sim.setJointTargetPosition(Servo5,(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))*math.pi/180)
    sim.setJointTargetPosition(Servo6,-(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))*math.pi/180)
    portion = 3

    elseif(theta < 240) then
    theta = theta - 180
    if(portion~=4 and portion ~=0) then
    temp = prev_ang1
    prev_ang1 = prev_ang2
    prev_ang2 = temp 
    end
    Angle_1 = (30*distance*math.sin((60-theta)*(math.pi/180)))/(7)
    Angle_2 = (30*distance*math.sin(theta*(math.pi/180)))/(7)
    sim.setJointTargetPosition(Servo5,(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))*math.pi/180)
    sim.setJointTargetPosition(Servo6,-(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))*math.pi/180)
    sim.setJointTargetPosition(Servo1,(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))*math.pi/180)
    sim.setJointTargetPosition(Servo2,-(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))*math.pi/180)
    portion = 4

    elseif(theta < 300) then
    theta = theta - 240
    if(portion~=5 and portion ~=0) then
    temp = prev_ang1
    prev_ang1 = prev_ang2
    prev_ang2 = temp 
    end
    Angle_1 = (30*distance*math.sin((60-theta)*(math.pi/180)))/(7)
    Angle_2 = (30*distance*math.sin(theta*(math.pi/180)))/(7)
    sim.setJointTargetPosition(Servo1,(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))*math.pi/180)
    sim.setJointTargetPosition(Servo2,-(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))*math.pi/180)
    sim.setJointTargetPosition(Servo3,(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))*math.pi/180)
    sim.setJointTargetPosition(Servo4,-(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))*math.pi/180)
    portion = 5
    
    elseif(theta < 360) then
    theta = theta - 300
    if(portion~=6 and portion ~=0) then
    temp = prev_ang1
    prev_ang1 = prev_ang2
    prev_ang2 = temp 
    end
    Angle_1 = (30*distance*math.sin((60-theta)*(math.pi/180)))/(7)
    Angle_2 = (30*distance*math.sin(theta*(math.pi/180)))/(7)
    sim.setJointTargetPosition(Servo3,(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))*math.pi/180)
    sim.setJointTargetPosition(Servo4,-(Kp*Angle_1+Kd*(prev_ang1-Angle_1)+Ki*(prev_ang1+Angle_1))*math.pi/180)
    sim.setJointTargetPosition(Servo5,-(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))*math.pi/180)
    sim.setJointTargetPosition(Servo6,(Kp*Angle_2+Kd*(prev_ang2-Angle_2)+Ki*(prev_ang2+Angle_2))*math.pi/180)
    portion = 6
    end

    prev_ang1 = Angle_1
    prev_ang2 = Angle_2
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- You can define additional system calls here:
--[[
function sysCall_suspend()
end

function sysCall_resume()
end

    function sysCall_dynCallback(inData)
    end

function sysCall_jointCallback(inData)
    return outData
end

function sysCall_contactCallback(inData)
    return outData
end

function sysCall_beforeCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be copied")
    end
end

function sysCall_afterCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was copied")
    end
end

function sysCall_beforeDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be deleted")
    end
    -- inData.allObjects indicates if all objects in the scene will be deleted
end

function sysCall_afterDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was deleted")
    end
    -- inData.allObjects indicates if all objects in the scene were deleted
end

function sysCall_afterCreate(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..value.." was created")
    end
end
--]]
