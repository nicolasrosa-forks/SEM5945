-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

if (sim_call_type==sim.syscb_init) then
    -- Check if the required ROS plugin is there:
    moduleName=0
    moduleVersion=0
    index=0
    pluginNotFound=true

    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='Ros') then
            pluginNotFound=false
        end
        index=index+1
    end
    
    -- Retrieve handles
    leftMotorHandle  = sim.getObjectHandle('left_motor') 
    rightMotorHandle = sim.getObjectHandle('right_motor')
    vehicleHandle    = sim.getObjectHandle('AMR_Full')
    frontSonarHandle = sim.getObjectHandle('front_sonar')
    rightSonarHandle = sim.getObjectHandle('right_sonar')
    leftSonarHandle = sim.getObjectHandle('left_sonar')

    -- Declaring the signals
    sim.setFloatSignal('frontSonarSignal',0.0)
    sim.setFloatSignal('rightSonarSignal',0.0)
    sim.setFloatSignal('leftSonarSignal',0.0)
    sim.setFloatSignal('rightMotorSignal',0.0)
    sim.setFloatSignal('leftMotorSignal',0.0)

    -- Ok now launch the ROS client application:
    if (not pluginNotFound) then
        -- Publisher
        simExtROS_enablePublisher('vehicle/frontSonar',1,simros_strmcmd_get_float_signal,-1,-1,'frontSonarSignal')
        simExtROS_enablePublisher('vehicle/leftSonar',1,simros_strmcmd_get_float_signal,-1,-1,'leftSonarSignal')
        simExtROS_enablePublisher('vehicle/rightSonar',1,simros_strmcmd_get_float_signal,-1,-1,'rightSonarSignal')
        simExtROS_enablePublisher('vehicle/odometry',1,simros_strmcmd_get_odom_data,vehicleHandle,-1,'')
        -- Subscriber
        simExtROS_enableSubscriber('vehicle/motorLeftSpeed',1,simros_strmcmd_set_float_signal ,-1,-1,'leftMotorSignal')
        simExtROS_enableSubscriber('vehicle/motorRightSpeed',1,simros_strmcmd_set_float_signal  ,-1,-1,'rightMotorSignal')
    end

end


if (sim_call_type==sim.syscb_actuation) then
    leftMotorVal = sim.getFloatSignal('leftMotorSignal')
    rightMotorVal = sim.getFloatSignal('rightMotorSignal')

    if (math.abs(leftMotorVal) > 10) then
        leftMotorVal = 10*math.abs(leftMotorVal)/leftMotorVal
    end
    if (math.abs(rightMotorVal) > 10) then
        rightMotorVal = 10*math.abs(rightMotorVal)/rightMotorVal
    end
    sim.setJointTargetVelocity(leftMotorHandle,leftMotorVal)
    sim.setJointTargetVelocity(rightMotorHandle,rightMotorVal)
end


if (sim_call_type==sim.syscb_sensing) then
    -- Put your main SENSING code here
    result,distance=sim.readProximitySensor(frontSonarHandle)
    -- Check if distance reading was valid
    if (result~= 1) then
        distance = 0
    end
    sim.setFloatSignal('frontSonarSignal',distance)

    -- Right Sensor
    result,distance=sim.readProximitySensor(rightSonarHandle)
    -- Check if distance reading was valid
    if (result~= 1) then
        distance = 0
    end
    sim.setFloatSignal('rightSonarSignal',distance)

    -- Left Sensor
    result,distance=sim.readProximitySensor(leftSonarHandle)
    -- Check if distance reading was valid
    if (result~= 1) then
        distance = 0
    end
    sim.setFloatSignal('leftSonarSignal',distance)


    
end


if (sim_call_type==sim.syscb_cleanup) then

    -- Put some restoration code here

end