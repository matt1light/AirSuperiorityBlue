# Make sure to have the server side running in CoppeliaSim:
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,leftJoint=sim.simxGetObjectHandle(clientID,"Left_joint",sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Connected to left joint')
    else:
        print ('Remote API function call returned with error code: ',res)
    res,rightJoint=sim.simxGetObjectHandle(clientID,"Right_joint",sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Connected to left joint')
    else:
        print ('Remote API function call returned with error code: ',res)

    res,ballSensor=sim.simxGetObjectHandle(clientID,"Ball_Sensor",sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Connected to ballSensor')
    else:
        print ('Remote API function call returned with error code: ',res)


    res = sim.simxSetJointTargetVelocity(clientID, leftJoint, 1, sim.simx_opmode_oneshot)
    while True:
        print('Still connected')
        res, detectionState, Data = sim.simxReadVisionSensor(clientID, ballSensor, sim.simx_opmode_blocking)
        print(Data)
        if(Data[0][9]<0.3):
            res = sim.simxSetJointTargetVelocity(clientID, rightJoint, 1, sim.simx_opmode_oneshot)
        else:
            res = sim.simxSetJointTargetVelocity(clientID, rightJoint, 0, sim.simx_opmode_oneshot)


    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
