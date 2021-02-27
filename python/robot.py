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
    print("--------------------------------------------------------------")
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print("Make sure both are in the same folder as this file,")
    print('or appropriately adjust the file "sim.py"')
    print("--------------------------------------------------------------")
    print("")

import time


class SoccerRobot:
    """

    """

    def __init__(self, clientID):
        # getting the initial body position and store it for later use
        # when resetting
        self.res, self.body = sim.simxGetObjectHandle(
            clientID, "Body", sim.simx_opmode_blocking
        )
        self.res, self.ball = sim.simxGetObjectHandle(
            clientID, "Ball", sim.simx_opmode_blocking
        )
        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        self.res, self.leftJoint = sim.simxGetObjectHandle(
            clientID, "Left_joint", sim.simx_opmode_blocking
        )
        self.res, self.rightJoint = sim.simxGetObjectHandle(
            clientID, "Right_joint", sim.simx_opmode_blocking
        )
        self.res, self.ballSensor = sim.simxGetObjectHandle(
            clientID, "Ball_Sensor", sim.simx_opmode_blocking
        )
        self.res, self.proximitySensor = sim.simxGetObjectHandle(
            clientID, "Proximity_sensor", sim.simx_opmode_blocking
        )
        self.res, self.goalLineSensor = sim.simxGetObjectHandle(
            clientID, "Goal_Line_Sensor", sim.simx_opmode_blocking
        )

        self.res, self.goalDetectionSensorA = sim.simxGetObjectHandle(
            clientID, "Goal_Detection_Sensor_A", sim.simx_opmode_blocking
        )
        sim.simxSetJointTargetVelocity(
            clientID, self.leftJoint, 1, sim.simx_opmode_oneshot
        )
        self.objects = [
            self.body,
            self.leftJoint,
            self.rightJoint,
            self.ballSensor,
            self.proximitySensor,
            self.goalLineSensor,
        ]
        self.objects.append(
            sim.simxGetObjectHandle(clientID, "Square", sim.simx_opmode_blocking)[1]
        )
        self.objects.append(
            sim.simxGetObjectHandle(clientID, "Cubiod", sim.simx_opmode_blocking)[1]
        )
        self.objects.append(
            sim.simxGetObjectHandle(clientID, "Cuboid1", sim.simx_opmode_blocking)[1]
        )
        self.objects.append(
            sim.simxGetObjectHandle(clientID, "Right_wheel", sim.simx_opmode_blocking)[1]
        )
        self.objects.append(
            sim.simxGetObjectHandle(clientID, "Left_wheel", sim.simx_opmode_blocking)[1]
        )
        self.objects.append(
            sim.simxGetObjectHandle(clientID, "Castor_Joint", sim.simx_opmode_blocking)[1]
        )
        self.get_initial_positions()

    def get_initial_positions(self):
        self.initial_ball_position = sim.simxGetObjectPosition(clientID, self.ball, -1, sim.simx_opmode_blocking)[1]
        self.initial_quaternions = [
            sim.simxGetObjectQuaternion(clientID, obj, -1, sim.simx_opmode_blocking)[1]
            for obj in self.objects
        ]
        self.initial_positions = [
            sim.simxGetObjectPosition(clientID, obj, -1, sim.simx_opmode_blocking)[1]
            for obj in self.objects
        ]

    def set_initial_positions(self):
        #sim.c_PauseSimulation(clientID, sim.simx_opmode_oneshot)
        """
        self.res = sim.simxSetObjectPosition(
            clientID, self.body, -1, self.initialPosition, sim.simx_opmode_blocking
        )
        """
        sim.simxSetObjectPosition(
            clientID, self.ball, -1, self.initial_ball_position, sim.simx_opmode_blocking
        )
        """
        [
            sim.simxSetObjectPosition(
                clientID, obj, -1, init_pos, sim.simx_opmode_blocking
            )
            for obj, init_pos in zip(self.objects, self.initial_positions)
        ]
        """
        """
        [
            sim.simxSetObjectQuaternion(
                clientID, obj, -1, init_pos, sim.simx_opmode_blocking
            )
            for obj, init_pos in zip(self.objects, self.initial_quaternions)
        ]
        """

    def update_robot(self, clientID) -> bool:
        _, ballDetectionState, ballSensorData = sim.simxReadVisionSensor(
            clientID, self.ballSensor, sim.simx_opmode_blocking
        )
        (_, proxDetectionState, detectedPoint, _, _,) = sim.simxReadProximitySensor(
            clientID, self.proximitySensor, sim.simx_opmode_blocking
        )
        _, goalLineSensorState, goalLineSensorData = sim.simxReadVisionSensor(
            clientID, self.goalLineSensor, sim.simx_opmode_blocking
        )
        _, goalScoredAState, goalScoredAData = sim.simxReadVisionSensor(
            clientID, self.goalDetectionSensorA, sim.simx_opmode_blocking
        )

        facingBall = ballSensorData[0][9] < 0.3
        touchingBall = proxDetectionState and detectedPoint[2] < 0.15
        facingGoalLine = goalLineSensorData[0][9] < 0.3
        goalScoredA = goalScoredAData[0][9] < 0.3
        if goalScoredA:
            print(goalScoredA)
        if goalScoredA and facingGoalLine:
            self.set_initial_positions()

        if facingBall:
            if not touchingBall:
                _ = sim.simxSetJointTargetVelocity(
                    clientID, self.rightJoint, 5, sim.simx_opmode_oneshot
                )
                _ = sim.simxSetJointTargetVelocity(
                    clientID, self.leftJoint, 5, sim.simx_opmode_oneshot
                )
            else:
                _ = sim.simxSetJointTargetVelocity(
                    clientID, self.rightJoint, 5, sim.simx_opmode_oneshot
                )
                _ = sim.simxSetJointTargetVelocity(
                    clientID, self.leftJoint, 5, sim.simx_opmode_oneshot
                )
                print("Have the ball")
                if facingGoalLine:
                    _ = sim.simxSetJointTargetVelocity(
                        clientID, self.rightJoint, 5, sim.simx_opmode_oneshot
                    )
                    _ = sim.simxSetJointTargetVelocity(
                        clientID, self.leftJoint, 5, sim.simx_opmode_oneshot
                    )
                else:
                    _ = sim.simxSetJointTargetVelocity(
                        clientID, self.rightJoint, -0.5, sim.simx_opmode_oneshot
                    )
                    _ = sim.simxSetJointTargetVelocity(
                        clientID, self.leftJoint, 0.5, sim.simx_opmode_oneshot
                    )

        else:
            _ = sim.simxSetJointTargetVelocity(
                clientID, self.rightJoint, -0.5, sim.simx_opmode_oneshot
            )
            _ = sim.simxSetJointTargetVelocity(
                clientID, self.leftJoint, 0.5, sim.simx_opmode_oneshot
            )

        return False


print("Program started")
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart(
    "127.0.0.1", 19999, True, True, 5000, 5
)  # Connect to CoppeliaSim
if clientID != -1:
    print("Connected to remote API server")

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    bot = SoccerRobot(clientID=clientID)

    while True:
        bot.update_robot(clientID=clientID)

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID, "Hello CoppeliaSim!", sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print("Failed connecting to remote API server")
print("Program ended")
