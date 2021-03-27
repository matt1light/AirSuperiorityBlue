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
import time
import random

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

class WatchDog:
    def __init__(self, clientID, ball):
        # Fetch the object handle for the ball
        self.ball = ball

        # The location of the ball in the last game loop
        self.last_ball_position = sim.simxGetObjectPosition(clientID, self.ball, -1, sim.simx_opmode_blocking)[1]

        # The last time that the ball was moved, if this is more than the timeout value
        self.last_moved_time = time.time()

        # the number of seconds until the ball's location is reset
        self.timeout = 10

        self.clientID = clientID

    def update_watchdog(self):
        # Get the current position of the ball
        new_position = sim.simxGetObjectPosition(self.clientID, self.ball, -1, sim.simx_opmode_blocking)[1]
        # Get the current time
        current_time = time.time()
        # If the ball has moved then update the stored ball location, and set the last moved time to the current time
        rounded_new_position = [round(elem, 3) for elem in new_position]
        rounded_last_position = [round(elem, 3) for elem in self.last_ball_position]

        if (rounded_new_position != rounded_last_position):
            self.last_ball_position = new_position
            self.last_moved_time = current_time
        else:
            # If the ball hasn't moved, check if the time since the last move is more than the timeout value
            time_elapsed = current_time - self.last_moved_time
            print(time_elapsed)
            if (time_elapsed >= self.timeout):
                # If the ball has not moved in timeout number of seconds, we reset the location of the ball
                self.set_initial_positions()

    def set_initial_positions(self):
        sim.simxSetObjectPosition(
            self.clientID, self.ball, -1, [random.uniform(-1,1), random.uniform(-0.5, 0.5), 0.05], sim.simx_opmode_blocking
        )

class Scoreboard:
    def __init__(self, clientID):
        self.robotAScore = 0
        self.robotBScore = 0
        self.dialog = 0
        self.clientID = clientID
        self._updateScoreBoard()

    def scoreA(self):
        self.robotAScore += 1
        self._updateScoreBoard()

    def scoreB(self):
        self.robotBScore += 1
        self._updateScoreBoard()

    def _updateScoreBoard(self):
        if (self.dialog != 0):
            sim.simxEndDialog(self.clientID,self.dialog,sim.simx_opmode_oneshot)
        _, self.dialog, _ = sim.simxDisplayDialog(self.clientID, " ", f'A: {self.robotAScore} || B: {self.robotBScore}', 0, " ", None , None, sim.simx_opmode_blocking)

class GameState:
    """
    """

    def __init__(self, clientID):
        self.res, self.ball = sim.simxGetObjectHandle(
            clientID, "Ball", sim.simx_opmode_blocking
        )
        self.res, self.goalDetectionSensorA = sim.simxGetObjectHandle(
            clientID, "Goal_Detection_Sensor_A", sim.simx_opmode_blocking
        )
        self.res, self.goalDetectionSensorB = sim.simxGetObjectHandle(
            clientID, "Goal_Detection_Sensor_B", sim.simx_opmode_blocking
        )
        # self.set_initial_positions()
        self.dialog = 0
        self.messagebox = 0

        self.watchdog = WatchDog(clientID, self.ball)

        self.scoreBoard = Scoreboard(clientID)

    def gameLoop(self):
        if self.messagebox == 0:
            sim.simxEndDialog(clientID,self.dialog,sim.simx_opmode_oneshot)
            self.messagebox = 1

        _, goalScoredAState, goalScoredAData = sim.simxReadVisionSensor(
            clientID, self.goalDetectionSensorA, sim.simx_opmode_blocking
        )
        _, goalScoredBState, goalScoredBData = sim.simxReadVisionSensor(
            clientID, self.goalDetectionSensorB, sim.simx_opmode_blocking
        )

        self.watchdog.update_watchdog()

        goalScoredA = goalScoredAData[0][9] < 0.3
        goalScoredB = goalScoredBData[0][9] < 0.3
        if goalScoredA:
            self.scoreBoard.scoreA()
            self.set_initial_positions()
        if goalScoredB:
            self.scoreBoard.scoreB()
            self.set_initial_positions()

    def get_initial_positions(self):
        self.initial_ball_position = sim.simxGetObjectPosition(clientID, self.ball, -1, sim.simx_opmode_blocking)[1]

    def set_initial_positions(self):
        sim.simxSetObjectPosition(
            clientID, self.ball, -1, [random.uniform(-1,1), random.uniform(-0.5, 0.5), 0.05], sim.simx_opmode_blocking
        )

class SoccerRobot:
    """

    """

    def __init__(self, clientID, robotLetter):
        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        self.res, self.leftJoint = sim.simxGetObjectHandle(
            clientID, f"Left_joint_{robotLetter}", sim.simx_opmode_blocking
        )
        self.res, self.rightJoint = sim.simxGetObjectHandle(
            clientID, f"Right_joint_{robotLetter}", sim.simx_opmode_blocking
        )
        self.res, self.ballSensor = sim.simxGetObjectHandle(
            clientID, f"Ball_Sensor_{robotLetter}", sim.simx_opmode_blocking
        )
        self.res, self.proximitySensor = sim.simxGetObjectHandle(
            clientID, f"Proximity_sensor_{robotLetter}", sim.simx_opmode_blocking
        )
        self.res, self.goalLineSensor = sim.simxGetObjectHandle(
            clientID, f"Goal_Line_Sensor_{robotLetter}", sim.simx_opmode_blocking
        )

        sim.simxSetJointTargetVelocity(
            clientID, self.leftJoint, 1, sim.simx_opmode_oneshot
        )
        self.objects = [
            self.leftJoint,
            self.rightJoint,
            self.ballSensor,
            self.proximitySensor,
            self.goalLineSensor,
        ]

    

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


        facingBall = ballSensorData[0][9] < 0.3
        touchingBall = proxDetectionState and detectedPoint[2] < 0.15
        facingGoalLine = goalLineSensorData[0][9] < 0.3

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
    bot = SoccerRobot(clientID=clientID, robotLetter="A")
    bot2 = SoccerRobot(clientID=clientID, robotLetter="B")
    gamestate = GameState(clientID=clientID)

    while True:
        bot2.update_robot(clientID=clientID)
        bot.update_robot(clientID=clientID)
        gamestate.gameLoop()

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID, "Hello CoppeliaSim!", sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print("Failed connecting to remote API server")
print("Program ended")
