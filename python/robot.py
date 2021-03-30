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
        self.last_ball_position = sim.simxGetObjectPosition(clientID, self.ball, -1, sim.simx_opmode_streaming)[1]

        # The last time that the ball was moved, if this is more than the timeout value
        self.last_moved_time = time.time()

        # the number of seconds until the ball's location is reset
        self.timeout = 20

        self.clientID = clientID

    def update_watchdog(self):
        # Get the current position of the ball
        new_position = sim.simxGetObjectPosition(self.clientID, self.ball, -1, sim.simx_opmode_buffer)[1]
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
            if (time_elapsed >= self.timeout):
                # If the ball has not moved in timeout number of seconds, we reset the location of the ball
                print(f"Ball did not move for {self.timeout} seconds, so its location was reset")
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
        self.robotAScore +=1
        self._updateScoreBoard()

    def scoreB(self):
        self.robotBScore += 1
        self._updateScoreBoard()

    def _updateScoreBoard(self):
        if (self.dialog != 0):
            sim.simxEndDialog(self.clientID, self.dialog, sim.simx_opmode_blocking)
        _, self.dialog, _ = sim.simxDisplayDialog(self.clientID, " ", f'Blue: {self.robotAScore} || Red: {self.robotBScore}', 0, " ", None , None, sim.simx_opmode_blocking)

    def close(self):
        if (self.dialog != 0):
            sim.simxEndDialog(self.clientID,self.dialog,sim.simx_opmode_oneshot)

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
        self.bot = SoccerRobot(clientID=clientID, robotLetter="A", direction=-1)
        self.bot2 = SoccerRobot(clientID=clientID, robotLetter="B", direction=-1)
        self.clientID = clientID

        self.set_initial_positions()
        self.dialog = 0
        self.messagebox = 0

        self.watchdog = WatchDog(clientID, self.ball)

        self.scoreBoard = Scoreboard(clientID)

        self.maxScore = 5
        self.quit = False
        self.opmode = sim.simx_opmode_streaming

    def start(self):
        while (not self.quit):
            self.gameLoop()

    def gameLoop(self):
        self.bot2.update_robot()
        self.bot.update_robot()
        if self.messagebox == 0:
            sim.simxEndDialog(clientID,self.dialog,sim.simx_opmode_oneshot)
            self.messagebox = 1


        _, goalScoredAState, goalScoredAData = sim.simxReadVisionSensor(
            clientID, self.goalDetectionSensorA, self.opmode
        )
        _, goalScoredBState, goalScoredBData = sim.simxReadVisionSensor(
            clientID, self.goalDetectionSensorB, self.opmode
        )
        self.opmode = sim.simx_opmode_buffer

        self.watchdog.update_watchdog()
        try:
            goalScoredA = goalScoredAData[0][9] < 0.3
            goalScoredB = goalScoredBData[0][9] < 0.3
        except IndexError:
            return

        if goalScoredA:
            self.scoreBoard.scoreA()
            self.set_initial_positions()
        if goalScoredB:
            self.scoreBoard.scoreB()
            self.set_initial_positions()
        if (self.scoreBoard.robotAScore >= self.maxScore):
            self.endGame("A")
        if (self.scoreBoard.robotBScore >= self.maxScore):
            self.endGame("B")

    def endGame(self, winner):
        self.scoreBoard.close()
        _winner = ""
        if (winner == "A"):
            _winner = "Blue"
        if (winner == "B"):
            _winner = "Red"
        _, self.dialog, _ = sim.simxDisplayDialog(self.clientID, " ", f'Robot {_winner} wins\nBlue: {self.scoreBoard.robotAScore} || Red: {self.scoreBoard.robotBScore}', 0, " ", None , None, sim.simx_opmode_oneshot)
        self.quit = True
        self.bot.end()
        self.bot2.end()


    def get_initial_positions(self):
        self.initial_ball_position = sim.simxGetObjectPosition(clientID, self.ball, -1, sim.simx_opmode_blocking)[1]

    def set_initial_positions(self):
        sim.simxSetObjectPosition(
            clientID, self.ball, -1, [random.uniform(-1,1), random.uniform(-0.5, 0.5), 0.05], sim.simx_opmode_oneshot
        )

class SoccerRobot:
    """

    """
    def __init__(self, clientID, robotLetter, direction):
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
        self.res, self.ballSensor2 = sim.simxGetObjectHandle(
            clientID, f"Ball_Sensor_{robotLetter}0", sim.simx_opmode_blocking
        )
        self.res, self.ballSensor3 = sim.simxGetObjectHandle(
            clientID, f"Ball_Sensor_{robotLetter}1", sim.simx_opmode_blocking
        )
        self.res, self.ballSensorLeft = sim.simxGetObjectHandle(
            clientID, f"Ball_Sensor_Left_{robotLetter}", sim.simx_opmode_blocking
        )
        self.res, self.ballSensorRight = sim.simxGetObjectHandle(
            clientID, f"Ball_Sensor_Right_{robotLetter}", sim.simx_opmode_blocking
        )
        self.res, self.proximitySensor = sim.simxGetObjectHandle(
            clientID, f"Proximity_sensor_{robotLetter}", sim.simx_opmode_blocking
        )
        self.res, self.goalLineSensor = sim.simxGetObjectHandle(
            clientID, f"Goal_Line_Sensor_{robotLetter}", sim.simx_opmode_blocking
        )
        self.res, self.goalLineSensorLeft = sim.simxGetObjectHandle(
            clientID, f"Goal_Line_Sensor_Left_{robotLetter}", sim.simx_opmode_blocking
        )
        self.res, self.goalLineSensorRight = sim.simxGetObjectHandle(
            clientID, f"Goal_Line_Sensor_Right_{robotLetter}", sim.simx_opmode_blocking
        )

        sim.simxSetJointTargetVelocity(
            clientID, self.leftJoint, 1, sim.simx_opmode_oneshot
        )
        self.clientID = clientID

        self.direction = direction

        self.objects = [
            self.leftJoint,
            self.rightJoint,
            self.ballSensor,
            self.proximitySensor,
            self.goalLineSensor,
        ]
        self.last_state = ""

        self.base_speed = 5
        self.slow_speed = 0.5

        self.last_location = []
        self.last_movement_time = 0
        self.timeout = 10
        self.opmode = sim.simx_opmode_streaming

    def setSpeed(self, leftSpeed, rightSpeed):
        _ = sim.simxSetJointTargetVelocity(
            self.clientID, self.rightJoint, rightSpeed, sim.simx_opmode_oneshot
        )
        _ = sim.simxSetJointTargetVelocity(
            self.clientID, self.leftJoint, leftSpeed, sim.simx_opmode_oneshot
        )




    def update_robot(self) -> bool:
        _, ballDetectionState, ballSensorData = sim.simxReadVisionSensor(
            self.clientID, self.ballSensor, self.opmode
        )
        _, ballDetectionState, ballSensorData2 = sim.simxReadVisionSensor(
            self.clientID, self.ballSensor2, self.opmode
        )
        _, ballDetectionState, ballSensorData3 = sim.simxReadVisionSensor(
            self.clientID, self.ballSensor3, self.opmode
        )
        _, ballDetectionState, ballSensorDataLeft = sim.simxReadVisionSensor(
            self.clientID, self.ballSensorLeft, self.opmode
        )
        _, ballDetectionState, ballSensorDataRight = sim.simxReadVisionSensor(
            self.clientID, self.ballSensorRight, self.opmode
        )
        (_, proxDetectionState, detectedPoint, _, _,) = sim.simxReadProximitySensor(
            self.clientID, self.proximitySensor, self.opmode
        )
        _, goalLineSensorState, goalLineSensorData = sim.simxReadVisionSensor(
            self.clientID, self.goalLineSensor, self.opmode
        )
        _, goalLineSensorState, goalLineSensorDataLeft = sim.simxReadVisionSensor(
            self.clientID, self.goalLineSensorLeft, self.opmode
        )
        _, goalLineSensorState, goalLineSensorDataRight = sim.simxReadVisionSensor(
            self.clientID, self.goalLineSensorRight, self.opmode
        )
        new_position_array = sim.simxGetObjectPosition(self.clientID, self.ballSensor, -1, self.opmode)
        self.opmode = sim.simx_opmode_buffer

        try:
            new_position = new_position_array[1]
            facingBall = ballSensorData[0][9] < 0.3
            ballRight = ballSensorData3[0][9] < 0.3
            ballLeft = ballSensorData2[0][9] < 0.3
            ballSideLeft = ballSensorDataLeft[0][9] < 0.3
            ballSideRight = ballSensorDataRight[0][9] < 0.3
            touchingBall = proxDetectionState and detectedPoint[2] < 0.23
            facingGoalLine = goalLineSensorData[0][9] < 0.3
            goalLineRight = goalLineSensorDataRight[0][9] < 0.3
            goalLineLeft = goalLineSensorDataLeft[0][9] < 0.3
        except IndexError:
            return

        if facingBall:
            if not touchingBall:
                if (self.last_state != "touchingBall"):
                    self.setSpeed(1.3*self.base_speed, 1.3*self.base_speed)
                    self.last_state = "touchingBall"
            else:
                if facingGoalLine:
                    if (self.last_state != "facingGoalLine"):
                        self.setSpeed(self.base_speed, self.base_speed)
                        self.last_state = "facingGoalLine"
                else:
                    if (self.last_state != "findingGoalLine"):
                        self.setSpeed(1.3*self.slow_speed*self.direction, -1.3*self.slow_speed*self.direction)
                        self.last_state = "findingGoalLine"
                if goalLineLeft:
                    self.direction = 1
                    self.last_state = "directionSwitch"
                if goalLineRight:
                    self.direction = -1
                    self.last_state = "directionSwitch"

        else:
            if ballLeft:
                if (self.last_state != "ballLeft"):
                    self.setSpeed(self.slow_speed, self.slow_speed*2)
                    self.last_state = "ballLeft"

            elif ballRight:
                if (self.last_state != "ballRight"):
                    self.setSpeed(self.slow_speed*2, self.slow_speed)
                    self.last_state = "ballRight"
            else:
                if (self.last_state != "findingBall"):
                    self.setSpeed(self.slow_speed*self.direction, -self.slow_speed*self.direction)
                    self.last_state = "findingBall"
            if ballSideLeft:
                self.direction = 1
                self.last_state = "switchDirection"
            elif ballSideRight:
                self.direction = -1
                self.last_state = "switchDirection"


        # Get the current time
        current_time = time.time()
        # If the ball has moved then update the stored ball location, and set the last moved time to the current time
        rounded_new_position = [round(elem, 1) for elem in new_position]
        rounded_last_position = [round(elem, 1) for elem in self.last_location]

        if (rounded_new_position != rounded_last_position):
            self.last_location = new_position
            self.last_movement_time = current_time
        else:
            # If the ball hasn't moved, check if the time since the last move is more than the timeout value
            time_elapsed = current_time - self.last_movement_time
            if (time_elapsed >= self.timeout):
                # If the ball has not moved in timeout number of seconds, we reset the location of the ball
                print(f"Robot did not move for {self.timeout} seconds, so it reversed")
                self.setSpeed(-90, -90)
                self.direction *= -1
                self.last_state = "reverse"
                time.sleep(0.2)
                self.last_movement_time = time.time()

        return False

    def end(self):
        _ = sim.simxSetJointTargetVelocity(
            self.clientID, self.rightJoint, 0, sim.simx_opmode_oneshot
        )
        _ = sim.simxSetJointTargetVelocity(
            self.clientID, self.leftJoint, 0, sim.simx_opmode_oneshot
        )



print("Program started")
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart(
    "127.0.0.1", 19999, True, True, 5000, 5
)  # Connect to CoppeliaSim
if clientID != -1:
    print("Connected to remote API server")

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    gamestate = GameState(clientID=clientID)

    gamestate.start()

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID, "Hello CoppeliaSim!", sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print("Failed connecting to remote API server")
print("Program ended")
