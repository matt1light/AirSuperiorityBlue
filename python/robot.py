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

# Watchdog monitors the state of the game and ensures that the ball is constantly being moved.
# If it isn't moved then the ball should be reset after a certain amount of time
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

        # the clientID used for connecting to the CoppeliaSim API
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

# Scoreboard controls the dialog boxes that display the score throughout the game
class Scoreboard:
    def __init__(self, clientID):
        # The scores start at zero
        self.robotAScore = 0
        self.robotBScore = 0
        # The dialog box starts uninitialized
        self.dialog = 0
        # The client ID is used for connecting to the remoteAPI
        self.clientID = clientID
        # The scoreboard should be initialized on start
        self._updateScoreBoard()

    # increment the score for robot a and update the scoreboard
    def scoreA(self):
        self.robotAScore +=1
        self._updateScoreBoard()

    # increment the score for robot b and update the scoreboard
    def scoreB(self):
        self.robotBScore += 1
        self._updateScoreBoard()

    # If there is a dialog box, destroy it and create a new one with the current score, if there isn't one no need to destroy it
    def _updateScoreBoard(self):
        if (self.dialog != 0):
            sim.simxEndDialog(self.clientID, self.dialog, sim.simx_opmode_blocking)
        _, self.dialog, _ = sim.simxDisplayDialog(self.clientID, " ", f'Blue: {self.robotAScore} || Red: {self.robotBScore}', 0, " ", None , None, sim.simx_opmode_blocking)

    # Close the dialog box
    def close(self):
        if (self.dialog != 0):
            sim.simxEndDialog(self.clientID,self.dialog,sim.simx_opmode_oneshot)

# Gamestate is used to run the main game loop
# Robots, scoreboard, and watchdog, are updated through a game loop repeatedly 
# additionaly the gamestate keeps track of whether or not there is a goal scored
class GameState:
    """
    """

    def __init__(self, clientID):
        # Get the handle of the ball
        self.res, self.ball = sim.simxGetObjectHandle(
            clientID, "Ball", sim.simx_opmode_blocking
        )
        # Get the handle of the goal detection sensors
        self.res, self.goalDetectionSensorA = sim.simxGetObjectHandle(
            clientID, "Goal_Detection_Sensor_A", sim.simx_opmode_blocking
        )
        self.res, self.goalDetectionSensorB = sim.simxGetObjectHandle(
            clientID, "Goal_Detection_Sensor_B", sim.simx_opmode_blocking
        )
        # Initialize two robots
        self.bot = SoccerRobot(clientID=clientID, robotLetter="A", direction=-1)
        self.bot2 = SoccerRobot(clientID=clientID, robotLetter="B", direction=-1)
        self.clientID = clientID

        # Move the ball to a random initial position
        self.set_initial_positions()
        # reset the dialog boxes
        self.dialog = 0
        self.messagebox = 0

        # Initialize the watchdog timer that controls the ball
        self.watchdog = WatchDog(clientID, self.ball)

        # Initialize a scoreboard
        self.scoreBoard = Scoreboard(clientID)

        # Max score determines how many goals are required to win
        self.maxScore = 5
        # when quit is true the gameloop will exit
        self.quit = False
        # the opmode needs to be streaming for the first loop, but after that it should be buffer
        self.opmode = sim.simx_opmode_streaming

    def start(self):
        # Run the gameloop until quit is true
        while (not self.quit):
            self.gameLoop()

    def gameLoop(self):
        # Run the update script on each robot, this is what allows the robots to change their behavior based on sensor data
        self.bot2.update_robot()
        self.bot.update_robot()
        if self.messagebox == 0:
            sim.simxEndDialog(clientID,self.dialog,sim.simx_opmode_oneshot)
            self.messagebox = 1

        # Read the goal detection sensors
        _, goalScoredAState, goalScoredAData = sim.simxReadVisionSensor(
            clientID, self.goalDetectionSensorA, self.opmode
        )
        _, goalScoredBState, goalScoredBData = sim.simxReadVisionSensor(
            clientID, self.goalDetectionSensorB, self.opmode
        )
        # Set the operation mode to buffer to increase performance
        self.opmode = sim.simx_opmode_buffer
        # Run the watchdog timer update script
        self.watchdog.update_watchdog()
        try:
            # Determine if a goal was scored by reading sensor data from the vision sensors
            goalScoredA = goalScoredAData[0][9] < 0.3
            goalScoredB = goalScoredBData[0][9] < 0.3
        # On the first run, the asynchronous nature of retrieving the sensor data, means goalScoredXData could not be defined and result in an error
        # If this is the case, we will skip the score updates this loop
        except IndexError:
            return

        # If a goal is scored we update the appropriate scoreboard and set the randomize the position of the ball
        if goalScoredA:
            self.set_initial_positions()
            self.scoreBoard.scoreA()
        if goalScoredB:
            self.set_initial_positions()
            self.scoreBoard.scoreB()
        # Check if the game is over, by one robot scoring MaxScore number of points
        if (self.scoreBoard.robotAScore >= self.maxScore):
            self.endGame("A")
        if (self.scoreBoard.robotBScore >= self.maxScore):
            self.endGame("B")

    def endGame(self, winner):
        # Close the scoredboard
        self.scoreBoard.close()
        # Determine the name of the winner based on the winner letter
        _winner = ""
        if (winner == "A"):
            _winner = "Blue"
        if (winner == "B"):
            _winner = "Red"
        # Display the dialog box stating who the winner is
        _, self.dialog, _ = sim.simxDisplayDialog(self.clientID, " ", f'{_winner} wins\nBlue: {self.scoreBoard.robotAScore} || Red: {self.scoreBoard.robotBScore}', 0, " ", None , None, sim.simx_opmode_oneshot)
        # Set the quit flag to true to exit the game loop
        self.quit = True
        # Kill the robots
        self.bot.end()
        self.bot2.end()


    # Gets the current position of the ball
    def get_initial_positions(self):
        self.initial_ball_position = sim.simxGetObjectPosition(clientID, self.ball, -1, sim.simx_opmode_blocking)[1]

    # Sets the position of the ball to somewherer random within the 8 squares in the center of the field
    def set_initial_positions(self):
        sim.simxSetObjectPosition(
            clientID, self.ball, -1, [random.uniform(-1,1), random.uniform(-0.5, 0.5), 0.05], sim.simx_opmode_oneshot
        )

# Each instance of soccer robot can control a robot within coppeliasim
class SoccerRobot:
    """

    """
    def __init__(self, clientID, robotLetter, direction):
        # Retrieves all of the required joints and sensors to read data from or control
        # These include the left and right joints (motors)
        # The 5 ball sensors: Three in front, one on each side
        # The 3 goal line sensors: One on the front, and each side
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

        # Set the initial velocity of the left joint to 1 so that it turns to the right
        sim.simxSetJointTargetVelocity(
            clientID, self.leftJoint, 1, sim.simx_opmode_oneshot
        )
        # The client is how we refer to the specific remoteApi connection that we are controlling
        self.clientID = clientID

        # Direction can either be 1 or -1, -1 swaps the direction of all turns
        self.direction = direction

        # Last state is used to store the last change in movement that the robot had
        # The value is checked against to reduce the number of requests made to the server
        self.last_state = ""

        # These speeds can be changed to adjust robot movement speed
        self.base_speed = 5
        self.slow_speed = 0.5

        # These fields are used to determine whether or not the robot is stuck,
        self.last_location = []
        self.last_movement_time = 0
        self.timeout = 10
        self.opmode = sim.simx_opmode_streaming

    # Sets the speed of the left and right joints
    def setSpeed(self, leftSpeed, rightSpeed):
        _ = sim.simxSetJointTargetVelocity(
            self.clientID, self.rightJoint, rightSpeed, sim.simx_opmode_oneshot
        )
        _ = sim.simxSetJointTargetVelocity(
            self.clientID, self.leftJoint, leftSpeed, sim.simx_opmode_oneshot
        )

    # Update robot is called every loop, it reads from the sensors and controls the movement of the robot
    def update_robot(self) -> bool:
        # First read the state of all of the sensors
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

        # Because the sensor read is asynchronous it is possible that some or all of the sensor readings will be undefined for the first few loops
        # As a result we catch the error, and exit until the next loop until all are defined
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

        # If the robot is facing the ball
        if facingBall:
            # If the robot is not yet close enough to the ball to have it within its arms
            if not touchingBall:
                # The robot will move forward
                if (self.last_state != "touchingBall"):
                    self.setSpeed(1.3*self.base_speed, 1.3*self.base_speed)
                    self.last_state = "touchingBall"
            else:
                # if the robot has the ball and is facing the goal-line
                # It will also move forward towards the goal line
                if facingGoalLine:
                    if (self.last_state != "facingGoalLine"):
                        self.setSpeed(self.base_speed, self.base_speed)
                        self.last_state = "facingGoalLine"
                # if it has the ball but is not facing the goal-line it will start turning
                else:
                    if (self.last_state != "findingGoalLine"):
                        self.setSpeed(1.3*self.slow_speed*self.direction, -1.3*self.slow_speed*self.direction)
                        self.last_state = "findingGoalLine"
                # if it has the ball, and one of the side goal-line sensors are triggerd
                # If the robot is turning in the direction that will face them towards the goal-line in a 1/4 turn it keeps going
                # If the robot will have to do a 3/4 turn, it reverses direction
                if goalLineLeft:
                    self.direction = 1
                    self.last_state = "directionSwitch"
                if goalLineRight:
                    self.direction = -1
                    self.last_state = "directionSwitch"
        # If the robot is not facing the ball
        else:
            # If the front-left ball sensor is triggered the ball is slightly to the left of the robot
            # Thus it should turn slightly to the left
            if ballLeft:
                if (self.last_state != "ballLeft"):
                    self.setSpeed(self.slow_speed, self.slow_speed*2)
                    self.last_state = "ballLeft"
            # If the front-right ball sensor is triggered the ball is slightly to the right of the robot
            # Thus it should turn slightly to the right
            elif ballRight:
                if (self.last_state != "ballRight"):
                    self.setSpeed(self.slow_speed*2, self.slow_speed)
                    self.last_state = "ballRight"
            # If none of the sensors are triggered the robot should turn to find the ball
            else:
                if (self.last_state != "findingBall"):
                    self.setSpeed(self.slow_speed*self.direction, -self.slow_speed*self.direction)
                    self.last_state = "findingBall"
            # if one of the side ball sensors are triggerd
            # If the robot is turning in the direction that will face them towards the ball in a 1/4 turn it keeps going
            # If the robot will have to do a 3/4 turn, it reverses direction
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

        # If the robot has moved, the timer is reset
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

    # Robot end stops the movement of the robots entirely
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

    # Initialize the game with the clientId that represents our connection
    gamestate = GameState(clientID=clientID)

    # Start the game
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
