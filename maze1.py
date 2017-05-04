from time import sleep
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

#How many degrees the robot must turn in order to correct the turn
ANGLE_CORRECT_THRESHOLD = 3
#How many seconds to check for angle correction
ANGLE_CORRECT_INTERVAL = 0.1
#Direction of the sideways ultrasonic sensor
#Left = -1, Right = 1
US_T_DIR = 1
#Direction to favor when solving the maze, -1 is left, 1 is right
#I don't guaruntee that this program will work if US_T_DIR == MAZE_DIR
MAZE_DIR = -1
assert US_T_DIR != MAZE_DIR

#Connect motors
rightMotor = LargeMotor(OUTPUT_A)
leftMotor = LargeMotor(OUTPUT_B)
#Forward ultrasonic sensor
usF = UltrasonicSensor(INPUT_1)
assert usF.connected
#Turned ultrasonic sensor
usT = UltrasonicSensor(INPUT_4)
assert usT.connected

gs = GyroSensor()
assert gs.connected

#IMPORTANT: Whenever moving straight, when there is a chance that the robot can
#Move slightly in one direction, use THIS as a turning direction,
#i.e. 
#LeftMotor = speed + motorTrim
#RightMotor = speed - motorTrim
#>0 means turn right, <0 means turn left
motorTrim = 0

#Main thread


#Checks if there is a place to turn left
def canTurn():
    

#Thread to check if the robot is moving off-course when going straight
#Reset/Run a new one every time you move on from an intersection
#Stop it once you get to a new intersection
class OffsetCheck(threading.Thread):
    #After the ultrasonic has passed the wall, this is the distance it detects
    initialAngle = 0
    
    def __init__(self):
        threading.Thread.__init__(self)
        self.interrupt = False
    
    def reset:
        initialAngle = gs.value
        self.interrupt = False()
    
    #Checking for any change in direction
    def run(self):
        while not self.interrupt:
        angle = gs.value()
        difference = angle - initialAngle
        if difference > ANGLE_CORRECT_THRESHOLD or difference < -ANGLE_CORRECT_THRESHOLD:
            #difference > 0 means it's turned right
            #difference < 0 means it's turned left
            #Reverse that for the direction
            motorTrim = -difference
            sleep(ANGLE_CORRECT_INTERVAL)
        
    #Stop Thread
    def stop(self):
        self.interrupt = True
        motorTrim = 0

def turn(dir):
    
    
    
