from time import sleep
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

#How many degrees the robot must turn in order to correct the turn
ANGLE_CORRECT_THRESHOLD = 3

#Connect motors
rightMotor = LargeMotor(OUTPUT_A)
leftMotor = LargeMotor(OUTPUT_B)
#Direction of the sideways ultrasonic sensor
#Left = -1, Right = 1
usTdir = 1
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
#i.e. LeftMotor = speed + motorTrim, and  RightMotor = speed - motorTrim
#1 means turn right, -1 means turn left
motorTrim = 0

"""#List of all previous turns made (going into the maze)
previous_turns = []

#Flags if the robot is currently in an intersection, probably turning
intersection_is = False

#A list of True:Has a wall and False:No wall, from left -> forward -> right
#Describing the current intersection
intersection_walls = []

#Either -1, 0, 1, so that when intersection_is is True, then
#this tells us how we've already turned. -1 for left, 1 for right, 0 for no turn
intersection_turn = 0"""

"""def scan_intersection():
    intersection_is = True
    intersection+turn = 0
    turn(1)
    """
    


#Thread to check if the robot is moving off-course when going straight
#Reset/Run a new one every time you move on from an intersection
#Stop it once you get to a new intersection
class OffsetCheck(threading.Thread):
    #After the ultrasonic has passed the wall, this is the distance it detects
    initialAngle = 0
    
    def __init__(self):
        threading.Thread.__init__(self)
        self.action_list = action_list
        self.interrupt = False
    
    def reset:
        initialAngle = gs.value()
    
    #Motor operation
    def run(self):
        angle = gs.value()
        difference = angle - initialAngle
        if difference > ANGLE_CORRECT_THRESHOLD or difference < -ANGLE_CORRECT_THRESHOLD:
            #Direction that it has turned in
            if difference < 0:
                direction = -1
            else
                direction = 1
            
            
        """for (left, right, duration) in self.action_list:
            self.left = left
            self.right = right
            rightMotor.run_timed(speed_sp=left, time_sp=duration)
            leftMotor.run_timed(speed_sp=right, time_sp=duration)
            while duration > 0:
                sleep(0.1)
                duration -= 0.1
                if self.interrupt:
                    return
        """
        
    #Stop Thread
    def stop(self):
        self.interrupt = True
    

def turn(dir):
    
    
    
