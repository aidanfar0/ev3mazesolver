from time import sleep
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

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

"""#List of all previous turns made (going into the maze)
previous_turns = []

#Flags if the robot is currently in an intersection, probably turning
intersection_is = false

#A list of true:Has a wall and false:No wall, from left -> forward -> right
#Describing the current intersection
intersection_walls = []

#Either -1, 0, 1, so that when intersection_is is true, then
#this tells us how we've already turned. -1 for left, 1 for right, 0 for no turn
intersection_turn = 0"""

"""def scan_intersection():
    intersection_is = true
    intersection+turn = 0
    turn(1)
    """
    




def turn(dir):
    
    
    
