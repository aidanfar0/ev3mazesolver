#!/bin/bash
from ev3dev.ev3 import *
from time import sleep
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

#Interval of time for the turn code to check if it's turned
TURN_CHECK_INTERVAL = 0.1
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

#Greater than distance from ultrasonic to wall
US_WALL_DIST = 500

#Connect motors
rightMotor = LargeMotor(OUTPUT_A)
leftMotor = LargeMotor(OUTPUT_B)

cs = ColorSensor(INPUT_2)


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
leftMotorTrim = 0
rightMotorTrim = 0

isTurning = False

offset_check_thread = OffsetCheck()
offset_check_thread.start()

#Main thread
while True:
    if not canTurn():
        rightMotor.run_direct(duty_cycle_sp=75 - leftMotorTrim)
        leftMotor.run_direct(duty_cycle_sp=75 - rightMotorTrim)
    
    turn(-1)
    

#Checks if there is a place to turn left
def canTurn():
    return usT.value() > US_WALL_DIST

#Thread to check if the robot is moving off-course when going straight
#Reset/Run a new one every time you move on from an intersection
#Stop it once you get to a new intersection
class OffsetCheck(threading.Thread):
    #After the ultrasonic has passed the wall, this is the distance it detects
    initialAngle = 0
    
    def __init__(self):
        threading.Thread.__init__(self)
        self.interrupt = False
    
    def reset(self):
        initialAngle = gs.value
        self.interrupt = False()
        leftMotorTrim = 0
        rightMotorTrim = 0
    
    #Checking for any change in direction
    def run(self):
        while not self.interrupt:
            if isTurning:
                sleep(TURN_CHECK_INTERVAL)
                initialAngle = gs.value()
                leftMotorTrim = 0
                rightMotorTrim = 0
            else:
                angle = gs.value()
                difference = angle - initialAngle
                #Reset trims
                rightMotorTrim = 0
                leftMotorTrim = 0
                if difference > ANGLE_CORRECT_THRESHOLD:
                    #difference > 0 means it's turned right
                    #Turn it left by reducing right motor:
                    rightMotorTrim = difference
                if difference < -ANGLE_CORRECT_THRESHOLD:
                    #difference < 0 means it's turned left
                    #Turn it right by reducing left motor:
                    leftMotorTrim = -difference
                sleep(ANGLE_CORRECT_INTERVAL)
        
    #Stop Thread
    def stop(self):
        self.interrupt = True
        leftMotorTrim = 0
        rightMotorTrim = 0

#Convert angle to between 0 and 360
def angleModulus(angle):
    return (angle + 360) % 360

#Convert angle to between -180 and 180 degrees
def angleRev(angle):
    if(angle > 0):
        modded = angleModulus(angle)
        if(modded > 180):
            return modded - 360
        else:
            return modded
    else:
        #Modulus 180 but keeps it < 0
        modded = angle
        while(modded + 360 < 0):
            modded += 360
        
        if(modded < -180):
            return modded + 360
        else:
            return modded

def turn(dir):
    gs_start = gs.value()
    target = gs_start + dir*90
    
    rightMotor.run_direct(duty_cycle_sp=75 - leftMotorTrim)
    leftMotor.run_direct(duty_cycle_sp=75 - rightMotorTrim)
    isTurning = True
    print("Turning...")
    while angleRev(gs.value() - target) > 0:
        print(gs.value()+"," + target+";");
        sleep(TURN_CHECK_INTERVAL)
    isTurning = False
    
