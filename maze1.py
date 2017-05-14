#!/usr/bin/env python3
from ev3dev.ev3 import *
from time import sleep
import sys, os, threading
#sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

"""Time to wait until turning (it will keep going forward)
This should be the amount of time that the robot takes to
Get from just detecting that the left side is open (ultrasonic)
To getting to the midpoint of the walls"""
TURN_WAIT = 1 #NEEDS CALIBRATION
#Amount of time to wait after turning to continue again
TURN_STOP_WAIT = 2
#How many centimeters the robot must be "off-center' in order to correct it
DIST_CORRECT_THRESHOLD = 10
#How many degrees the robot must turn in order to correct the turn
ANGLE_CORRECT_THRESHOLD = 3
#How many seconds to check for angle correction
ANGLE_CORRECT_INTERVAL = 0.1
#Interval of time for the turn code to check if it's turned
TURN_CHECK_INTERVAL = 0.05
#interval to check if the can is near
CAN_DIST_CHECK_INTERVAL = 0.05
#how close to get to to claw the can
CAN_DIST_THRESHOLD = 3

FORWARD_SPEED = 60

TURN_SPEED = 25

CAN_FORWARD_SPEED = 25
CAN_TURN_SPEED = 10

#DON'T CHANGE THESE 2:
#Direction of the sideways ultrasonic sensor
#Left = -1, Right = 1
US_T_DIR = -1
#Direction to favor when solving the maze, -1 is left, 1 is right
MAZE_DIR = -1
assert US_T_DIR == MAZE_DIR

#NEEDS CALIBRATION
#Greater than distance from ultrasonic to wall
US_WALL_DIST = 17

#Connect motors
rightMotor = LargeMotor(OUTPUT_D)
leftMotor = LargeMotor(OUTPUT_C)

frontMotor = MediumMotor(OUTPUT_B)

cs = ColorSensor()
assert cs.connected

#Forward ultrasonic sensor
usF = UltrasonicSensor(INPUT_4)
assert usF.connected
#Turned ultrasonic sensor
usT = UltrasonicSensor(INPUT_3)
assert usT.connected

gs = GyroSensor()
assert gs.connected

#IMPORTANT: Whenever moving straight, when there is a chance that the robot can
#Move slightly in one direction, use THIS as a turning direction,
leftMotorTrim = 0
rightMotorTrim = 0
leftMotorTrim2 = 0
rightMotorTrim2 = 0

isTurning = False
#Flag if the program is going forward but waiting to turn
turningWaiting = False
#Flag when it's moving toward the center of the intersection

#isCan = False #generally doing an operation with the can
movingToCan = False #moving towards the can
lostCan = False #when it found the can, but lost it (probably due to the robot not going straight to it)

#When we have yet to detect the ultrasonic on the side
recentlyTurned = True

#Checks if there is a place to turn left
def foundCan():
    return cs.color == 5 #red = 5

def canTurn():
    if recentlyTurned:
        return False
    else:
        print("Current Wall: %d" % usT.value())
        if usT.value() > US_WALL_DIST:
            return True
        else:
            return False
    #return (not recentlyTurned) and usT.value() > US_WALL_DIST

def canGoForward():
    return usF.value() > US_WALL_DIST

def getCan():
    movingToCan = True
    rightMotor.run_direct(duty_cycle_sp=CAN_FORWARD_SPEED)
    leftMotor.run_direct(duty_cycle_sp=CAN_FORWARD_SPEED)
    
    offset_can_check = OffsetCanCheck()
    offset_can_check.start()
    
    dist_can_check = CanDistCheck()
    dist_can_check.start()
    
    sleep(3)
    
    
    #while(usF.value() < CAN_DIST_THRESHOLD):
    #    print("Moving towards can...")
    
    print("Moved to can, closing grabbers...")
    frontMotor.run_direct(duty_cycle_sp=-40)
    sleep(3)
    frontMotor.run_direct(duty_cycle_sp=-10)
    #frontMotor.stop(stop_action='brake')
    #frontMotor.stop()

#moves on the spot until the can is found
def reFindCan():
    dir = 1
    rightMotor.run_direct(duty_cycle_sp=CAN_TURN_SPEED * -dir)
    leftMotor.run_direct(duty_cycle_sp=CAN_TURN_SPEED * dir)
    #turn on the spot until found the can again
    while not foundCan():
        sleep(TURN_CHECK_INTERVAL)
    print("Re-found the can")
    stop()
    lostCan = False
    rightMotor.run_direct(duty_cycle_sp=CAN_FORWARD_SPEED)
    leftMotor.run_direct(duty_cycle_sp=CAN_FORWARD_SPEED)

#checks if the can is not infront of the colour sensor
#start the thread when starts moving to the can
class OffsetCanCheck(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.interrupt = False
    
    def reset(self):
        self.interrupt = False
    
    def run(self):
        while not self.interrupt and movingToCan:
            if not foundCan() and not lostCan:
                lostCan = True
                reFindCan()
            else:
                sleep(TURN_CHECK_INTERVAL)

    def stop():
        self.interrupt = True

class CanDistCheck(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.interrupt = False
    
    def reset(self):
        self.interrupt = False
    
    def run(self):
        while not self.interrupt and movingToCan:
            if usF.value() < CAN_DIST_THRESHOLD:
                grabCan()
            else:
                sleep(CAN_DIST_CHECK_INTERVAL)

    def stop():
        self.interrupt = True

#This has been TESTED INDEPENDENTLY (i.e. in python console)
#Convert angle to between 0 and 360
def angleModulus(angle):
    return (angle + 360) % 360

#This has been TESTED INDEPENDENTLY (i.e. in python console)
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
            
def stop():
    leftMotor.stop(stop_action='brake')
    rightMotor.stop(stop_action='brake')
    leftMotor.stop()
    rightMotor.stop()

def forward():
    rightMotor.run_direct(duty_cycle_sp=FORWARD_SPEED - leftMotorTrim - leftMotorTrim2)
    leftMotor.run_direct(duty_cycle_sp=FORWARD_SPEED - rightMotorTrim - rightMotorTrim2)

def turn(dir):
    if(dir == 0):
        return
    
    isTurning = True
    gs_start = gs.value()
    target = gs_start + dir*90
    
    rightMotor.run_direct(duty_cycle_sp=TURN_SPEED * -dir)
    leftMotor.run_direct(duty_cycle_sp=TURN_SPEED * dir)
    print("Turning...")
    #angleRev(gs.value() () - target) Approaches 0
    #Approaches from the right if turning left
    #Approaches from the left if turning right
    #As it turns into the direction
    
    sleep(1)
    
    #while abs(angleRev(gs.value() - target)) > 0:
    #    print("%d" % gs.value());
    #    sleep(TURN_CHECK_INTERVAL)
    print("Finished turn")
    
    stop()
    sleep(TURN_STOP_WAIT)
    isTurning = False


#Thread to check if the robot is moving off-course when going straight
#Reset/Run a new one every time you move on from an intersection
class OffsetCheck(threading.Thread):
    #After the ultrasonic has passed the wall, this is the distance it detects
    initialAngle = 0
    
    def __init__(self):
        threading.Thread.__init__(self)
        self.interrupt = False
        initialAngle = gs.value()
    
    def reset(self):
        initialAngle = gs.value()
        self.interrupt = False
        leftMotorTrim = 0
        rightMotorTrim = 0
    
    #Checking for any change in direction
    def run(self):
        while not self.interrupt:
            if isTurning or turningWaiting or movingToCan:
                #Wait until not turning anymore
                sleep(TURN_CHECK_INTERVAL)
                #Set initial variables (for if it stopped turning)
                initialAngle = gs.value()
                leftMotorTrim = 0
                rightMotorTrim = 0
            else:
                #Use gyroscope to correct the trim of the wheels
                angle = gs.value()
                #difference: >0 when leaning right, and <0 when leaning left
                difference = angle - initialAngle
                #Reset trims
                rightMotorTrim = 0
                leftMotorTrim = 0
                if difference > ANGLE_CORRECT_THRESHOLD:
                    #Turn it left by reducing right motor:
                    rightMotorTrim = difference
                    print("TRIM: LEFT " + difference)
                elif difference < -ANGLE_CORRECT_THRESHOLD:
                    #Turn it right by reducing left motor:
                    leftMotorTrim = -difference
                    print("TRIM: RIGHT " + difference)
                
                sleep(ANGLE_CORRECT_INTERVAL)
        
    #Stop Thread
    def stop(self):
        self.interrupt = True
        leftMotorTrim = 0
        rightMotorTrim = 0


#Uses the ultrasonic sensor to detect when it's heading off-course
class OffsetCheckUS(threading.Thread):
    #After the ultrasonic has passed the wall, this is the distance it detects
    initialDist = 0
    foundWall = False
    
    def __init__(self):
        threading.Thread.__init__(self)
        self.interrupt = False
    
    def reset(self):
        initialDist = usT.value()
        self.interrupt = False
        leftMotorTrim2 = 0
        rightMotorTrim2 = 0
    
    #Checking for any change in direction
    def run(self):
        while not self.interrupt:
            if isTurning or turningWaiting or movingToCan:
                #Wait until not turning anymore
                sleep(TURN_CHECK_INTERVAL)
                #Set initial variables (for if it stopped turning)
                initialAngle = usT.value()
                leftMotorTrim2 = 0
                rightMotorTrim2 = 0
            elif not foundWall:
                #Check if the wall is found
                if usT.value() < US_WALL_DIST:
                    foundWall = True
                    initialDist = usT.value()
                else:
                    foundWall = False
                    sleep(ANGLE_CORRECT_TIME)
            elif foundWall:
                #Use ultrasonic sensor to correct the trim of the wheels
                dist = usT.value()
                #difference: >0 when leaning right, and <0 when leaning left
                difference = -1 * US_T_DIR * (dist - initialDist)
                #Reset trims
                rightMotorTrim2 = 0
                leftMotorTrim2 = 0
                if difference > DIST_CORRECT_THRESHOLD:
                    #Turn it left by reducing right motor:
                    rightMotorTrim2 = difference
                    print("TRIM: LEFT " + difference)
                if difference < -DIST_CORRECT_THRESHOLD:
                    #Turn it right by reducing left motor:
                    leftMotorTrim2 = -difference
                    print("TRIM: RIGHT " + difference)
                
                sleep(ANGLE_CORRECT_INTERVAL)
        
    #Stop Thread
    def stop(self):
        self.interrupt = True
        leftMotorTrim2 = 0
        rightMotorTrim2 = 0



offset_check_thread = OffsetCheck()
offset_check_thread.start()


#Main thread
while True:
    if usT.value() < US_WALL_DIST:
        recentlyTurned = False
        print("Now it works")
    else:
        if foundCan():
            getCan()
            print("FOUND CAN")
        elif (not recentlyTurned) and canTurn():
            print("CAN TURN")
            stop()
            sleep(0.5)
            
            turningWaiting = True
            sleep(TURN_WAIT)
            turningWaiting = False
            turn(MAZE_DIR)
            recentlyTurned = True
            turningWaiting = True
            forward()
        elif canGoForward():
            print("CAN GO FORWARD")
            forward()
        else:
            print("CANNOT TURN OR GO FORWARD, TURNING RIGHT")
            stop()
            sleep(0.5)
            
            turningWaiting = True
            sleep(TURN_WAIT)
            turningWaiting = False
            turn(-(MAZE_DIR))
            recentlyTurned = True
            turningWaiting = True
            forward()
