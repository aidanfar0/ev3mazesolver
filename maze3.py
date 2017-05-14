#!/bin/bash
from ev3dev.ev3 import *
from time import sleep
import sys, os, threading
#sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

"""Time to wait until turning (it will keep going forward)
This should be the amount of time that the robot takes to
Get from just detecting that the left side is open (ultrasonic)
To getting to the midpoint of the walls"""
TURN_WAIT = 1.1  #NEEDS CALIBRATION

#Amount of time to wait after turning to continue again
TURN_STOP_WAIT = 2

#Target to aim for to get the ultrasonic sensor on the side to get to when going straight forward
DIST_CORRECT_TARGET=17
#How many centimeters the robot must be "off-center' in order to correct it
DIST_CORRECT_THRESHOLD = 3
#How many degrees the robot must turn in order to correct the turn
ANGLE_CORRECT_THRESHOLD = 3
#How many seconds to check for angle correction
ANGLE_CORRECT_INTERVAL = 0.1
#Interval of time for the turn code to check if it's turned
TURN_CHECK_INTERVAL = 0.05
#interval to check if the can is near
CAN_DIST_CHECK_INTERVAL = 0.05

#How much red the sensor needs to detect the can
COLOUR_THRESHOLD = -1

#how close to get to to claw the can
#This is so that it doesn't bump into the end of the maze
CAN_DIST_THRESHOLD = 100

#Minimum amount of power to the motors when turning and getting closer
MIN_TURN_POWER = 10

#How fast it goes forward
FORWARD_SPEED = 45

#How fas the wheels turn on the spot
TURN_SPEED = 25

#How fast to go forward when fond the can
CAN_FORWARD_SPEED = 25

#How fast to turn when found the can
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
US_WALL_DIST = 30
FUS_WALL_DIST = 200

#Connect motors
rightMotor = LargeMotor(OUTPUT_D)
leftMotor = LargeMotor(OUTPUT_C)

frontMotor = MediumMotor(OUTPUT_B)

cs = ColorSensor()
assert cs.connected

#Forward ultrasonic sensor
usF = UltrasonicSensor(INPUT_3)
assert usF.connected
#Side ultrasonic sensor
usT = UltrasonicSensor(INPUT_4)
assert usT.connected

gs = GyroSensor()
assert gs.connected
gs.mode = 'GYRO-ANG'

#IMPORTANT: Whenever moving straight, when there is a chance that the robot can
#Move slightly in one direction, use THIS as a turning direction,
leftMotorTrim = 0
rightMotorTrim = 0
leftMotorTrim2 = 1
rightMotorTrim2 = 0

isTurning = False
#Flag when it's moving toward the center of the intersection

#isCan = False #generally doing an operation with the can
movingToCan = False #moving towards the can
lostCan = False #when it found the can, but lost it (probably due to the robot not going straight to it)

#Flags when moving forward
global isForward
global offset_check_thread
global noTrim
global recentlyTurned
noTrim = False
isForward = False

#When we have yet to detect the ultrasonic on the side
recentlyTurned = False


def refresh_val_thread():
    global usT_value
    global usF_value
    global gs_value
    global cs_red
    global cs_green
    global cs_blue
    while True:
        usT_value = usT.value()
        usF_value = usF.value()
        gs_value = gs.value()
        cs_red = cs.red
        cs_green = cs.green
        cs_blue = cs.blue
        
        sleep(.05)


usT_value = usT.value()
usF_value = usF.value()
gs_value = gs.value()
cs_red = cs.red
cs_green = cs.green
cs_blue = cs.blue

def calibrateGyro():
    gs.mode = 'GYRO-RATE'
    gs.mode = 'GYRO-ANG'
    while (not(gs.value() == 0)):
        pass

#Checks if there is a place to turn left
def foundCan():
    return (cs_red > cs_green + COLOUR_THRESHOLD) and (cs_red > cs_blue + COLOUR_THRESHOLD)
    #return cs.color == 5 #red = 5

def canTurn():
    if recentlyTurned:
        return False
    else:
        #TEMP: print("Current SIDE Wall: %d" % usT_value)
        if usT_value > US_WALL_DIST:
            return True
        else:
            return False
    #return (not recentlyTurned) and usT_value > US_WALL_DIST

def canGoForward():
    print("Current FRNT Wall: %d" % usF_value)
    return usF_value > FUS_WALL_DIST

def getCan():
    movingToCan = True
    rightMotor.run_direct(duty_cycle_sp=CAN_FORWARD_SPEED)
    leftMotor.run_direct(duty_cycle_sp=CAN_FORWARD_SPEED)
    
    #offset_can_check = OffsetCanCheck()
    #offset_can_check.start()
    
    #dist_can_check = CanDistCheck()
    #dist_can_check.start()
    
    #sleep(1.5)
    
    #Goes for the can until the maze wall is reached
    while(usF_value < CAN_DIST_THRESHOLD) and foundCan():
        print("Moving towards can...")
    
    sleep(0.5)
    if not foundCan():
        #Recursivley gets the can until it actually gets it at the end
        lostCan = True
        reFindCan()
        getCan()
        return
    
    print("Moved to can, closing grabbers...")
    frontMotor.run_direct(duty_cycle_sp=-40)
    sleep(3)
    frontMotor.run_direct(duty_cycle_sp=-5)
    #frontMotor.stop(stop_action='brake')
    #frontMotor.stop()
    movingToCan = False

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

#checks if the can is not infront of the colour sensor
#start the thread when starts moving to the can

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
    global isForward
    isForward = False
    leftMotor.stop(stop_action='brake')
    rightMotor.stop(stop_action='brake')
    leftMotor.stop()
    rightMotor.stop()

def forward():
    global isForward
    isForward = True

def turn(dir):
    if(dir == 0):
        return
    elif dir>0:
        Sound.speak("Turning right")
    else:
        Sound.speak("Turning left")
    
    
    isTurning = True
    gs_start = gs_value
    target = gs_start + dir*90
    
    #rightMotor.run_direct(duty_cycle_sp=TURN_SPEED * -dir)
    #leftMotor.run_direct(duty_cycle_sp=TURN_SPEED * dir)
    print("Turning...")
    #angleRev(gs_value () - target) Approaches 0
    #Approaches from the right if turning left
    #Approaches from the left if turning right
    #As it turns into the direction
    
    turnTo(target)
    
    #sleep(1.3)
    #print("ANGLE: %d" % angleRev(gs_value));
    #print("TARGET: %d" % angleRev(target))
    #print("DISTANCE: %d" % dir*-(angleRev(angleRev(gs_value) - angleRev(target))))
    
    #while dir*-(angleRev(angleRev(gs_value) - angleRev(target))) > 5 :
    #    print("ANGLE: %d" % angleRev(gs_value));
    #    print("TARGET: %d" % angleRev(target))
    #    #print("DISTANCE: %d" % dir*-(angleRev(angleRev(gs_value) - angleRev(target))))
    #    sleep(TURN_CHECK_INTERVAL * 10)
    #    
    #    if foundCan():
    #        getCan()
    #        print("FOUND CAN")
    
    print("Finished turn")
    
    stop()
    sleep(TURN_STOP_WAIT)
    isTurning = False

#Turn the robot until it reaches the target angle
def turnTo(target):
    #>0 means robot is too far to the right, <0 too far to the left
    angle = gs_value
    difference = angleRev(angle - target)
    print("Target: %d" % target)
    
    while abs(difference) > 0:
        angle = gs_value
        difference = angleRev(angle - target)
        #print("[TURN] Difference: %d" % difference)
        if difference > 0:
            dir = 1
        else:
            dir = -1
        
        #Caps the difference at 90
        if abs(difference) > 90:
            difference = dir * 90
        
        #It slows down when it gets closer
        #But it's always at least 15
        
        rightMotor.run_direct(duty_cycle_sp=(TURN_SPEED - MIN_TURN_POWER) *  difference / 90 + MIN_TURN_POWER *  dir)
        leftMotor.run_direct( duty_cycle_sp=(TURN_SPEED - MIN_TURN_POWER) * -difference / 90 + MIN_TURN_POWER * -dir)
        
        sleep(TURN_CHECK_INTERVAL)
    stop()
    
    sleep(TURN_CHECK_INTERVAL*20)
    #Wait for gyro to calm down or something
    angle = gs_value
    difference = angleRev(angle - target)
    if difference > 2:
        print("Target reached then unreached")
        turnTo(target)

#Uses the ultrasonic sensor to detect when it's heading off-course
class OffsetCheckUS(threading.Thread):
    #After the ultrasonic has passed the wall, this is the distance it detects
    initialDist = 0
    foundWall = recentlyTurned
    
    def __init__(self):
        threading.Thread.__init__(self)
        self.interrupt = False
    
    def reset(self):
        initialDist = usT_value
        self.interrupt = False
        leftMotorTrim2 = 0
        rightMotorTrim2 = 0
    
    #Checking for any change in direction
    def run(self):
        global recentlyTurned
        while not self.interrupt:
            #print("isForward: %s, foundWall: %s" % ("True" if isForward else "False", "True" if self.foundWall else "False"))
            if not isForward: #isTurning:
                #print("Waiting to go forward")
                #Wait until not turning anymore
                sleep(TURN_CHECK_INTERVAL)
                #Set initial variables (for if it stopped turning)
                self.foundWall = False
                initialDist = usT_value
                leftMotorTrim2 = 0
                rightMotorTrim2 = 0
            elif recentlyTurned or noTrim:
                #Constant speed, because we don't know the trim
                print("[Forward]Constant speed")
                
                rightMotor.run_direct(duty_cycle_sp=FORWARD_SPEED)
                leftMotor.run_direct(duty_cycle_sp=FORWARD_SPEED)
                sleep(ANGLE_CORRECT_INTERVAL)
            elif not self.foundWall:
                #Check if the wall is found
                if usT_value < US_WALL_DIST:
                    self.foundWall = True
                    initialDist = usT_value
                    print("[Forward]Found the wall again")
                else:
                    self.foundWall = False
                    sleep(ANGLE_CORRECT_INTERVAL)
                
            elif self.foundWall:
                #Use ultrasonic sensor to correct the trim of the wheels
                dist = usT_value
                #difference: >0 when leaning right, and <0 when leaning left
                difference = -1 * US_T_DIR * (dist - DIST_CORRECT_TARGET)#initialDist)
                if difference > 200:
                    #Very likley that the robot has come too close to the wall for the sensors to work
                    difference = 0
                
                #print("[TRIM] Difference: %d" % difference)
                #Reset trims
                rightMotorTrim2 = 0
                leftMotorTrim2 = 0
                if difference > DIST_CORRECT_THRESHOLD:
                    #Turn it left by reducing right motor:
                    rightMotorTrim2 = difference
                    #print("TRIM: LEFT: %d" % difference)
                if difference < -DIST_CORRECT_THRESHOLD:
                    #Turn it right by reducing left motor:
                    leftMotorTrim2 = -difference
                    #print("TRIM: RIGHT %d" % difference)
                
                rightMotor.run_direct(duty_cycle_sp=FORWARD_SPEED - leftMotorTrim2)
                leftMotor.run_direct(duty_cycle_sp=FORWARD_SPEED - rightMotorTrim2)
                sleep(ANGLE_CORRECT_INTERVAL)
        
    #Stop Thread
    def stop(self):
        self.interrupt = True
        leftMotorTrim2 = 0
        rightMotorTrim2 = 0

#Runs the functions that get sensor values
refresh_thread = threading.Thread(target = refresh_val_thread)
refresh_thread.start()

offset_check_thread = OffsetCheckUS()
offset_check_thread.start()

def mainFunc():
    global DIST_CORRECT_TARGET
    DIST_CORRECT_TARGET = usT_value
    print("Distance to left: %d" % DIST_CORRECT_TARGET)
    
    hasntFoundCan = True
    global recentlyTurned
    recentlyTurned = False
    global isForward
    calibrated = False
    
    #Main thread
    while True:
        if movingToCan:
            isForward = False
            sleep(0.5)
        else:
            if (usT_value < US_WALL_DIST) and (recentlyTurned):
                recentlyTurned = False
                print("[Main]Found wall again")
            
            if foundCan():
                if hasntFoundCan:
                    hasntFoundCan = False
                    Sound.beep()
                    Sound.speak("I found the object")
                isForward = False
                getCan()
                print("[Main] Found Object")
            elif (not recentlyTurned) and canTurn():
                calibrated = False
                print("[Main] Left")
                recentlyTurned = True
                
                sleep(TURN_WAIT)
                isForward = False
                turn(MAZE_DIR)
                recentlyTurned = True
            elif canGoForward():
                print("[Main] Forward")
                forward()
                if not calibrated:
                    calibrated = True
                    calibrateGyro()
                if recentlyTurned:
                    #Currently, the robot is going forward after a turn;
                    #This will make sure that it goes past the wall
                    sleep(TURN_WAIT)
                    print("[Main]Moved out of turn")
                    recentlyTurned = False
            else:
                calibrated = False
                print("[Main] Right")
                stop()
                
                sleep(TURN_WAIT)
                turn(-(MAZE_DIR))
                recentlyTurned = True

#Runs the code if this python .py file is not imported
#This is because we may want to test the functions individually
#Using the command-line python, and we can just import this
#File, but not have the main function run.
if __name__ == '__main__':
    mainFunc()
