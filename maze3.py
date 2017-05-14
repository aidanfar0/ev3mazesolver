#!/usr/bin/env python3
from ev3dev.ev3 import *
from time import sleep
import sys, os, threading
from queue import Queue
from inspect import currentframe, getframeinfo
#sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


TURN_OUT_WAIT = 1.7  #NEEDS CALIBRATION
TURN_IN_WAIT = 0.9

#Amount of time to wait after turning to continue again
TURN_STOP_WAIT = 2

#Target to aim for to get the ultrasonic sensor on the side to get to when going straight forward
DIST_CORRECT_TARGET=17
#How many centimeters the robot must be "off-center' in order to correct it
DIST_CORRECT_THRESHOLD = 3
#How many degrees the robot must turn in order to correct the turn
ANGLE_CORRECT_THRESHOLD = 10
#Maximum it should aim to turn to correct for angle
ANGLE_MAX_CORRECT = 20
#How many seconds to check for angle correction
ANGLE_CORRECT_INTERVAL = 0.05

ANGLE_CORRECT_MAX = 15
#Interval of time for the turn code to check if it's turned
TURN_CHECK_INTERVAL = 0.05
#interval to check if the can is near
CAN_DIST_CHECK_INTERVAL = 0.05

SENSOR_INTERVAL = 0.05

#How much red the sensor needs to detect the can
COLOUR_THRESHOLD = 7
colours = ['none', 'black', 'blue', 'green', 'yellow', 'red', 'white', 'brown' ]

#how close to get to to claw the can
#This is so that it doesn't bump into the end of the maze
CAN_DIST_THRESHOLD = 100

#Minimum amount of power to the motors when turning and getting closer
MIN_TURN_POWER = 14

#How fast it goes forward
FORWARD_SPEED = 35

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
US_WALL_DIST = 50
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
loops_since_red = COLOUR_THRESHOLD
cs_is_red = False

#When we have yet to detect the ultrasonic on the side
recentlyTurned = False

SENSOR_QUEUE_SIZE = 3

#Queues of the last SENSOR_QUEUE_SIZE values from the sensor
usT_queue = Queue()
usF_queue = Queue()
gs_queue  = Queue()

#Current amount of items in the sensor value queue
usT_queue_len = 0
usF_queue_len = 0
gs_queue_len = 0

#These are the useful parts:
#The last values added to the array; which means
#they are not outliers (vs sensor_value) AND they are more
#up-to-date than the average of the queue
usT_queue_last = 0
usF_queue_last = 0
gs_queue_last = 0

#Maximum range counted for a sensor reading to be a non-outlier
usT_MAX_DIFF = 30
usF_MAX_DIFF = 70
gs_MAX_DIFF = 10

def refresh_val_thread():
    global usT_value
    global usF_value
    global gs_value
    
    global usT_queue
    global usF_queue
    global gs_queue
    
    global usT_queue_len
    global usF_queue_len
    global gs_queue_len
    
    global usT_queue_last
    global usF_queue_last
    global gs_queue_last
    
    while True:
        try:
            last_val = usT_value
            usT_value = usT.value()
            
            #No queue yet, or not outlier, or two outliers in a row:
            if usT_queue_len == 0 or inQueueRange(usT_queue, usT_value, usT_MAX_DIFF) or (usT_value < last_val + usT_MAX_DIFF and usT_value > last_val - usT_MAX_DIFF):
                if usT_queue_len == SENSOR_QUEUE_SIZE:
                    usT_queue.get() #Remove last element
                else:
                    usT_queue_len += 1 #Don't remove; net Addition of element
                usT_queue.put(usT_value)
                usT_queue_last = usT_value
            else:
                pass#print('[Sensor-Refresh] Outlier Side Ultrasonic Value: %d; Normal: %s' % (usT_value, list(usT_queue.queue)))
            
        except ValueError:
            print('[Sensor-Refresh] Left Ultrasonic Refresh Failed')
        
        try:
            last_val = usF_value
            usF_value = usF.value()
            if usF_queue_len == 0 or inQueueRange(usF_queue, usF_value, usF_MAX_DIFF) or (usF_value < last_val + usF_MAX_DIFF and usF_value > last_val - usF_MAX_DIFF):
                if usF_queue_len == SENSOR_QUEUE_SIZE:
                    usF_queue.get()
                else:
                    usF_queue_len += 1
                usF_queue.put(usF_value)
                usF_queue_last = usF_value
            else:
                pass#print('[Sensor-Refresh] Outlier Front Ultrasonic Value: %d; Normal: %s' % (usF_value, list(usF_queue.queue)))
        except ValueError:
            print('[Sensor-Refresh] Front Ultrasonic Refresh Failed')
        
        try:
            last_val = gs_value
            gs_value = gs.value()
            if gs_queue_len == 0 or inQueueRange(gs_queue, gs_value, gs_MAX_DIFF) or (gs_value < last_val + gs_MAX_DIFF and gs_value > last_val - gs_MAX_DIFF):
                if gs_queue_len == SENSOR_QUEUE_SIZE:
                    gs_queue.get()
                else:
                    gs_queue_len += 1
                gs_queue.put(gs_value)
                gs_queue_last = gs_value
            else:
                pass#print('[Sensor-Refresh] Outlier Gyro Value: %d; Normal: %s' % (gs_value, list(gs_queue.queue)))
        except ValueError:
            print('[Sensor-Refresh] Gyroscope Refresh Failed')
        
        sleep(SENSOR_INTERVAL)

#Determines if the value given is an outlier compared with the rest of the queue
def inQueueRange(q, val, max_diff):
    inRange = False
    for elem in list(q.queue):
        if val > elem-max_diff and val < elem+max_diff:
            inRange = True
            break
    
    return inRange


usT_value = usT.value()
usF_value = usF.value()
gs_value = gs.value()

def calibrateGyro():
    gs.mode = 'GYRO-RATE'
    gs.mode = 'GYRO-ANG'
    
    try:
        gs_value = gs.value()
    except ValueError:
        print("[Calibrate-Gyro] Gyroscope Refresh Failed")
        return
    
    while (not(gs_value == 0)):
        sleep(0.05)
        try:
            gs_value = gs.value()
        except ValueError:
            print("[Calibrate-Gyro] Gyroscope Refresh Failed")
            return

#Checks if there is a place to turn left
def foundCan():
    return cs_is_red
    #return (cs_red > cs_green + COLOUR_THRESHOLD) and (cs_red > cs_blue + COLOUR_THRESHOLD)
    #return cs.color == 5 #red = 5

def canTurn():
    if recentlyTurned:
        return False
    else:
        #TEMP: print("Current SIDE Wall: %d" % usT_value)
        if usT_value > US_WALL_DIST and not (usT_value == 255):
            print("[CanTurn:~156]: %d > %d" % (usT_value, US_WALL_DIST))
            return True
        else:
            return False
    #return (not recentlyTurned) and usT_value > US_WALL_DIST

def canGoForward():
    #print("Current FRNT Wall: %d" % usF_value)
    return usF_value > FUS_WALL_DIST

def colourDetect():
    global cs_red
    global cs_green
    global cs_blue
    global cs_intensity
    global cs_is_red
    global loops_since_red
    while True:
        #cs.mode = 'REF-RAW'
        #sleep(0.5)
        #cs_intensity = cs.reflected_light_intensity
        
        cs.mode = 'RGB-RAW'
        try:
            cs_green = cs.green
            cs_blue = cs.blue
            cs_red = cs.red
        except ValueError:
            print('[Colour-Sensor-Refresh] Colour Sensor Refresh Failed')
            return
            
        sleep(0.1)
        
        #hasGB = cs_green > 8 or cs_blue > 8
        
        #hasR = cs_intensity > 8
        
        #print("I: %d R: %d G: %d B: %d" % (cs_intensity, cs_red, cs_green, cs_blue))
        
        if cs_red > cs_blue and cs_red > 2:
            #print("RED")
            if not cs_is_red:
                print("[ColourDetect] Red: R: %d G: %d B: %d" % (cs_red, cs_green, cs_blue))
            cs_is_red = True
            loops_since_red = 0
        else:
            #Keeps the red active for 5 cycles
            if cs_is_red:
                loops_since_red += 1
                if loops_since_red >= COLOUR_THRESHOLD:
                    cs_is_red = False
                    print("[ColourDetect] Other: R: %d G: %d B: %d" % (cs_red, cs_green, cs_blue))

def canCheck():
    while True:
        if foundCan():
            print("[canCheck] Found Target")
            Sound.beep()
            Sound.speak("Target Found")
            getCan()

def getCan():
    global movingToCan
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
        print("[getCan]Moving towards can...")
    
    sleep(0.5)
    if not foundCan():
        #Recursivley gets the can until it actually gets it at the end
        print("[getCan]Lost the can!")
        lostCan = True
        reFindCan()
        getCan()
        return
    
    print("[getCan]Moved to can, closing grabbers...")
    frontMotor.run_direct(duty_cycle_sp=-40)
    sleep(3)
    frontMotor.run_direct(duty_cycle_sp=-5)
    #frontMotor.stop(stop_action='brake')
    #frontMotor.stop()
    movingToCan = False

#moves on the spot until the can is found
def reFindCan():
    print("[reFindCan]Turning until can found again")
    dir = 1
    rightMotor.run_direct(duty_cycle_sp=CAN_TURN_SPEED * -dir)
    leftMotor.run_direct(duty_cycle_sp=CAN_TURN_SPEED * dir)
    #turn on the spot until found the can again
    while not foundCan():
        sleep(TURN_CHECK_INTERVAL)
    print("[reFindCan]Found the can")
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
    
    while abs(difference) > 0 and not movingToCan: #Allows interruption by movingToCan variable
        angle = gs_value
            #if foundCan():
            #    print("[Main] Found Target (1st Time)")
            #    if hasntFoundCan:
            #        hasntFoundCan = False
            #        Sound.beep()
            #       Sound.speak("Target Found")
            #   getCan()
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
    
    if movingToCan:
        print("[turnTo]Turn interrupted by finding object")
        return
    
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
    #initialDist = 0
    foundWall = recentlyTurned
    gs_start = 361
    previous_us = usT_queue_last
    
    def __init__(self):
        threading.Thread.__init__(self)
        self.interrupt = False
    
    def reset(self):
        #initialDist = usT_value
        self.interrupt = False
        leftMotorTrim2 = 0
        rightMotorTrim2 = 0
    
    #Checking for any change in direction
    def run(self):
        global recentlyTurned
        echoConstantSpeed = True
        while not self.interrupt:
            #print("isForward: %s, foundWall: %s" % ("True" if isForward else "False", "True" if self.foundWall else "False"))
            if (not isForward) or movingToCan: #isTurning:
                #print("Waiting to go forward")
                #Wait until not turning anymore
                sleep(TURN_CHECK_INTERVAL)
                #Set initial variables (for if it stopped turning)
                self.foundWall = False
                #initialDist = usT_value
                leftMotorTrim2 = 0
                rightMotorTrim2 = 0
                echoConstantSpeed = True
                self.gs_start = angleRev(gs_queue_last)
            elif recentlyTurned or noTrim:
                #Constant speed, because we don't know the trim
                if echoConstantSpeed:
                    print("[Forward]Constant speed")
                echoConstantSpeed = False
                
                rightMotor.run_direct(duty_cycle_sp=FORWARD_SPEED)
                leftMotor.run_direct(duty_cycle_sp=FORWARD_SPEED)
                sleep(ANGLE_CORRECT_INTERVAL)
                self.gs_start = angleRev(gs_queue_last)
            elif not self.foundWall:
                echoConstantSpeed = True
                #Check if the wall is found
                if usT_queue_last < US_WALL_DIST:
                    self.foundWall = True
                    #initialDist = usT_value
                    print("[Forward]Found the wall again")
                else:
                    self.foundWall = False
                    sleep(ANGLE_CORRECT_INTERVAL)
                
                self.gs_start = angleRev(gs_queue_last)
            elif self.foundWall:
                echoConstantSpeed = True
                #Use ultrasonic sensor to correct the trim of the wheels
                #difference: >0 when leaning right, and <0 when leaning left
                us_difference = US_T_DIR * -(usT_value - DIST_CORRECT_TARGET)#initialDist)
                
                print("Ultrasonic Target: %d, Current: %d, Diff: %d" % (DIST_CORRECT_TARGET, usT_value, us_difference))
                
                if usT_value > 150:
                    continue
                    #Very likley that the robot has come too close to the wall for the sensors to work
                
                #target_gs_rel: Target angle relative to straight line
                
                if us_difference > ANGLE_CORRECT_THRESHOLD:
                    target_gs_rel = ANGLE_MAX_CORRECT
                elif us_difference < -ANGLE_CORRECT_THRESHOLD:
                    target_gs_rel = -ANGLE_MAX_CORRECT
                else:
                    target_gs_rel = 0
                
                if self.gs_start > 180: #First gyro value
                    self.gs_start = angleRev(gs_queue_last)
                
                #Where to turn to
                target_gs = angleRev(self.gs_start + target_gs_rel)
                
                #Difference between where the gyro is now, and where it's aimed for
                gs_difference = angleRev(gs_value - target_gs)
                
                rightMotorTrim2 = 0
                leftMotorTrim2 = 0
                
                if gs_difference > 0:
                    #Turn it left by reducing right motor:
                    rightMotorTrim2 = MIN_TURN_POWER + gs_difference  
                if gs_difference < -0:
                    #Turn it right by reducing left motor:
                    leftMotorTrim2 = -(MIN_TURN_POWER + gs_difference)
                
                print("Position %s; Angle %s;" % (("Too far left" if us_difference < ANGLE_CORRECT_THRESHOLD else ("Too far right" if us_difference > ANGLE_CORRECT_THRESHOLD else "Okay")),("Too far left" if gs_difference < 0 else ("Too far right" if gs_difference > 0 else "Okay"))))
                
                leftSpeed = max(FORWARD_SPEED - min(leftMotorTrim2, ANGLE_CORRECT_MAX), 0)
                rightSpeed = max(FORWARD_SPEED - min(rightMotorTrim2, ANGLE_CORRECT_MAX), 0)
                
                rightMotor.run_direct(duty_cycle_sp=leftSpeed)
                leftMotor.run_direct(duty_cycle_sp=rightSpeed)
                #sleep(ANGLE_CORRECT_INTERVAL)
                sleep(0.5)
        
    #Stop Thread
    def stop(self):
        self.interrupt = True
        leftMotorTrim2 = 0
        rightMotorTrim2 = 0

#Runs the functions that get sensor values
offset_check_thread = None
refresh_thread = None
colour_thread = None
can_thread = None

def startThreads():
    global offset_check_thread
    offset_check_thread = OffsetCheckUS()
    offset_check_thread.start()
    global refresh_thread
    refresh_thread = threading.Thread(target = refresh_val_thread)
    refresh_thread.start()
    global colour_thread
    colour_thread = threading.Thread(target=colourDetect)
    colour_thread.start()
    global can_thread
    can_thread = threading.Thread(target=canCheck)
    can_thread.start()

def mainFunc():
    global DIST_CORRECT_TARGET
    DIST_CORRECT_TARGET = usT_value
    print("Distance to left: %d" % DIST_CORRECT_TARGET)
    
    #Attempts to get us out of a loop by counting how 
    #Many lefts we make, and ignoring any left turns after
    #The 4 limit has been reached
    consecutiveLefts = 0
    
    hasntFoundCan = True
    global recentlyTurned
    recentlyTurned = False
    global isForward
    calibrated = False
    
    startThreads()
    
    #Main thread
    while True:
        if movingToCan:
            sleep(0.5)
        else:
            #if (usT_value < US_WALL_DIST) and (recentlyTurned):
            #    recentlyTurned = False
            #    print("[Main] Found wall again after turn")
            
            #if foundCan():
            #    print("[Main] Found Target (1st Time)")
            #    if hasntFoundCan:
            #        hasntFoundCan = False
            #        Sound.beep()
            #       Sound.speak("Target Found")
            #   getCan()
            if (not recentlyTurned) and canTurn() and (consecutiveLefts < 5):
                
                #Go into the middle of the intersection
                recentlyTurned = True
                   
                #sleep(TURN_IN_WAIT)
                
                #Waits until TURN_IN_WAIT has completed:
                #Does this while measuring the sensor every
                #time the sensor is updated
                #So that we don't get a single outlier that causes a turn
                falses = 0
                trues = 0
                for i in range(0, int(TURN_IN_WAIT/SENSOR_INTERVAL)):
                    if not canTurn():
                        falses += 1
                    else:
                        trues += 1
                    sleep(SENSOR_INTERVAL)
                
                if not falses >= trues: #False positive detected
                    print("[Main] False left")
                    recentlyTurned = False
                    continue
                else:
                    calibrated = False #Calibrates next time it goes forward
                    print("[Main] Left")
                    Sound.speak("left")
                    
                    consecutiveLefts += 1
                    
                    #Stop, and turn
                    isForward = False
                    turn(MAZE_DIR)
                    
                    #Will go out of the intersection (unless forward is blocked) VV
                    recentlyTurned = True
            elif canGoForward():
                if not isForward:
                    print("[Main] Forward")
                forward()
                if not calibrated:
                    calibrated = True
                    calibrateGyro()
                
                if recentlyTurned:
                    #Currently, the robot is going out of a turn
                    #This will make sure that it goes past the wall before
                    #Any other detections or anything
                    print("[Main] Moving out of turn...")
                    sleep(TURN_OUT_WAIT)
                    print("[Main] Moved out of turn")
                    recentlyTurned = False
            else:
                calibrated = False
                print("[Main] Right")
                Sound.speak("right")
                
                #Right turns are already at the middle of the intersection
                #By the time it is detected that it can't go forward
                #Therefore, no need for sleep
                #sleep(TURN_IN_WAIT)
                recentlyTurned = True
                
                consecutiveLefts = 0
                
                #Stop, and turn
                isForward = False
                turn(-(MAZE_DIR))
                
                #Will go out of the intersection (unless forward is blocked) ^^
                recentlyTurned = True

#Runs the code if this python .py file is not imported
#This is because we may want to test the functions individually
#Using the command-line python, and we can just import this
#File, but not have the main function run.
if __name__ == '__main__':
    mainFunc()
