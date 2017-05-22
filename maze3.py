#!/usr/bin/env python3
from ev3dev.ev3 import *
from time import sleep
import sys, os, threading
from queue import Queue
from inspect import currentframe, getframeinfo
#sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

#DO NOT USE SENSOR VALUES DIRECTLY
#Instead, use usT_value, gs_value, usF_value
#Using it directly can call the .value() function too many times
#Which crashes the program

#TO DEBUG:
#python
#from maze3 import *
#startThreads(False) #for not trying to find the can
#startThreads(True) #If you are trying to find the can
#forward() to test the wall-following
#turn(..) and turnTo(..) to test turning
#mainFunc() to start the program (Needs startThreads(true) first)

#FORWARD/COURSE CORRECTION variables:
#Amount of time to wait after turning (in general)
#This is only useful if the gyro likes to stuff around
TURN_STOP_WAIT = 0

#How far away from the wall the robot should keep (usT_value)
#Currently, this is set to the left ultrasonics' when the program starts
DIST_CORRECT_TARGET=17
#When the robot is 'off-center' (from DIST_CORRECT_TARGET) by this many units
#Then it will attempt to correct itself.
DIST_CORRECT_THRESHOLD = 3
#When going forward, the interval of time for the robot to attempt to do course correction
ANGLE_CORRECT_INTERVAL = 0.05
#Maximum amount of correction to apply to each wheel
ANGLE_CORRECT_MAX = 15
#How fast it goes forward
FORWARD_SPEED = 35

#TURNING/TIMING VARIABLES:
#Timing variables for
#After the robot detects that it can turn left, it will go forward for TURN_IN_WAIT seconds
#before Actually turning left, so that it can get into the center of the intersection,
#Aswell as it will go forward TURN_OUT_WAIT seconds after it's made the actual turn, so
#That it doesn't detect a left turn and go back the way it came from
TURN_OUT_WAIT = 1.7
TURN_IN_WAIT = 0.9
#Speed of wheels when turning
TURN_SPEED = 25
#When turning, the motors should run at a minimum of this much power
#If the robot overshoots on turns, try turn this down so it gets more precision
#If the robot doesn't complete the turn all the way, and needs a little help
#(Pushin it manully a bit), then turn it up
MIN_TURN_POWER = 14
#When the robot is turning, the interval of time for it to 
#change turn speed/check if it's turned yet
TURN_CHECK_INTERVAL = 0.05
#interval to check if the can is near
CAN_DIST_CHECK_INTERVAL = 0.05
#Interval to update sensor values at
SENSOR_INTERVAL = 0.05

#COLOUR/CAN VARIABLES:
#How many times the sensor has to detect a red colour for it to register
COLOUR_THRESHOLD = 3
#How sensitive it is to the red colour in front
RED_SENSITIVITY = 3/2
#Just colour names, not used but maybe for debugging?
colours = ['none', 'black', 'blue', 'green', 'yellow', 'red', 'white', 'brown' ]
#How much cs_red has to be to grab the can
#This should be the value of cs_red when the can is in the claws
CAN_GRAB_THRESHOLD = 50
#How fast to go forward when it detected the red can
CAN_FORWARD_SPEED = 17
#How fast to turn when trying to "Re-Find" the can
#I.e. if the can is found, but then the red sensor is lost,
#It will turn at this speed until it detects red again
#
#This is because maybe the robot is going forward and 
#when it detects the can, it may be not straight forward,
#So when it goes forward to the can, it may loose sight of it again.
#To combat this, the robot turns on the spot until it finds the can again
CAN_TURN_SPEED = 10

#DON'T CHANGE THESE 2:
#Because I haven't checked that the code works with them changed
#Direction of the sideways ultrasonic sensor
#Left = -1, Right = 1
US_T_DIR = -1
#Direction to favor when solving the maze, -1 is left, 1 is right
MAZE_DIR = -1
assert US_T_DIR == MAZE_DIR

#Threshold that the ultrasonic sensor (on the side) has to be higher than
#For it to count as a wall
US_WALL_DIST = 50
#Same but for front sensor. 
#It's different because the sensors are physically different (NXT vs EV3 sensor)
FUS_WALL_DIST = 200

#Makes everything reliant on the gyroscope, in 90 degree increments
RELY_ON_GS = True

#Where the gyroscope starts
gs_offset = 0

#Connect sensors/motors:
rightMotor = LargeMotor(OUTPUT_D)
leftMotor = LargeMotor(OUTPUT_C)

frontMotor = MediumMotor(OUTPUT_B)

cs = ColorSensor()
assert cs.connected

#Forward ultrasonic sensor
usF = UltrasonicSensor(INPUT_3)
assert usF.connected
#Side ultrasonic sensor ('T' for turn :P)
usT = UltrasonicSensor(INPUT_4)
assert usT.connected

gs = GyroSensor()
assert gs.connected
gs.mode = 'GYRO-ANG'

#Flags when the robot is turning (not when it's going into/out of a turn)
isTurning = False
#When it IS moving out of/into a turn (i.e. centering into the intersection)
global recentlyTurned
recentlyTurned = False

movingToCan = False #moving towards the can
grabbedCan = False #Can is in its claws

#Variable used for inter-thread communication to tell it to go straight forward
global isForward
isForward = False
#Thread for this whole forward/course correct stuff:
global offset_check_thread
#Turns off course-correct, (including using gyro)
global noTrim
noTrim = False

#Flags if it found red colour (the can is in front of it)
cs_is_red = False
#When detecting red colour, it has to detect it This many loops in a row
#For it to register as actually the can
loops_as_red = 0
#After it's actually found the can, it has to be NOT red for this many
#loops in a row for it to count as lost the can
loops_since_red = COLOUR_THRESHOLD

#QUEUEING stuff is NOT USED (I tried to use it, it didn't do much)
#Maybe use it to get average of last values?
#This is the size of the queue for the last sensor values
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
#I.e. if a sensor reading is different to all 3 previous sensor readings
#By at least this much, then it will count as an outlier
usT_MAX_DIFF = 30
usF_MAX_DIFF = 70
gs_MAX_DIFF = 10

#This is the function for a thread that updates the sensor values
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
        #Sometimes, the sensors still fail, so we need the try block to catch the exception and not crash the program
        #The value calls usually work again afterwards
        try:
            last_val = usT_value
             #If you want to remove the queue stuff, keep this only: usT_value = usT.value(), (for other sensors aswell)
            usT_value = usT.value()
            
            #No queue yet, or not outlier, or two outliers in a row (value actually did change then):
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

#Initializes the values
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

#Checks if the can is infront
def foundCan():
    return cs_is_red

#Checks if there is a place to turn left
def canTurn():
    if recentlyTurned:
        return False
    else:
        #TEMP: print("Current SIDE Wall: %d" % usT_value)
        if usT_value > US_WALL_DIST and not (usT_value == 255):
            print("[CanTurn]: %d > %d" % (usT_value, US_WALL_DIST))
            return True
        else:
            return False

#Checks if there is no wall in front. Duh.
def canGoForward():
    return usF_value > FUS_WALL_DIST

#Separate function to detect if the can is in front.
#This is quite difficult,
def colourDetect():
    global cs_red
    global cs_green
    global cs_blue
    global cs_intensity
    global cs_is_red
    global loops_since_red
    global loops_as_red
    while True:
        #Probably dont need this
        cs.mode = 'RGB-RAW'
        
        #The sensor values may fail here aswell:
        #This is similar to the gs,us refresher but for colour
        try:
            cs_green = cs.green
            cs_blue = cs.blue
            cs_red = cs.red
        except ValueError:
            print('[Colour-Sensor-Refresh] Colour Sensor Refresh Failed')
            return
            
        sleep(0.1)
        
        #The sensor is quite insensitive to red unfortunatley 
        #(Probably because the red LED doesn't light up very much when in colour mode,
        #However, the red LED DOES light up when in 'REF-RAW' or 'Reflected intensity' mode)
        
        #Therefore, we just need it to be bigger than blue (or green? green is lower than blue generally)
        #And it has to actually be red
        if cs_red > cs_blue*1/RED_SENSITIVITY and cs_red > 2:
            if not cs_is_red:
                #Just changed from not red->red
                print("[ColourDetect] Red: R: %d G: %d B: %d" % (cs_red, cs_green, cs_blue))
            loops_since_red = 0
            loops_as_red += 1
            if loops_as_red >= COLOUR_THRESHOLD:
                cs_is_red = True
        else:
            #Red not detected
            #Keeps the red active for 5 cycles
            if cs_is_red:
                loops_since_red += 1
                if loops_since_red >= COLOUR_THRESHOLD:
                    loops_as_red = 0
                    cs_is_red = False
                    #Just changed from red->not red
                    print("[ColourDetect] Other: R: %d G: %d B: %d" % (cs_red, cs_green, cs_blue))

#Run in the background to override everything else and check for the can
def canCheck():
    while not grabbedCan:
        if foundCan():
            print("[canCheck] Found Target")
            Sound.beep()
            Sound.speak("Target Found")
            getCan()

#Once the can is detected as being in front, this makes it go forward and grab it
#   Maybe we should use the forward() function without wall distance for this?
#   to possibly get it on the same gyro path? but it's a small enough distance
#   that the robot's turn doens't really matter that much
def getCan():
    global movingToCan
    global grabbedCan
    movingToCan = True
    rightMotor.run_direct(duty_cycle_sp=CAN_FORWARD_SPEED)
    leftMotor.run_direct(duty_cycle_sp=CAN_FORWARD_SPEED)
    
    #Goes for the can until the sensor finds it INSIDE the claws
    while (cs_red < CAN_GRAB_THRESHOLD) and foundCan():
        print("[getCan]Moving towards can...")
        sleep(0.1)
    
    if not foundCan():
        #Recursivley gets the can until it actually gets it at the end
        print("[getCan]Lost the can!")
        #reFindCan()
        #getCan()foundCan()
        movingToCan = False
        return
    
    #Don't run over the can! D:
    stop()
    
    print("[getCan]Moved to can, closing grabbers...")
    #Initial motion
    frontMotor.run_direct(duty_cycle_sp=-40)
    sleep(3)
    #Applies constant pressure to make sure it's in there, This may not be needed
    frontMotor.run_direct(duty_cycle_sp=-5)
    grabbedCan = True
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

def nearestRightAngle(angle):
    angle -= offset
    #Round to nearest 90
    angle = round(angleRev(angle)/90, 0) * 90
    angle += offset
    return angleRev(angle)

#Generic stop function, stops the motors, and stops the forward thread from running
def stop():
    global isForward
    isForward = False
    leftMotor.stop(stop_action='brake')
    rightMotor.stop(stop_action='brake')
    leftMotor.stop()
    rightMotor.stop()

#Tells the forward thread to go forward
def forward():
    global isForward
    isForward = True

#Turns in the given direction. 1 for right, -1 for left.
def turn(dir):
    if(dir == 0):
        return
    
    isTurning = True
    gs_start = gs_value
    target = gs_start + dir*90
    
    print("Turning...")
    
    if RELY_ON_GS:
        turnTo(nearestRightAngle(target))
    else:
        turnTo(target)
    
    print("Finished turn")
    
    stop()
    isTurning = False

#Turn the robot until it reaches the target angle
def turnTo(target):
    #>0 means robot is too far to the right, <0 too far to the left
    difference = angleRev(gs_value - target)
    print("Target: %d" % target)
    
    #Allows interruption by movingToCan variable
    #Might want to add some other interruptable variables to it, like
    #A stop variable maybe?
    while abs(difference) > 0 and not movingToCan:
        difference = angleRev(gs_value - target)
        #print("[TURN] Difference: %d" % difference)
        if difference > 0:
            dir = 1
        else:
            dir = -1
        
        #Caps the difference at 90, because any more would be pointless to measure
        #And it may make the motors run too fast.
        if abs(difference) > 90:
            difference = dir * 90
        
        #It slows down when it gets closer
        #But it's always at least MIN_TURN_POWER
        #And when it's at the maximum difference (i.e. 90), then it's at TURN_SPEED speed
        rightMotor.run_direct(duty_cycle_sp=(TURN_SPEED - MIN_TURN_POWER) *  difference / 90 + MIN_TURN_POWER *  dir)
        leftMotor.run_direct( duty_cycle_sp=(TURN_SPEED - MIN_TURN_POWER) * -difference / 90 + MIN_TURN_POWER * -dir)
        
        sleep(TURN_CHECK_INTERVAL)
    
    if movingToCan:
        print("[turnTo]Turn interrupted by finding object")
        return
    
    stop()
    
    #Wait for gyro to calm down, get a grip and give us a good value
    sleep(TURN_STOP_WAIT)
    difference = angleRev(gs_value - target)
    #Because sometimes it may be a bit lagged behind and we want it to be more precise
    if difference > 2:
        print("Target reached then unreached")
        turnTo(target)

#Uses the ultrasonic sensor to detect when it's heading off-course
#It's a mix of a wall-following robot and a robot that keeps the same heading (gyro)
class OffsetCheckUS(threading.Thread):
    #It's important that there actually is a wall to follow, this will determine if there is:
    #If it's not found, then it will fall back to only keeping the same heading, not using wall-following
    foundWall = recentlyTurned
    gs_start = 0
    
    def __init__(self):
        threading.Thread.__init__(self)
        self.interrupt = False
    
    def reset(self):
        self.interrupt = False
    
    #Checking for any change in direction
    def run(self):
        global recentlyTurned
        #This flag allows us to detect when it changed from constant speed(moving out of turn) -> wall follow (normal mode)
        #used to reset the gyro value when it does change
        firstConstantSpeed = True
        while not self.interrupt:
            if (not isForward) or movingToCan: #isTurning:
                #Wait until not turning anymore
                sleep(TURN_CHECK_INTERVAL)
                #Set initial variables (for if it stopped turning)
                self.foundWall = False
                firstConstantSpeed = True
                self.gs_start = angleRev(gs_value)
            elif recentlyTurned or noTrim:
                #Constant speed, trying to keep same heading
                if firstConstantSpeed:
                    print("[Forward] Constant speed, resetting gyro value to %d" % gs_value)
                    self.gs_start = gs_value
                    firstConstantSpeed = False
                
                #Keeps straight using the heading only
                self.straight(False)
                self.foundWall = False
            elif not self.foundWall:
                if not firstConstantSpeed:
                    print("[Forward] Just came out of constant speed; Haven't found wall yet; Resetting gyro value to %d" % gs_value)
                    self.gs_start = gs_value
                    firstConstantSpeed = True
                
                #Check if the wall is found
                if usT_value < US_WALL_DIST:
                    self.foundWall = True
                    print("[Forward] Found the wall again")
                else:
                    #Keeps using the heading only
                    self.straight(False)
                
            elif self.foundWall:
                if not firstConstantSpeed:
                    #Just came out of the constant speed part, so we need to reset the gyro's value
                    print("[Forward] Just came out of constant speed; Resetting gyro target to %d" % gs_value)
                    self.gs_start = gs_value
                    firstConstantSpeed = True
                
                if usT_value > US_WALL_DIST:
                    self.foundWall = False
                    print("[Forward] Lost the wall")
                else:
                    #Wall-following hybrid code executed since the wall is found
                    self.straight(True)
    
    def straight(self, use_us):
        #use_us: True for keep-heading and wall-following hybrid
        #        False to only keep the heading
        
        #For keep heading, if we adjust the robot mid-way
        #It will try to get back to the original heading, which is not what we want
        #So maybe we should instead use the average of the last few seconds of gyro
        #values to aim for?
        if RELY_ON_GS:
            gs_target = nearestRightAngle(self.gs_start)
        else:
            if use_us:
                #us_difference: <0 if the robot is too far right; >0 if the robot is too far left
                us_difference = US_T_DIR * (usT_value -DIST_CORRECT_TARGET)
                
                #Where to aim the gyro at (Average of ultrasonic and starting gyro)
                gs_target = ((gs_value + us_difference) + self.gs_start) / 2
            else:
                gs_target = self.gs_start;
        
        #Difference between where the gyro is now, and where it's aimed for
        gs_difference = angleRev(gs_value - gs_target)
        
        rightMotorAdjust = 0
        leftMotorAdjust = 0
        
        if gs_difference > 0:
            #Turn it left by reducing left motor:
            leftMotorAdjust = MIN_TURN_POWER
        if gs_difference < 0:
            #Turn it right by reducing right motor:
            rightMotorAdjust = MIN_TURN_POWER
        
        leftSpeed = max(FORWARD_SPEED - min(leftMotorAdjust, ANGLE_CORRECT_MAX), 0)
        rightSpeed = max(FORWARD_SPEED - min(rightMotorAdjust, ANGLE_CORRECT_MAX), 0)
        
        rightMotor.run_direct(duty_cycle_sp=rightSpeed)
        leftMotor.run_direct(duty_cycle_sp=leftSpeed)
        sleep(ANGLE_CORRECT_INTERVAL)
    
    #Stop Thread
    def stop(self):
        self.interrupt = True


#Runs the functions that get sensor values
offset_check_thread = None
refresh_thread = None
colour_thread = None
can_thread = None

#canDetect is True, unless you're trying to debug the program
#And you don't want it occasianally detecting the can
def startThreads(canDetect):
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
    if canDetect: 
        can_thread = threading.Thread(target=canCheck)
        can_thread.start()

def mainFunc():
    global DIST_CORRECT_TARGET
    DIST_CORRECT_TARGET = usT_value
    print("Distance to left: %d" % DIST_CORRECT_TARGET)
    
    #Attempts to get us out of a loop by counting how 
    #Many lefts we make, and ignoring any left turns after
    #The 4 limit has been reached
    
    #this doesn't work well in practice as sometimes it turns right in the loop when it shouldn't
    consecutiveLefts = 0
    
    global recentlyTurned
    recentlyTurned = False
    global isForward
    calibrated = False
    
    gs_offset = gs_value
    
    #Main thread
    while True:
        if movingToCan:
            sleep(0.5)
            #Waits until its grabned the can to keep going through the maze
        else:
            if (not recentlyTurned) and canTurn() and (consecutiveLefts < 5):
                
                #Go into the middle of the intersection
                recentlyTurned = True
                
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
    
    startThreads(True)
    mainFunc()
