#!/usr/bin/env python3

from ev3dev.ev3 import *
from time import sleep
import sys, os, threading

rightMotor = LargeMotor(OUTPUT_D)
leftMotor  = LargeMotor(OUTPUT_C)
leftsensor = UltrasonicSensor(INPUT_1)
frontsensor = UltrasonicSensor(INPUT_2)
btn = Button()

INITIAL_WAITE_TIME = 2
SENSOR_DISTANCE = 500
MAX_TIME = 180000
CENTER_TIME = .01
REFIND_TIME = 5
MAX_SPEED = 500
TURN_SPEED = 800
turntime = 3
REFIND_SPEED = 200
firstrefind = 0.2
aimcentre = 0.1

class RunMotors(threading.Thread):

	#Thread setup
	def __init__(self, action_list):
		threading.Thread.__init__(self)
		self.action_list = action_list
		self.interrupt = False
		left = right = 0

	#Motor operation
	def run(self):
		for (left, right, duration) in self.action_list:
			self.left = left
			self.right = right
			rightMotor.run_timed(speed_sp=left, time_sp=duration)
			leftMotor.run_timed(speed_sp=right, time_sp=duration)
			while duration > 0:
				sleep(0.1)
				duration -= 0.1
				if self.interrupt:
					return

	#Stop Thread
	def stop(self):
		self.interrupt = True
		
def stop():
	leftMotor.stop(stop_action='brake')
	rightMotor.stop(stop_action='brake')
	leftMotor.stop()
	rightMotor.stop()
	
def new_action(interrupt, action_list):
	global current_action
	if interrupt:
		current_action.stop();
	current_action.join()
	current_action = RunMotors(action_list)
	current_action.start()
	
def turn(side):
	if(side==1):
		new_action(True, [(TURN_SPEED, -TURN_SPEED, turntime)])
	else:
		new_action(True, [(-TURN_SPEED, TURN_SPEED, turntime)])
#Indecate ready
Leds.set_color(Leds.LEFT, Leds.RED)
Leds.set_color(Leds.RIGHT, Leds.RED)
#Wait for GO signal
while (not btn.any()):
	pass
#Indicate start
Leds.set_color(Leds.LEFT, Leds.GREEN)
Leds.set_color(Leds.RIGHT, Leds.GREEN)
sleep(INITIAL_WAITE_TIME)
#Start motors in a stationary turn
current_action = RunMotors([(MAX_SPEED, MAX_SPEED, MAX_TIME)])
current_action.start()

while (not btn.any()):
	if(frontsensor.value() < SENSOR_DISTANCE):
		stop()
		