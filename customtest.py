#!/usr/bin/env python3
from time import sleep
import sys, os
from ev3dev.ev3 import *

rightMotor = LargeMotor(OUTPUT_C)
leftMotor  = LargeMotor(OUTPUT_B)
run = True
while(run == True):
	leftmotorspeed = input("Left motor speed: ")
	rigthmotorspeed = input("Right motor speed: ")
	time = input("Time: ")
	if(leftmotorspeed == 0 and rightmotorspeed == 0 and time == 0):
		sys.exit("Test finished")
	leftMotor.run_timed(speed_sp = leftmotorspeed, time_sp = time)
	rightMotor.run_timed(speed_sp = rightmotorspeed, time_sp = time)
	while any(m.state for m in (leftMotor, rightMotor)):
		sleep(0.1)
	rightMotor.stop()
	leftMotor.stop()
	print("Done")
	input()
