#!/usr/bin/python
import subprocess
import numpy
import math
import arduinoComm
from time import time

DEFAULT_TIMEOUT = 1000
TARGET_DEPTH = 810.0
DEPTH_P = 0.004
ROTATION_P = 0.05
WRITE_TO_ARDUINO = False
FOLLOW_MODE = False

def execute(command):

    if WRITE_TO_ARDUINO:
	comm = arduinoComm.ArduinoComm("/dev/tty.usbmodem1411",9600)
    popen = subprocess.Popen(command, stdout=subprocess.PIPE, stdin=subprocess.PIPE)
    lines_iterator = iter(popen.stdout.readline, b"")
    if FOLLOW_MODE:
	popen.stdin.write("F\n");
    else:
	popen.stdin.write("C\n");
    for line in lines_iterator:
	label_vec = line.split(":")
	if (len(label_vec) > 1):
	    vec = label_vec[1]
	    vec_parts = numpy.array([float(x) for x in vec.split(",")])
	    dir_vec = vec_parts[0:3]
	    depth = vec_parts[3]
	    if label_vec[0] == "O" or label_vec[0] == "P":
		c_angle = math.acos(dir_vec.dot([1,0,0]))*180.0/math.pi
		if c_angle > 90:
		    c_angle = c_angle - 90
		else:
		    c_angle = -(90 - c_angle)
		x = 0
		y = -DEPTH_P*(depth - TARGET_DEPTH)
		if abs(y) <= 0.1:
		    y = 0
		rotation = ROTATION_P*(c_angle)
		print x,y,rotation
		if WRITE_TO_ARDUINO:
		    print dir_vec
		    comm.motorVector(x,y,rotation,DEFAULT_TIMEOUT)
	    elif label_vec[0] == "C":
		pass
	if FOLLOW_MODE:
	    popen.stdin.write("F\n");
	else:
	    popen.stdin.write("C\n");

execute(["kinect_handler/kinect_handler"])
