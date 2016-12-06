#!/usr/bin/env python
import rospy
from mavros_msgs.msg import OverrideRCIn

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode

import time
import serial
import serial.tools
import array
import sys

import struct 
import threading
import binascii
import copy

from types import *

N_RC = 3

# Add some mora advanced safety features:
class serialRC:
	def __init__(self,port):
		ser = serial.Serial(port=port)

		if not ser.isOpen():
		    print "Port is already opened"
		    ser.close()
		    ser.open()

		#ser.setDTR(False)
		time.sleep(1)
		# toss any data already received, see
		# http://pyserial.sourceforge.net/pyserial_api.html#serial.Serial.flushInput
		ser.flushInput()
		ser.flushOutput()
		ser.setDTR(True)

		ser = serial.Serial(port=port,
		                    baudrate=115200,
		                    bytesize=serial.EIGHTBITS,
		                    parity=serial.PARITY_NONE,
		                    stopbits=serial.STOPBITS_ONE,
		                    timeout=1.0,
		                    xonxoff=0,
		                    rtscts=0
		                    )
		self.ser = ser
		self.chan = [[1500]*8]*N_RC
		self.mode = [1147]*N_RC
		self.cmd= [False]*N_RC
		self.cmd_chan = self.chan

	# chan: array of 8 intergers (between 1000 and 2000)
	def getChan(self,i,data):
		if self.cmd[i]==True:
			self.chan[i] = self.cmd_chan[i]
		else:
			self.chan[i] = list(data.channels)
		self.chan[i][4] = self.mode[i]

	def send_ppm(self,_id,_data):
		assert type(_data) is ListType
		assert len(_data) == 8
		assert type(_id) is IntType
		d = struct.pack('<H8H',_id,*_data)
		cs = sum([ord(p) for p in d])%256
		d += struct.pack('<B',cs)
		head = '\x12\x34'
		tail = '\x56\x78'
		packet = head+d+tail

		#print (">> " + binascii.hexlify(packet))
		self.ser.write(packet)

	def send(self,_id):
		if self.cmd[_id]==True:
			self.send_ppm(_id,self.cmd_chan[_id])
		else:
			self.send_ppm(_id,self.chan[_id])

	def stopRC(self):
		self.ser.close()

	def set_mode(self,i,data):
		if data.custom_mode == 'STABILIZE':
			print('changing to STABILIZE')
			self.mode[i] = 1100
		elif data.custom_mode == 'ACRO':
			print('changing to ACRO')
			self.mode[i] = 1300
		else:
			print('changing to LAND')
			self.mode[i] = 1400
		return True

	def arm(self,i):
		self.cmd[i]=True
		self.cmd_chan[i][2] = 1000
		self.cmd_chan[i][3] = 2000
		print self.cmd_chan[i]
		self.send_ppm(i,self.cmd_chan[i])
		rospy.sleep(3.0)
		self.cmd_chan[i][2] = 1000
		self.cmd_chan[i][3] = 1500
		print self.cmd_chan[i]
		self.send_ppm(i,self.cmd_chan[i])
		self.cmd[i]=False

		print('Arming')

	def disarm(self,i):
		self.cmd[i]=True
		self.cmd_chan[i][2] = 1000
		self.cmd_chan[i][3] = 1000
		self.send_ppm(i,self.cmd_chan[i])
		rospy.sleep(3.0)
		self.cmd_chan[i][2] = 1000
		self.cmd_chan[i][3] = 1500
		self.send_ppm(i,self.cmd_chan[i])
		self.cmd[i]=False

		print('Disarming')

	def cmd_arm(self,i,data):
		if data.value:
			self.arm(i)
		else:
			self.disarm(i)
		return {'success':True,'result':0}

def rosRC():
	rospy.init_node('chan2serial', anonymous=True)
	
	port = rospy.get_param("rc_serial_port")

	print "RC connect to " + port
	ser = serialRC(port)

	iris_id = [5,2,3,4,5,6]
	for i in range(0,N_RC):
		j = copy.copy(i)
		prefix = "/Iris"+str(iris_id[j])+"/"
		rospy.Service(prefix + 'mavros/set_mode', SetMode, lambda data,i=i: ser.set_mode(i,data))
		rospy.Service(prefix + 'mavros/cmd/arming', CommandBool, lambda data,i=i: ser.cmd_arm(i,data))
		rospy.Subscriber(prefix + "mavros/rc/override", OverrideRCIn, lambda data,i=i: ser.getChan(i,data))

	rospy.on_shutdown(ser.stopRC)
	rate = rospy.Rate(50)

	while not rospy.is_shutdown():
		try:
			ser.send(0)
			ser.send(1)
		except Exception,e:
			print e
		rate.sleep()

if __name__ == '__main__':
	try:
		rosRC()
	except rospy.ROSInterruptException:
		pass