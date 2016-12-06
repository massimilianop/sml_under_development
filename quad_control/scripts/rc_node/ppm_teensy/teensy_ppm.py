
from __future__ import print_function

import serial
import time
import struct 
import threading
import binascii
import sys

from types import *

# TODO: put everything in another thread

class aSerial(threading.Thread):
	def __init__(self,port,baudrate):
		threading.Thread.__init__(self)
		ser = serial.Serial(port)
		ser.setDTR(False)
		ser.flush()
		time.sleep(1)
		ser.setDTR(True)
		ser.close()

		self.ser = serial.Serial(port,
							baudrate,
							parity=serial.PARITY_NONE,
							stopbits=serial.STOPBITS_ONE,
							timeout=0.1)

 		self.stop = False

 		self.EOP = [b'\x01',b'\x23']
 		self.SOP = [b'\x45',b'\x67']


 		self.id_packet = 0
 		self.errors = 0

 		self._log_data = False
 		self.data = []

 		self.congested = False

 	def close(self):
 		print("** Closing connection **")
 		self.stop = True
 		self.join()

	def disconnect(self):
		self.ser.close()

	def connect(self):
		if self.ser.isOpen()==False:
			self.ser.open()
			return True
		return False

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

	def run(self):
		while self.stop==False:
			data = self.ser.read()
			if len(data)>0:
				#print(data,end="")
				pass

arduino = aSerial("/dev/ttyACM0",115200)

arduino.start()

while arduino.stop==False:
	try:
		n = 2100
		arduino.send_ppm(0,[n,n,n,n,n,n,n,n])
		n = 1100
		arduino.send_ppm(1,[n,n,n,n,n,n,n,n])
		n = 1150
		arduino.send_ppm(2,[n,n,n,n,n,n,n,n])
		time.sleep(1./50)
	except KeyboardInterrupt:
		arduino.close()