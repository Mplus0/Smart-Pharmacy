# -*- coding: utf-8 -*-
import serial

ser = serial.Serial('/dev/xfserial', 115200)

while 1:
	head = ser.read(1).encode('hex')
	if head != "a5":
		continue
	userid = ser.read(1).encode('hex')
	msgtype = ser.read(1).encode('hex')
	len_l=ser.read(1).encode('hex')
	len_h=ser.read(1).encode('hex')	
	data_len = int(len_h+len_l, 16)
	msgid = ser.read(2).encode('hex')
	data = ser.read(data_len)
	check = ser.read(1).encode('hex')
	print msgtype,data
ser.close()
