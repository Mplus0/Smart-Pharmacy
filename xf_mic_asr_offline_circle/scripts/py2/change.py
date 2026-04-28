# -*- coding: utf-8 -*-
import serial
import struct

ser = serial.Serial('/dev/xfserial', 115200)

def changehuan():
	head=0xA5
	userid=0x01
	msgtype=0x05
															#唤醒词
	msg='{"type": "wakeup_keywords","content": {"keyword": "xiao3 yi4 tong2 xue2","threshold": "900"}}\n'
	
	msglen = len(msg)
	msg_h = (msglen >> 8) & 0xFF
	msg_l = msglen & 0xFF
	
	msgid_l=0x01
	msgid_h=0x00
	checksum = ((~sum([head, userid, msgtype, msg_l, msg_h, msgid_l, msgid_h] + [ord(c) for c in msg])) & 0xFF) +1
	head_byte = chr(head)
	userid_byte = chr(userid)
	msgtype_byte = chr(msgtype)
	msg_l_byte = chr(msg_l)
	msg_h_byte = chr(msg_h)
	msgid_l_byte = chr(msgid_l)
	msgid_h_byte = chr(msgid_h)
	checksum_byte = chr(checksum)
	complete_msg = head_byte + userid_byte + msgtype_byte + msg_l_byte + msg_h_byte + msgid_l_byte + msgid_h_byte + msg + checksum_byte
	return complete_msg



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
	print head,userid,msgtype,data_len,msgid,data,check
	break

ser.write(changehuan())

while 1:
	head = ser.read(1).encode('hex')
	if head == "a5":
		userid = ser.read(1).encode('hex')
		msgtype = ser.read(1).encode('hex')

		len_l=ser.read(1).encode('hex')
		len_h=ser.read(1).encode('hex')	
		data_len = int(len_h+len_l, 16)

		msgid = ser.read(2).encode('hex')
		data = ser.read(data_len)
		check = ser.read(1).encode('hex')
		if msgtype=="ff":
			print "更改完成 请重新上电"
			break

ser.close()
