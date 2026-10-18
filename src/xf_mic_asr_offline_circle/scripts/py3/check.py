import serial

ser = serial.Serial('/dev/xfserial', 115200)

while 1:
	head = ser.read(1).hex()
	if head != "a5":
		continue
	userid = ser.read(1).hex()
	msgtype = ser.read(1).hex()
	len_l=ser.read(1).hex()
	len_h=ser.read(1).hex()	
	data_len = int(len_h+len_l, 16)
	msgid = ser.read(2).hex()
	data = ser.read(data_len)
	check = ser.read(1).hex()
	print(msgtype,data)
ser.close()
