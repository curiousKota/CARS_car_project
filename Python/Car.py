

#This program manages the data input from the arduino and sends commands via pyserial. It runs a loop to constantly update the variables that describe the car. It also must record these data points in a document

'''What i need to make progress:
	pyserial trial and error with an arduino
	(ok i want to have tokens that correspond to the max number of messages i can send without response, every time i get a response I can add a token and every
	time i send a message i can take away a token, I would like to do these things on different threads because that would be super nice.)
	update loop
	openCV integrating the camera data
	sensor fusion into variables that can be used by the neural net
	harddrive data storage in terms of these variables that we gonna use

'''
from robust_serial import Order, write_order, write_i8, write_i16
from robust_serial.utils import open_serial_port

throttleVal;#from throttle pot
regenBrakeVal;#from braking pot
mechanicalBrakeVal; #from hand brake pot
turnVal;#from pot belt driven from shaft
accelerationVal;#from accelerometer (BNO055 could be used which gives absolute orientation by fusing gyro accelerometer and compass)
distanceVal;#from encoder
#camera 1
#camera 2
#camera 3







