import RPi.GPIO as GPIO
import time
from time import sleep
import numpy as np
import math
import matplotlib.pyplot as plt
import drawnow

def take_reading():#SENSOR FUNCTION TO CALCULATE DISTANCE
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    TRIG = 23
    ECHO = 24
    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)
   
    global pulse_end
    pulse_end = None
    global pulse_start 
    pulse_start = None
    while True:
       GPIO.output(TRIG,True)
       time.sleep(0.0001)
       GPIO.output(TRIG,False)
       print"sensor ok"
       
       while GPIO.input(ECHO)==0:
          pulse_start = time.time()
       while GPIO.input(ECHO)==1:
          pulse_end = time.time()

       pulse_duration =abs(pulse_end) - abs(pulse_start)
       distance= pulse_duration * 17150
       distance=round(distance,2)
       print "Distance:",distance,"cm"
       print(distance)
       x = (distance)
       y = str(x)
       fb = open('/home/pi/mm.txt','a+')
       fb.write(y)
       fb.write('\t')
       fb.close()
       return distance



def servo(angle1):#SERVO MOTOR ROTATION FROM 0 TO 180:
    
     GPIO.setmode(GPIO.BOARD)
     GPIO.setwarnings(False)
     GPIO.setup(22,GPIO.OUT)
     print "hello"
     pwm=GPIO.PWM(22,100)
     pwm.start(10)
     duty1=float(angle1)/10+2.5
    
     print "start"  

     pwm.ChangeDutyCycle(duty1)
     time.sleep(2)

     time.sleep(0.5)

def new():#FUNCTION TO TAKE NEXT READING ON A NEW LINE:
   fb = open('/home/pi/mm.txt','a+')
   fb.write('\n')


def dc_motor_fs():#FUNCTION TO GO FORWARD AND STOP
   
   GPIO.setmode(GPIO.BOARD)
   GPIO.setwarnings(False)

   motor1A = 11 
   motor1B = 12
   motor1E = 13
   motor2A = 38
   motor2B = 36
   motor2E = 40
    
   GPIO.setup(motor1A,GPIO.OUT)
   GPIO.setup(motor1B,GPIO.OUT)
   GPIO.setup(motor1E,GPIO.OUT)
   GPIO.setup(motor2A,GPIO.OUT)
   GPIO.setup(motor2B,GPIO.OUT)
   GPIO.setup(motor2E,GPIO.OUT)

   print"going forward"

   GPIO.output(motor1A,GPIO.HIGH)
   GPIO.output(motor1B,GPIO.LOW)
   GPIO.output(motor1E,GPIO.HIGH)
   GPIO.output(motor2A,GPIO.LOW)
   GPIO.output(motor2B,GPIO.HIGH)
   GPIO.output(motor2E,GPIO.HIGH)
   sleep(0.3)

   print"stop"

   GPIO.output(motor1E,GPIO.LOW)
   GPIO.output(motor2E,GPIO.LOW)
   GPIO.cleanup()
   sleep(4)

def dc_motor_right():#FUNCTION TO TAKE RIGHT

   GPIO.setmode(GPIO.BOARD)
   GPIO.setwarnings(False)

   motor1A = 11
   motor1B = 12
   motor1E = 13
   motor2A = 38
   motor2B = 36
   motor2E = 40

   GPIO.setup(motor1A,GPIO.OUT)
   GPIO.setup(motor1B,GPIO.OUT)
   GPIO.setup(motor1E,GPIO.OUT)
   GPIO.setup(motor2A,GPIO.OUT)
   GPIO.setup(motor2B,GPIO.OUT)
   GPIO.setup(motor2E,GPIO.OUT)

   print"going right"

   GPIO.output(motor1A,GPIO.HIGH)
   GPIO.output(motor1B,GPIO.HIGH)
   GPIO.output(motor1E,GPIO.HIGH)
   GPIO.output(motor2A,GPIO.LOW)
   GPIO.output(motor2B,GPIO.HIGH)
   GPIO.output(motor2E,GPIO.HIGH)
   sleep(0.8)
   GPIO.cleanup()
print "\n"

def total():#FUNCTION TO ROTATE SERVO AND SENSOR:
	new()
	servo(1)
	take_reading()
	servo(45)
        take_reading()
	servo(90)
        take_reading()
	servo(135)
        take_reading()
	servo(180)
        take_reading()
	servo(135)
        take_reading()
	servo(90)
        take_reading()
	servo(45)
        take_reading()
	servo(1)
        take_reading()


#ACTUAL PROGRAM STARTS HERE:
total()#TAKE READINGS
dc_motor_fs()#GO FORWARD AND STOP 
total()
dc_motor_fs()
total()
dc_motor_fs()
total()
dc_motor_fs()
total()
dc_motor_right()#TAKE RIGHT
total()
dc_motor_fs()
total()
dc_motor_fs()
total()
dc_motor_fs()
total()
dc_motor_fs()
total()
dc_motor_right()
total()
dc_motor_fs()
total()
dc_motor_fs()
total()
dc_motor_fs()
total()
dc_motor_fs()
total()
dc_motor_right()
total()
dc_motor_fs()
total()
dc_motor_fs()
total()
dc_motor_fs()
total()
dc_motor_fs()
total()

#MAPPING POLAR TO RECTANGULAR COORDINATES AND PLOTTING GRAPH:
file_content = open('/home/pi/mm.txt')#PATH OF FILE WHERE READINGS ARE STORED
file_content = file_content.readlines()
cnt = 0
theta = [0,45,90,135,180,135,90,45,0]

for lines in file_content[0:]:
	p = lines.split()
	for i in p:
		x = float(i) * np.cos(math.radians(float(theta[cnt])))
		y = float(i) * np.sin(math.radians(float(theta[cnt])))
		print(x,y)
		cnt = cnt+1
		if cnt == 9:
			cnt = 0
	print(p)
	plt.subplot(3,3,1)
	plt.plot(p)
	plt.xlabel('x')
	plt.ylabel('y')
	plt.show()


GPIO.cleanup()

