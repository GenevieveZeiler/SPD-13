#------------------------- SPD-13 - SER300 DEAKIN 2020 -------------------------#
#       Script Name:    SPD-13.py                                               #
#       Programmer:     Genevieve Zeiler                                        #
#       Version:        3.7                                                     #
#       Status:         Working...                                              #
#       Software:       Python, OpenCV                                          #
#       Hardware:       Raspberry Pi Zero W; TB6612FNG Motor Driver;            #
#                         Raspberry Pi Camera Board v2.1; 3xLEDS; 2xButtons.    #
#       Description:    Task one or two on button push                          #
#		    ToDo:                                                                   #
#-------------------------------------------------------------------------------#
#!/usr/bin/env python3

# IMPORT REQUIRED STUFF
print("BOOTING")
import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

#CAMERA SETUP
vid_cam = cv2.VideoCapture(0)
vid_cam.set(cv2.CAP_PROP_FPS, 30)
vid_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
vid_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

#BALL HIGH LOW HSV LEVLES
ilowH = 2
ihighH = 20
ilowS = 108
ihighS = 233
ilowV = 130
ihighV = 255

defaultPWM = 50
pwmBound = float(defaultPWM)
cameraBound = float(160)
kp = pwmBound / cameraBound
imageCenter = 160
imageCenterBuffer = 10
leftBearing = int(imageCenter - imageCenterBuffer)
rightBearing = int(imageCenter + imageCenterBuffer)


#PCB PIN SETUP
motorStandBy = 22           #STBY
motorRightForward = 17      #AIN1
motorRightBackward = 27     #AIN2
motorLeftBackward = 23      #BIN1
motorLeftForward = 24       #BIN2
motorRightPWM = 18          #PWMAS
motorLeftPWM = 13           #PWMB
locateLED = 12              #LED1
acquireLED = 25             #LED2
returnLED = 6               #LED3
taskOneButton = 16          #TASK1
taskTwoButton = 26          #TASK2

#GPIO SETUP
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(motorLeftBackward, GPIO.OUT)
GPIO.setup(motorLeftForward, GPIO.OUT)
GPIO.setup(motorRightForward, GPIO.OUT)
GPIO.setup(motorRightBackward, GPIO.OUT)
GPIO.setup(motorLeftPWM, GPIO.OUT)
GPIO.setup(motorRightPWM, GPIO.OUT)
GPIO.setup(motorStandBy, GPIO.OUT)
GPIO.setup(locateLED, GPIO.OUT)
GPIO.setup(acquireLED, GPIO.OUT)
GPIO.setup(returnLED, GPIO.OUT)
GPIO.setup(taskOneButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(taskTwoButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#PWM SETUP
rightMotorPWM = GPIO.PWM(motorRightPWM, 50)
leftMotorPWM = GPIO.PWM(motorLeftPWM, 50)
rightMotorPWM.start(defaultPWM)
leftMotorPWM.start(defaultPWM)


###############################################################################
#   Method      : LED_Start
#   Inputs      : locateLED, acquireLED, returnLED LOW
#   Outputs     : locateLED, acquireLED, returnLED LOW
#   Description : Used as menu indicator, to show tasks are ready to be selected
#                   or finished
#   Notes       :   
###############################################################################

def LED_Start():
        GPIO.output(locateLED, GPIO.HIGH)
        GPIO.output(acquireLED, GPIO.LOW)
        GPIO.output(returnLED, GPIO.LOW) # Turn Locate LED on
        time.sleep(1) # Sleep for one second
        GPIO.output(locateLED, GPIO.HIGH)
        GPIO.output(acquireLED, GPIO.HIGH)
        GPIO.output(returnLED, GPIO.LOW) # Turn Locate and Acquire LED on
        time.sleep(1) # Sleep for one second
        GPIO.output(locateLED, GPIO.LOW)
        GPIO.output(acquireLED, GPIO.HIGH)
        GPIO.output(returnLED, GPIO.LOW) # Turn Locate off, Acquire on and Return LED off
        time.sleep(1) # Sleep for one second
        GPIO.output(locateLED, GPIO.LOW)
        GPIO.output(acquireLED, GPIO.HIGH)
        GPIO.output(returnLED, GPIO.HIGH) # Turn Locate off, Acquire and Return LED on
        time.sleep(1) # Sleep for one second
        GPIO.output(locateLED, GPIO.LOW)
        GPIO.output(acquireLED, GPIO.LOW)
        GPIO.output(returnLED, GPIO.HIGH) # Turn Locate, Acquire off and Return LED on
        time.sleep(1) # Sleep for one second
        Reset_LED()
        
def Reset_LED():
    GPIO.output(locateLED, GPIO.LOW) # Turn Locate LED off
    GPIO.output(acquireLED, GPIO.LOW) # Turn Acquire LED off
    GPIO.output(returnLED, GPIO.LOW) # Turn Return LED off
    
    
###############################################################################
#   Method      : Motor Controller
#   Inputs      : Motor Driver Pins: AIN1, AIN2, BIN1, BIN2, STBY
#   Outputs     : forward, backward, left, right. leftPivot, rightPivot, 
#                 resetMotors
#   Description : Controllers motors for navigation
#   Notes       : Add Advanced Differental Drive  
###############################################################################
    
def forward(R_PWM, L_PWM):
    GPIO.output(motorStandBy, GPIO.HIGH)
    GPIO.output(motorRightForward, GPIO.HIGH)
    GPIO.output(motorRightBackward, GPIO.LOW)
    GPIO.output(motorLeftForward, GPIO.HIGH)
    GPIO.output(motorLeftBackward, GPIO.LOW)
    rightMotorPWM.ChangeDutyCycle(R_PWM)
    leftMotorPWM.ChangeDutyCycle(L_PWM)
    
def backward(R_PWM, L_PWM):
    GPIO.output(motorStandBy, GPIO.HIGH)
    GPIO.output(motorRightForward, GPIO.LOW)
    GPIO.output(motorRightBackward, GPIO.HIGH)
    GPIO.output(motorLeftForward, GPIO.LOW)
    GPIO.output(motorLeftBackward, GPIO.HIGH)
    rightMotorPWM.ChangeDutyCycle(R_PWM)
    leftMotorPWM.ChangeDutyCycle(L_PWM)

def left(R_PWM, L_PWM):
    GPIO.output(motorStandBy, GPIO.HIGH)
    GPIO.output(motorRightForward, GPIO.LOW)
    GPIO.output(motorRightBackward, GPIO.LOW)
    GPIO.output(motorLeftForward, GPIO.HIGH)
    GPIO.output(motorLeftBackward, GPIO.LOW)
    rightMotorPWM.ChangeDutyCycle(R_PWM)
    leftMotorPWM.ChangeDutyCycle(L_PWM)
    
def right(R_PWM, L_PWM):
    GPIO.output(motorStandBy, GPIO.HIGH)
    GPIO.output(motorRightForward, GPIO.HIGH)
    GPIO.output(motorRightBackward, GPIO.LOW)
    GPIO.output(motorLeftForward, GPIO.LOW)
    GPIO.output(motorLeftBackward, GPIO.LOW)
    rightMotorPWM.ChangeDutyCycle(R_PWM)
    leftMotorPWM.ChangeDutyCycle(L_PWM)

def leftPivot(R_PWM, L_PWM):
    GPIO.output(motorStandBy, GPIO.HIGH)
    GPIO.output(motorRightForward, GPIO.LOW)
    GPIO.output(motorRightBackward, GPIO.HIGH)
    GPIO.output(motorLeftForward, GPIO.HIGH)
    GPIO.output(motorLeftBackward, GPIO.LOW)
    rightMotorPWM.ChangeDutyCycle(R_PWM)
    leftMotorPWM.ChangeDutyCycle(L_PWM)
    
def rightPivot(R_PWM, L_PWM):
    GPIO.output(motorStandBy, GPIO.HIGH)
    GPIO.output(motorRightForward, GPIO.HIGH)
    GPIO.output(motorRightBackward, GPIO.LOW)
    GPIO.output(motorLeftForward, GPIO.LOW)
    GPIO.output(motorLeftBackward, GPIO.HIGH)
    rightMotorPWM.ChangeDutyCycle(R_PWM)
    leftMotorPWM.ChangeDutyCycle(L_PWM)
    
def resetMotors():
    GPIO.output(motorStandBy, GPIO.LOW)
    GPIO.output(motorRightForward, GPIO.LOW)
    GPIO.output(motorRightBackward, GPIO.LOW)
    GPIO.output(motorLeftForward, GPIO.LOW)
    GPIO.output(motorLeftBackward, GPIO.LOW)
    rightMotorPWM.ChangeDutyCycle(0)
    leftMotorPWM.ChangeDutyCycle(0)
        
###############################################################################
#   Method      : Task_One
#   Inputs      : 
#   Outputs     : 
#   Description : The robot travels to each collection zone, changes status  
#                   indicator  when  itreaches  each  corner.
#   Notes       :   
###############################################################################
def Task_One():
    GPIO.output(locateLED, GPIO.HIGH)
    forward(100, 90)
    time.sleep(3) #forward for 3 second
    Reset_LED()     #stop led
    
    GPIO.output(acquireLED, GPIO.HIGH)
    rightPivot(100, 95)
    time.sleep(0.75) #left Pivot for 0.75 second
    Reset_LED()     #stop led
    
    GPIO.output(locateLED, GPIO.HIGH)
    forward(100, 95)
    time.sleep(3) #forward for 3 second
    Reset_LED()     #stop led
    
    GPIO.output(acquireLED, GPIO.HIGH)
    rightPivot(100, 95)
    time.sleep(0.75) #left Pivot for 0.75 second
    Reset_LED()     #stop led
    
    GPIO.output(locateLED, GPIO.HIGH)
    forward(100, 95)
    time.sleep(3) #forward for 3 second
    Reset_LED()     #stop led
    
    GPIO.output(acquireLED, GPIO.HIGH)
    rightPivot(100, 95)
    time.sleep(1.1) #forward for 1.1 second
    Reset_LED()     #stop led
    
    GPIO.output(returnLED, GPIO.HIGH)
    forward(80, 75)
    time.sleep(0.065)    #forward for 0.065 second
    Reset_LED()     #stop led

    GPIO.output(returnLED, GPIO.HIGH)
    time.sleep(3) #forward for 3 second
    Reset_LED()     #stop led
    resetMotors()

    main()
    
###############################################################################
#   Method      : Task_Two
#   Inputs      : 
#   Outputs     : task two
#   Description : Finds ball, move towards, drives over, return to base,
#                 checks to see if ball is still there, if so repeat if not
#                 return to main 
#   Notes       :   
###############################################################################
def Task_Two():
    GPIO.output(locateLED, GPIO.HIGH) # Locate LED On
    while vid_cam.isOpened():
        ret, image_frame = vid_cam.read()
        hsv_frame = cv2.cvtColor(image_frame, cv2.COLOR_BGR2HSV)

        lower_hsv = np.array([ilowH, ilowS, ilowV])
        higher_hsv = np.array([ihighH, ihighS, ihighV])
        mask = cv2.inRange(hsv_frame, lower_hsv, higher_hsv)
        kernel = np.ones((20,20),np.uint8)
        erode = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)

        _, contours, _  = cv2.findContours(erode, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        centres = []
        for i in range(len(contours)):
            moments = cv2.moments(contours[i])
            if moments['m00'] !=0:
                centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
            else:
                centres.append((0,0))
                
            ballArea = cv2.contourArea(contours[i])
            ballCentreX = int(centres[i][0])
            ballCentreY = int(centres[i][1])
            
            if (ballArea == 0 ):  # Ball not in sight of camera
                print("no ball")
                forward(100,95)
                time.sleep(0.4)
                resetMotors()
                time.sleep(0.35)
                if (ballArea == 0):
                    Reset_LED()
                    GPIO.output(acquireLED, GPIO.HIGH)
                    forward(100, 95)
                    time.sleep(1)
                    Reset_LED()
                    GPIO.output(returnLED, GPIO.HIGH)
                    leftPivot(100,95)
                    time.sleep(1.5)
                    forward(100,95)
                    time.sleep(2.5)
                    resetMotors()
                    Reset_LED()
                    main()
                        
            elif (ballCentreX < leftBearing) or (ballCentreX > rightBearing):
                error = imageCenter - ballCentreX
                pwmOut = abs(error * kp) 
                print (ballCentreX)
                turnPWM = pwmOut + defaultPWM
                if ballCentreX < (leftBearing):
                    print ("left")         
                    forward(turnPWM, defaultPWM)
                elif ballCentreX > (rightBearing):
                    print ("right")
                    forward(defaultPWM, turnPWM)
                else:
                    print (ballCentreX)
                    print ("onwards")
                    forward(defaultPWM, defaultPWM)
            else:
                print (ballCentreX)
                print ("onwards")
                forward(defaultPWM, defaultPWM)

            
###############################################################################
#   Method      : main()
#   Outputs     : Task_One() Task_Two()
#   Inputs      : taskOneButton, taskTwoButton
#   Description : main menu
#   Notes       :   
###############################################################################
def main():
    resetMotors()
    LED_Start()
    print("TASK SELECTION READY")
    while True: #Run Forever
        task1 = GPIO.input(taskOneButton)
        task2 = GPIO.input(taskTwoButton)
        if task1 == False:
            print("TASK ONE")
            Reset_LED()
            time.sleep(5)
            Task_One()
        elif task2 == False:
            print("TASK TWO")
            Reset_LED()
            time.sleep(5)
            Task_Two() 
    vid_cam.release()
    
if __name__ == "__main__":
    main()
