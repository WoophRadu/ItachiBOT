# Imports
import time
import importlib.util
import wiringpi as wp
import cv2
from collections import deque
import numpy as np
import imutils

# If script is running on the Raspberry Pi, we'll import RPi.GPIO
# If script is running on Windows for development reasons, we'll import FakeRPi.GPIO
# FakeRPi: https://github.com/sn4k3/FakeRPi
try:
    importlib.util.find_spec('RPi.GPIO')
    import RPi.GPIO as GPIO
except ImportError:
    import FakeRPi.GPIO as GPIO


### Tweakable constants
# Loop cycle delay - How much to wait each main loop before entering the next one
loopDelay = 0.01 # seconds
# Responsiveness - This controls a buffer time between motion control commands, when sent to the controller
delayBetweenCommands = 0.001 # seconds
# PWM Frequency
pwmFreq = 1000 # Hz
# Light Sensor Calibration Cycles
calibCycles = 30
# Absolute Minimum value that the sensors are allowed to take
absoluteMinValue = 30
# Delay between light sensor calibration cycles
calibDelay = 0.1 # seconds

### Global speed variables
# These store individual current speed for the motors, and they get sent to the controller when calling setSpeeds()
speedLeft = 0 # percent
speedRight = 0 # percent

### Global pin variables
# These store pin numbers for the high and low pins connected to the motors through the controller
# They get sent to the controller when calling setPins()
pinHighL = 15
pinLowL = 13
pinHighR = 12
pinLowR = 11
pinPwmL = 8
pinPwmR = 7
pwmL = None
pwmR = None
pinsSensor = [29,31,33,35,37,38]
sensorAmount = len(pinsSensor)
sensorValue = [0,0,0,0,0,0]
calibMax = [0,0,0,0,0,0]
calibMin = [1000, 1000,1000,1000,1000,1000]
speedMultiplier = 1

###################################
### Motor Controlling Functions ###
###################################
def setSpeeds():
    # Sends global speeds to motor controller
    # DOES NOT ACCEPT ARGUMENTS, speed must be set using one of the motion control functions
    global speedLeft, speedRight, delayBetweenCommands
    speedLeft = speedLeft*speedMultiplier
    speedRight = speedRight*speedMultiplier
    if speedLeft>0:
        GPIO.output(pinHighL,GPIO.HIGH)
        GPIO.output(pinLowL,GPIO.LOW)
        pwmL.ChangeDutyCycle(speedLeft)
        print("[PWM] Left speed: " + str(speedLeft) + "%")
    elif speedLeft<0:
        GPIO.output(pinHighL,GPIO.LOW)
        GPIO.output(pinLowL,GPIO.HIGH)
        pwmL.ChangeDutyCycle(-speedLeft)
        print("[PWM] Left speed: " + str(speedLeft) + "%")
    else:
        GPIO.output(pinHighL,GPIO.LOW)
        GPIO.output(pinLowL,GPIO.LOW)
        pwmL.ChangeDutyCycle(0)
        print("[PWM] Left speed: " + str(speedLeft) + "%")
    if speedRight>0:
        GPIO.output(pinHighR,GPIO.HIGH)
        GPIO.output(pinLowR,GPIO.LOW)
        pwmR.ChangeDutyCycle(speedRight)
        print("[PWM] Right speed: " + str(speedRight) + "%")
    elif speedRight<0:
        GPIO.output(pinHighR,GPIO.LOW)
        GPIO.output(pinLowR,GPIO.HIGH)
        pwmR.ChangeDutyCycle(-speedRight)
        print("[PWM] Right speed: " + str(speedRight) + "%")
    else:
        GPIO.output(pinHighR,GPIO.LOW)
        GPIO.output(pinLowR,GPIO.LOW)
        pwmR.ChangeDutyCycle(0)
        print("[PWM] Right speed: " + str(speedRight) + "%")
    #time.sleep(delayBetweenCommands)

def stop():
    # Completly freeze in place
    global speedLeft, speedRight
    print("stop() called")
    speedLeft = 0
    speedRight = 0
    setSpeeds()

def goForward(speed = 100):
    # Full-speed forward movement, unless a speed is given
    # Both motors will be set to the same speed
    global speedLeft, speedRight
    print("goForward(" + str(speed) + ") called")
    speedLeft = speed
    speedRight = speed
    setSpeeds()

def goBackwards(speed = 100):
    # Full-speed backwards movement, unless a speed is given
    # Both motors will be set to the same speed
    # Global speeds will be set to the local -speed, since we'll be going backwards
    global speedLeft, speedRight
    print("goBackwards(" + str(speed) + ") called")
    speedLeft = -speed
    speedRight = -speed
    setSpeeds()

def manevra(speedL = 0, speedR = 0):
    # Custom speed maneuver for both motors
    # Speeds default to 0
    # Can specify individual speeds for one or both motors, positive for forward movement and negative for backward movement
    global speedLeft, speedRight
    print("manevra(" + str(speedL) + ", " + str(speedR) + ") called")
    speedLeft = speedL
    speedRight = speedR
    setSpeeds()

def manevraL(speed = 0):
    # Custom speed maneuver LEFT motor
    # Speed defaults to 0
    # Adresses
    global speedLeft
    print("manevraL(" + str(speed) +") called")
    speedLeft = speed
    setSpeeds()

def manevraR(speed = 0):
    # Custom speed maneuver LEFT motor
    # Speed defaults to 0
    # Adresses
    global speedRight
    print("manevraR(" + str(speed) +") called")
    speedRight = speed
    setSpeeds()

def setup():
    global pwmL, pwmR
    GPIO.setmode(GPIO.BOARD)
    print("GPIO pin mode was set to GPIO.BOARD")
    GPIO.setup(pinHighL, GPIO.OUT)
    print("Using pin " + str(pinHighL) + " as GPIO.OUT")
    GPIO.setup(pinLowL, GPIO.OUT)
    print("Using pin " + str(pinLowR) + " as GPIO.OUT")
    GPIO.setup(pinHighR, GPIO.OUT)
    print("Using pin " + str(pinHighR) + " as GPIO.OUT")
    GPIO.setup(pinLowR, GPIO.OUT)
    print("Using pin " + str(pinLowR) + " as GPIO.OUT")
    GPIO.setup(pinPwmL, GPIO.OUT)
    print("Using pin " + str(pinPwmL) + " as GPIO.OUT")
    GPIO.setup(pinPwmR, GPIO.OUT)
    print("Using pin " + str(pinPwmR) + " as GPIO.OUT")
    pwmL = GPIO.PWM(pinPwmL, pwmFreq)
    print("Initiated PWM instance on pin " + str(pinPwmL))
    pwmR = GPIO.PWM(pinPwmR, pwmFreq)
    print("Initiated PWM instance on pin " + str(pinPwmR))
    pwmL.start(0)
    print("[PWM] Left speed: 0%")
    pwmR.start(0)
    print("[PWM] Right speed: 0%")
    print("RPi.GPIO setup complete!")
    print("Setting up WiringPi for the line sensors...")
    wp.wiringPiSetupPhys()
    for pin in pinsSensor:
        wp.pullUpDnControl(pin, wp.PUD_DOWN)
    print("WiringPi setup complete!")

def reset():
    GPIO.cleanup()
    print("GPIO was cleaned up.")
    setup()

#############################
### Line Sensor Functions ###
#############################

def pulseIn(pin):
    wp.pinMode(pin, wp.OUTPUT)
    wp.digitalWrite(pin, wp.HIGH)
    startTime = wp.micros()
    lastTime = startTime
    wp.pinMode(pin, wp.INPUT)
    while wp.digitalRead(pin) == 1:
        lastTime = wp.micros()
    return lastTime-startTime

def sensorReadRaw():
    i=0
    for pin in pinsSensor:
        sensorValue[i] = pulseIn(pin)
        i=i+1

def sensorOutput():
    print(sensorValue)

def calibrate():
    global sensorValue, calibMax, calibMin
    for j in range(0, calibCycles):
        sensorReadRaw()
        for i in range(0, sensorAmount):
            if calibMax[i] < sensorValue[i]:
                calibMax[i] = sensorValue[i]
            if calibMin[i] > sensorValue[i] and sensorValue[i] > absoluteMinValue:
                calibMin[i] = sensorValue[i]
        print("Calibration: min=" + str(calibMin) + ", max=" + str(calibMax))
        time.sleep(calibDelay)

def sensorRead():
    global sensorValue, calibMax, calibMin
    sensorReadRaw()
    for i in range(0, sensorAmount):
        calibDelta = calibMax[i] - calibMin[i]
        x = 0
        if calibDelta != 0:
            x = (sensorValue[i] - calibMin[i]) * 1000 / calibDelta
        if x < 0:
            x = 0
        elif x>1000:
            x = 1000
        sensorValue[i] = int(x)

##################################
### Image processing functions ###
##################################

def get_frame(cap, scaling_factor):
    # Capture the frame from video capture object
    ret, frame = cap.read()

    # Resize the input frame
    frame = cv2.resize(frame, None, fx=scaling_factor,
            fy=scaling_factor, interpolation=cv2.INTER_AREA)

    return frame
    

#############################
### Execution begins here ###
#############################

try:
    reset() # If the process gets killed in the middle of something, motors might still be rolling, so this resets whatever the robot is doing
    time.sleep(1) # Let's wait a second so the GPIO setup has some leeway to complete
    calibrate()
    cap = cv2.VideoCapture(0)
    scaling_factor = 0.5
    print("Entering main loop:")
    while True:
        sensorReadRaw()
        #if sensorValue[3]>250:manevra(-50, 100)
        if sensorValue[5] > 290 or sensorValue[4] > 290:
            manevra(0,100)
        elif sensorValue[0] > 290 or sensorValue[1] > 290:
            manevra(100, 0)
        else:
            goForward(25)
        sensorOutput()
        
        frame = get_frame(cap, scaling_factor)

        # Convert the HSV colorspace
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define 'blue' range in HSV colorspace
        lower_green = np.array([40, 65, 60])
        upper_green = np.array([89, 255, 255])

        lower_red1 = np.array([150, 100, 70])
        upper_red1 = np.array([179, 255, 255])
        lower_red2 = np.array([0, 120, 100])
        upper_red2 = np.array([10, 255, 255])

        lower_blue = np.array([90, 100, 70])
        upper_blue = np.array([128, 255, 255])

        lower_yellow = np.array([20, 50, 60])
        upper_yellow = np.array([35, 255, 255])

        # Threshold the HSV image to get only blue color
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_green = cv2.erode(mask_green, None, iterations=2)
        mask_green = cv2.dilate(mask_green, None, iterations=2)

        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_blue = cv2.erode(mask_blue, None, iterations=2)
        mask_blue = cv2.dilate(mask_blue, None, iterations=2)

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red1 = cv2.erode(mask_red1, None, iterations=2)
        mask_red1 = cv2.dilate(mask_red1, None, iterations=2)

        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red2 = cv2.erode(mask_red2, None, iterations=2)
        mask_red2 = cv2.dilate(mask_red2, None, iterations=2)

        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
        mask_yellow = cv2.dilate(mask_yellow, None, iterations=2)

        # Bitwise-AND mask and original image
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        #res = cv2.medianBlur(res, 5)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts_green = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]

        cnts_blue = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)[-2]

        cnts_red = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)[-2]

        cnts_yellow = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)[-2]
        center_green = None
        center_blue = None
        center_red = None
        center_yellow = None
        radius_red = 0
        radius_green = 0
        radius_blue= 0
        radius_yellow = 0
        x=0
        y=0

        # only proceed if at least one contour was found
        if len(cnts_green) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts_green, key=cv2.contourArea)
            ((x, y), radius_green) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center_green = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if len(cnts_blue) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts_blue, key=cv2.contourArea)
            ((x, y), radius_blue) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center_blue = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if len(cnts_red) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts_red, key=cv2.contourArea)
            ((x, y), radius_red) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center_red = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if len(cnts_yellow) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts_yellow, key=cv2.contourArea)
            ((x, y), radius_yellow) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center_yellow = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        '''cv2.imshow('Original image', frame)
        cv2.imshow('Green', mask_green)
        cv2.imshow('Blue', mask_blue)
        cv2.imshow('Red', mask_red)
        cv2.imshow('Yellow', mask_yellow)'''
        radius_max = max(radius_blue, radius_green, radius_red, radius_yellow)
        if radius_max > 10:
            if radius_max==radius_blue:
                print("max: blue, radius=" + str(radius_max))
                cv2.circle(frame, (int(x), int(y)), int(radius_max),
                           (0, 255, 255), 2)
                cv2.circle(frame, center_blue, 5, (0, 0, 255), -1)
            elif radius_max==radius_red:
                print("max: red, radius=" + str(radius_max))
                cv2.circle(frame, (int(x), int(y)), int(radius_max),
                           (0, 255, 255), 2)
                cv2.circle(frame, center_red, 5, (0, 0, 255), -1)
            elif radius_max==radius_green:
                print("max: green, radius=" + str(radius_max))
                cv2.circle(frame, (int(x), int(y)), int(radius_max),
                           (0, 255, 255), 2)
                cv2.circle(frame, center_green, 5, (0, 0, 255), -1)
            elif radius_max==radius_yellow:
                print("max: yellow, radius=" + str(radius_max))
                cv2.circle(frame, (int(x), int(y)), int(radius_max),
                           (0, 255, 255), 2)
                cv2.circle(frame, center_yellow, 5, (0, 0, 255), -1)
        if radius_max > 100:
            if radius_max==radius_blue:
                speedMultiplier = 0.33
            elif radius_max==radius_red:
                speedMultiplier = 1
            
        cv2.circle(frame, center_green, 5, (0, 0, 255), -1)

        # Check if the user pressed ESC key
        c = cv2.waitKey(5)
        if c == 27:
            break

        
        
        #time.sleep(loopDelay)
    cv2.destroyAllWindows()
except AttributeError:
    pass
# Write sum code here
