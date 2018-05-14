# Imports
import time
import importlib.util
import wiringpi as wp

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
loopDelay = 0.05 # seconds
# Responsiveness - This controls a buffer time between motion control commands, when sent to the controller
delayBetweenCommands = 0.01 # seconds
# PWM Frequency
pwmFreq = 50 # Hz
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
pinHighL = 38
pinLowL = 37
pinHighR = 35
pinLowR = 36
pinPwmL = 32
pinPwmR = 31
pwmL = None
pwmR = None
pinsSensor = [7, 11]
sensorAmount = len(pinsSensor)
sensorValue = [0, 0]
calibMax = [0, 0]
calibMin = [1000, 1000]

###################################
### Motor Controlling Functions ###
###################################
def setSpeeds():
    # Sends global speeds to motor controller
    # DOES NOT ACCEPT ARGUMENTS, speed must be set using one of the motion control functions
    global speedLeft, speedRight, delayBetweenCommands
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
    time.sleep(delayBetweenCommands)

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
    global sensorValue
    for i in range(0, calibCycles):
        if calibMax[i] < sensorValue[i]:
            calibMax = sensorValue[i]
        if calibMin[i] > sensorValue[i] and sensorValue[i] > absoluteMinValue:
            calibMin[i] = sensorValue[i]
        time.sleep(calibDelay)

def sensorRead():
    global sensorValue
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

#############################
### Execution begins here ###
#############################

try:
    reset() # If the process gets killed in the middle of something, motors might still be rolling, so this resets whatever the robot is doing
    time.sleep(1) # Let's wait a second so the GPIO setup has some leeway to complete
    calibrate()
    print("Entering main loop:")
    while True:
        sensorRead()
        out()
        time.sleep(loopDelay)
except AttributeError:
    pass
# Write sum code here
