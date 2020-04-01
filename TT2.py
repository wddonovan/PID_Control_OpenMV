import sensor, image, time
import pyb
from pyb import UART
from pyb import LED

# Control Algorithm for Team Bubble Sort

# PID values
Kp1 = 35 # proportional value
Kp2 = 48 # slope between ROIs
Kd = 6 # frame derivative

# DutyCycle values
# DutyPercent is the percent DutyCycle being used, Range = 0 - 100
# DutyOffset is the minimum DutyCycle that car will run at
DutyPercent = 55
DutyOffset = 45

# Braking values
# BrakeTime is the time spent braking in ms
# BrakeThreshold is the minimum slope required for the car to brake
BrakeTime = 90
BrakeThreshold = 25

# Pins to VNH
ENA = pyb.Pin("P2", pyb.Pin.OUT_PP) # pin2 is ENA
ENB = pyb.Pin("P3", pyb.Pin.OUT_PP) # pin3 is ENB
INA = pyb.Pin("P8", pyb.Pin.OUT_PP) # pin8 is INA
INB = pyb.Pin("P9", pyb.Pin.OUT_PP) # pin9 is INB

# Set the car to forward mode
ENA.high() # ENA = HIGH (3.3V)
ENB.high() # ENB = HIGH (3.3V)
INA.high() # INA = HIGH (3.3V)
INB.low()  # INB = LOW (0V)

# Initialize LEDs
red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

# Set parameters for OpenMV Cam
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
GRAYSCALE_THRESHOLD = [(225 , 255)]

# Car will stop when true
stop = False

# Regions of interest
ROIS = [ # [x start, y start, width, length, weight]
        (0, 0, 160, 10, 0.1),
        (0, 40, 160, 30, 0.1),
        (0, 80, 160, 10, 0.1)
       ]

# Initialize values
clock = time.clock() # Clock used for PWM signals
pw = 11000 # Pulse width
duty_cycle = 10
SteerError = [0,0,0] # Error values for each ROI
timer = pyb.Timer(4, freq=300)   # timer for PWM
timer2 = pyb.Timer(2, freq=2000) # timer for duty cycle
ch2 = timer.channel(1, pyb.Timer.PWM, pin=pyb.Pin("P7"), pulse_width=16000) # Pulse width = straight
ch3 = timer2.channel(1, pyb.Timer.PWM, pin=pyb.Pin("P6"), pulse_width_percent=0) # Duty cycle = 0
Pos1 = 0 # Derivative terms
Pos2 = 0
Pos3 = 0
Pos4 = 0
finishenable = 0 # Cool down period until finish line detection is active
cross = False # Cross detection
brakeCounter = 0 # Counts number of high duty cycle frames until braking is enabled
i = 0 # Blob iterator
LargeBlobCount = 0 # Count number of large blobs for finishline detection

# Initialize UART for bluetooth
uart = UART(3,115200) # (pin, baud rate)
uart.init(115200, bits=8, parity=None, stop=1, timeout=1000, flow=0)

############# CONTROL CODE ###############
while(True):
    clock.tick()
    img = sensor.snapshot()
    finishenable = finishenable + 1 # Increase until it reaches threshold for finishline detection
	
	# Read blobs in each ROI
    for r in ROIS:
        blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True) # Finds a blob
        if blobs:
            middle_blob = min(blobs, key=lambda b: abs(b.cx()-80))
            SteerError[i] = middle_blob.cx() - 80 # Error = position of middle blob
        for b in blobs:
            img.draw_rectangle(b.rect(), color = 2)
            img.draw_cross(b.cx(),
                b.cy(), color = 0)
			
			# Set cross if there is a very lage blob
            if b.area() > 900:
                cross = True

            if i == 1 and b.area() > 400:
                LargeBlobCount = LargeBlobCount + 1
        # Stops the car if there are more than 3 blobs
        if LargeBlobCount > 2 and finishenable > 250:
            print("Finish line detected with area ", middle_blob.area())
            INA.low()
            INB.high()
            pyb.delay(350) # delay in ms
            INA.high()
            INB.low()
            stop = True
        i = i + 1

	##### PID CONTROLLER #####
    SteerErrorAvg = sum(SteerError) / len(SteerError) # Find the average error between all ROIs
    SteerDiff = SteerError[0] - SteerError[1] # Slope between top and middle ROIs
    SteerDiff2 = (SteerError[0] - SteerError[2]) / 1.6 # Slope between top and bottom ROIs
	
	# Prevent division by 0
    if (SteerDiff == 0):
        SteerDiff = 1
    if (SteerDiff2 == 0):
        SteerDiff2 = 1
    SteerDiff = (SteerDiff + SteerDiff2) / 2 # Average of two slopes

	# Cascading derivative calculation
    Pos1 = Pos2
    Pos2 = Pos3
    Pos3 = Pos4
    Pos4 = SteerError[1]
    Slope1 = (Pos4 - Pos2) / 0.1 # Derivative of 2nd and 4th newest frames
    Slope2 = (Pos3 - Pos1) / 0.1 # Derivative of newest and 3rd newest frames
    SlopeAvg = (Slope1 + Slope2) / 2 # Average of derivatives

	# The big equation
    output = int((Kp1 * SteerErrorAvg) + (Kp2 * SteerDiff) + (Kd * SlopeAvg))
	
	# duty cycle limits
    if output > 2250:
        output = 2250
    if output < -2250:
        output = -2250
	DutyCycle = 100 - abs(output / 22.5) # limit duty cycle range to 0 - 100
    DutyCycle = int((float(DutyCycle) * (float(DutyPercent) / 100.0)) + float(DutyOffset)) # add offset and constrain to percent

	# Steering Parameters
	# Straight = 15857
	# Left limit = 18500
	# Right limit = 13250
    Turn = 15857 - output
	
	# PWM limits
    if Turn < 13250:
        Turn = 13250
    if Turn > 18500:
        Turn = 18500

	# Send duty cycle and PWM to channels
    ch2 = timer.channel(1, pyb.Timer.PWM, pin=pyb.Pin("P7"), pulse_width = Turn)
    if not stop:
        ch3 = timer2.channel(1, pyb.Timer.PWM, pin=pyb.Pin("P6"), pulse_width_percent = DutyCycle)

	# Set LEDs
    if blobs:
        if (middle_blob.cx() > 106):
            red_led.on()
            green_led.off()
            blue_led.off()

        elif (middle_blob.cx() > 53):
            red_led.off()
            green_led.on()
            blue_led.off()

        elif (middle_blob.cx() > 0):
            red_led.off()
            green_led.off()
            blue_led.on()
    else:
        red_led.off()
        green_led.off()
        blue_led.off()

	##### BRAKING #####
    if DutyCycle > 60: # Car must be going fast to enable braking
        brakeCounter = brakeCounter + 1

	# Brake if high threshold, not a cross, and going fast
    if (abs(SteerDiff) > BrakeThreshold) and (cross == False) and (brakeCounter > 60):
       uart.write('braking')
       ch3 = timer2.channel(1, pyb.Timer.PWM, pin=pyb.Pin("P6"), pulse_width_percent = 0)
       brakeCounter = 0
       INA.low()
       INB.high()
       pyb.delay(int(BrakeTime * DutyCycle * 0.017)) # Delay in ms, proportional to duty cycle
       INA.high()
       INB.low()

    # Some braking parameters
    cross = False

######## BLUETOOTH #########
    if (uart.any() != 0): # if there are characters to be read
        p = uart.readchar() # read first character
        print(p)
        if (p == ord('s')): # Stop, ascii: 115
            stop = True
            print("stop = ", stop)
        if (p == ord('r')): # Restart, ascii: 114
            stop = False
            print("stop = ", stop)
            red_led.off()
            green_led.off()
            blue_led.on()
        if (p == ord('p')): # Kp value, ascii: 112
            # Read ascii values and convert to integer.
            n1 = uart.readchar() - 48
            n2 = uart.readchar() - 48
            Kp1 = (n1 * 10) + n2
            print("digit 1 = ", n1, "digit 2 = ", n2, "Kp1 = ", Kp1)
        if (p == ord('d')): # Kd value
            # Read ascii values and convert to integer.
            n1 = uart.readchar() - 48
            n2 = uart.readchar() - 48
            Kd = (n1 * 10) + n2
            print("digit 1 = ", n1, "digit 2 = ", n2, "Kd = ", Kd)
        if (p == ord('l')): # Slope value
            # Read ascii values and convert to integer.
            n1 = uart.readchar() - 48
            n2 = uart.readchar() - 48
            Kp2 = (n1 * 10) + n2
            print("digit 1 = ", n1, "digit 2 = ", n2, "Kp2 = ", Kp2)
        if (p == ord('%')): # Duty cycle percent
            # Read ascii values and convert to integer.
            n1 = uart.readchar() - 48
            n2 = uart.readchar() - 48
            DutyPercent = (n1 * 10) + n2
            print("digit 1 = ", n1, "digit 2 = ", n2, "DutyPercent = ", DutyPercent)
        if (p == ord('o')): # Duty cycle offset
            # Read ascii values and convert to integer.
            n1 = uart.readchar() - 48
            n2 = uart.readchar() - 48
            DutyOffset = (n1 * 10) + n2
            print("digit 1 = ", n1, "digit 2 = ", n2, "DutyOffset = ", DutyOffset)
        if (p == ord('b')): # Brake delay
            # Read ascii values and convert to integer.
            n1 = uart.readchar() - 48
            n2 = uart.readchar() - 48
            BrakeTime = (n1 * 10) + n2
            print("digit 1 = ", n1, "digit 2 = ", n2, "BrakeTime = ", BrakeTime)
        if (p == ord('t')): # Brake threshold
            # Read ascii values and convert to integer.
            n1 = uart.readchar() - 48
            n2 = uart.readchar() - 48
            BrakeThreshold = (n1 * 10) + n2
            print("digit 1 = ", n1, "digit 2 = ", n2, "BrakeThreshold = ", BrakeThreshold)
        if (p == ord('v')): # Print out values
            # Read ascii values and convert to integer.
            uart.write('P: ')
            uart.write(str(Kp1))
            uart.write(' D: ')
            uart.write(str(Kd))
            uart.write(' Slope: ')
            uart.write(str(Kp2))
            uart.write(' DutyPercent: ')
            uart.write(str(DutyPercent))
            uart.write(' DutyOffset: ')
            uart.write(str(DutyOffset))
            uart.write(' BrakeTime: ')
            uart.write(str(BrakeTime))
            uart.write(' BrakeThreshold: ')
            uart.write(str(BrakeThreshold))
            uart.write('\n')

	# If car is stopped
    if (stop):
        red_led.on()
        green_led.on()
        blue_led.on()
        ch3 = timer2.channel(1, pyb.Timer.PWM, pin=pyb.Pin("P6"), pulse_width_percent = 0)
        INA.high()
        INB.low()
