# Python code for MAS.S65 Science Fiction Fabrication Class, Fall 2015
# Daniel Fitzgerald
#
# The project has two parts: 
#  1) A head-mounted 2-DOF laser pointer gimbal linked to a head-worn gaze tracker.
#  The laser points where the user is looking
#  
#  2) A Shoulder-mounted laser-tracking nerf turret. The turret shoots where the laser points.
#  This turret can also be mounted on a standard camera tripod and used as an automatic laser-guided sentry turret.
#
# This code Serves two purposes:
# 1) Listen to gaze tracking frames broadcast by a pupil server over tcp. Analyse the pupil location, map to gimbal angles, and send the angles to the arduino to control the laser gimbal.
# 2) Run its own OpenCV process to track the laser dot and aim the turret at it by sending turret angles to the arduino.

# TODO:
# - Put the message handler in it's own thread
# -veryify laser eye tracking functionality and improve mapping
# - implement dot tracking in main thread

import time
import math
import serial # for communicating with arduino
import struct # for packing bits
import zmq    # thingy for sockets


# Pupil Calibration values (Use these in PupilCapture
# "Pupil Intensity Range" = 20
# Pupil min" = 40.0
#Pupil max" = 100

# Laser gimbal calibration vars
yawRange = 55
maxPitch = 37
minPitch = -37
laser_upper_left = [-yawRange, maxPitch]
laser_upper_right = [yawRange, maxPitch]
laser_lower_left = [-yawRange, minPitch]
laser_lower_right = [yawRange, minPitch]
laser_mid_right = [yawRange+5, 0]
laser_mid_left = [-yawRange-5, 0]
laser_upper_mid = [0,maxPitch]
laser_lower_mid = [0, minPitch]
laser_center = [0,0]
laser_calibration_points = [laser_mid_left, laser_upper_left, laser_upper_mid, laser_upper_right, laser_mid_right, laser_lower_right, laser_lower_mid, laser_lower_left, laser_center]
pupil_calibration_points = [[0.0,0.0]] * len(laser_calibration_points) # These are set by calibration


######################################################## HELPER FUNCTIONS ########################################################
def distanceSquared(a,b):
    diff_x = a[0] - b[0]
    diff_y = a[1] - b[1]
    return (diff_x * diff_x + diff_y * diff_y)

def distance(a, b):
    return math.sqrt(distanceSquared(a,b))

def sum(a,b):
    return a[0] + b[0], a[1] + b[1]

def normalize(a, d):
    return a[0]/d, a[1]/d

def map_ranges(val, from_a, from_b, to_a, to_b):
    return (val - from_a) * (to_b - to_a)/(from_b-from_a) + to_a

def interp_x(pupil_x):
    return map_ranges(pupil_x, pupil_calibration_points[0][0], pupil_calibration_points[1][0], laser_calibration_points[0][0], laser_calibration_points[1][0])

def interp_y(pupil_y):
    return map_ranges(pupil_y, pupil_calibration_points[0][1], pupil_calibration_points[1][1], laser_calibration_points[0][1], laser_calibration_points[1][1])

def invDistWeight(pupil_pos, pupil_calib_point):
    return 1.0/math.pow(distance(pupil_pos, pupil_calib_point), distPow)

def invDistInterp(pupil_pos):
    yaw = 0
    pitch = 0

    weights = [invDistWeight(pupil_pos, pupil_calib_point) for pupil_calib_point in pupil_calibration_points]

    # average
    for i in range(len(laser_calibration_points)):
        yaw   += weights[i] * laser_calibration_points[i][0]
        pitch += weights[i] * laser_calibration_points[i][1]

    # normalize
    weightSum = 0
    for weight in weights:
        weightSum += weight
    yaw /= weightSum
    pitch /= weightSum

    return ((yaw, pitch))

def getTurretAngleForPupilPos(pupil_pos):
    return invDistInterp(pupil_pos)
    #return (interp_x(pupil_pos[0]), interp_y(pupil_pos[1]))

######################################################## GIMBAL INTERFACE ########################################################

gimbalLaserOn = False 3 Keep track of the laser state

# Messaging protocol for the laser gimbal. These should match the corresponding variables in the Arduino sketch
GIMBAL_COMMAND_FRAME_START = 'G'
GIMBAL_LASERON_MESSAGE_INDICATOR = 'L'
GIMBAL_LASEROFF_MESSAGE_INDICATOR = 'l'
GIMBAL_AIM_MESSAGE_INDICATOR = 'A'
GIMBAL_AIM_MESSAGE_DATA_LENGTH = 4

# Sends a turret aim command: "TAYYPP" where YY is the yaw and PP is the pitch (both 2-byte signed shorts)
def gibal_aim(yaw, pitch):
    print "SEND GIMBAL AIM (", yaw, ",", pitch, ")"

    yawBytes = struct.pack(">h", yaw)
    pitchBytes = struct.pack(">h", pitch)

    print "Y", yaw, "->", int(ord(yawBytes[0])), int(ord(yawBytes[1])), "\tP",pitch, "->", int(ord(pitchBytes[0])) , int(ord(pitchBytes[1]))  

    message = bytearray([GIMBAL_COMMAND_FRAME_START, GIMBAL_AIM_MESSAGE_INDICATOR])
    message.extend(yawBytes)
    message.extend(pitchBytes)

    if (ser.write(message) != 2 + TURRET_AIM_MESSAGE_DATA_LENGTH):
        print "ERROR: GImbal aim message wrong size!"

    # Turn the laser on if it was off from blinking
    if (!gimbalLaserOn){
        gimbal_laser_on();

# Sends the Gimbal Laser-ON command: "GO" (GImbal ON)
def gimbal_laser_on():
    print "SEND GIMBAL LASER ON!"

    message = bytearray([GIMBAL_COMMAND_FRAME_START, GIMBAL_LASEROFF_MESSAGE_INDICATOR])
    if (ser.write(message) != 2):
        print "ERROR: Gimbal laser-on message wrong size!"
    gimbalLaserOn = True

# Sends the Gimbal Laser-ON command: "GO" (GImbal ON)
def gimbal_laser_off():
    print "SEND GIMBAL LASER ON!"

    message = bytearray([GIMBAL_COMMAND_FRAME_START, GIMBAL_LASERON_MESSAGE_INDICATOR])
    if (ser.write(message) != 2):
        print "ERROR: Gimbal laser-off message wrong size!"
    gimbalLaserOn = False
        
def testGimbal():
    # quick test
    print "TESTING TURRET"
    sendLaserTurretPosition((-80, 0))
    time.sleep(2)
    for i in range(-80, 80):
        sendLaserTurretPosition((i, 0))
        time.sleep(0.1)
    sendLaserTurretPosition((0, -80))
    time.sleep(2)
    for i in range(-80, 80):
        sendLaserTurretPosition((0, i))
        time.sleep(0.1)
    sendLaserTurretPosition((0, 0))
    time.sleep(2)

######################################################## GIMBAL MAPPING AND CALIBRATION ########################################################

# What to do when the user blinks
# TODO: detect a certain time of blinking and activate the turret firing
# NOTE that this is the only intersection of the otherwise seperate laser gimbal and shoulder turret systems
def blink():
    print "BLINK!"
    gimbal_laser_off();

#get the next pupil message
def getNextMessage():
    msg = socket.recv()
    items = msg.split("\n") 
    msg_type = items.pop(0)
    items = dict([i.split(':',1) for i in items[:-1] ])
    return msg_type, items

# This is meant to be called in an infinite loop
def processPupilMessagesLoop():
    msg_type, items = getNextMessage();

    print "\n"
    if msg_type == 'Pupil':
        try:
            #print "Pupil:\nnorm_pos:\t%s\ndiameter:\t%s" %(items['norm_pos'], items['diameter'])
            pupilPosStr = items['norm_pos']
            x, y = map(float, pupilPosStr.strip('()').split(','))
            print "PUPIL (", x, ", ", y, ")"
            if (x==0.0 and y==0.0):
                blink()
            else:
                turret_pos = getTurretAngleForPupilPos(pupil_pos)
                sendLaserTurretPosition(turret_pos)
        except KeyError:
            pass


# get the next pupil position from the next pupil-specific message
def getNextPupilPos():

    # Get the items data from the next "Pupil" message (as opposed to "Gaze" messages)
    msg_type, items = getNextMessage()
    while (msg_type != "Pupil"):
        msg_type, items = getNextMessage()

    try:
        x, y = map(float, items['norm_pos'].strip('()').split(','))

        # ignore pupil not found (blink)
        while (x == 0.0 and y == 0.0):
            items = getNextPupilMessageItems();
            x, y = map(float, items['norm_pos'].strip('()').split(','))

        return x,y
    except KeyError:
        pass

# Eye-Gimbal mapping calibration routing
# The program points the laser gimbal at a number of calibration point and waits for the user to stare at the dot.
# The gimbal coordinates and corresponding pupil coordinates are stored in a list of calibration points for later mapping/interpolation
def calibrate():
    calib_registration_time = 2.0
    calib_registration_deviation = 0.1

    print "CALIBRATION..."
    for i in range(0,len(laser_calibration_points)):
        #print "Calibration point ", 1+i
        calibration_point = laser_calibration_points[i]
        print "Please look at laser target at ", calibration_point
        
        # send the laser to position and wait for it to get there
        sendLaserTurretPosition(calibration_point)
        time.sleep(1);

        #registration occurs when the user looks at the same area for a certain amount of time
        registered = False
        while (not registered):
            print "Registration failed. retrying..."
            cur_pos = getNextPupilPos()
            print "\t", cur_pos
            
            # reset everything
            start_time = time.time()
            num_readings = 1
            running_sum = cur_pos
            running_average = cur_pos
            dist_from_avg = 0
            ellapsed_time = 0

            # as long as the user looks in generally the same area (within calib_registration_deviation)
            # in other words, looking away too soon will reset the registration process for this point
            while (dist_from_avg <= calib_registration_deviation):
                cur_pos = getNextPupilPos()

                # update statistics
                num_readings +=1
                running_sum = sum(running_sum, cur_pos)
                running_average = normalize(running_sum, num_readings)
                dist_from_avg = distance(running_average, cur_pos)
                ellapsed_time = time.time() - start_time
                print "\t", cur_pos, "\ta=", running_average, "\td=", dist_from_avg, ")\tt=", ellapsed_time, ""

                if (ellapsed_time >= calib_registration_time):
                    registered = True
                    pupil_calibration_points[i] = [coord for coord in running_average]
                    break


######################################################## TURRET CONTROL ########################################################

# Sends a turret aim command: "TAYYPP" where YY is the yaw and PP is the pitch (both 2-byte signed shorts)

        
# Messaging protocol for the shoulder turret. These should match the corresponding variables in the Arduino sketch
TURRET_COMMAND_FRAME_START = 'T'
TURRET_FIRE_MESSAGE_INDICATOR = 'F'
TURRET_AIM_MESSAGE_INDICATOR = 'A'
TURRET_AIM_MESSAGE_DATA_LENGTH = 4

def turret_aim(yaw, pitch):
    print "SEND TURRET AIM (", yaw, ",", pitch, ")"

    #yawByte =   floatToCommandByte(yaw)
    #pitchByte = floatToCommandByte(pitch)

    yawBytes = struct.pack(">h", yaw)
    pitchBytes = struct.pack(">h", pitch)

    print "Y", yaw, "->", int(ord(yawBytes[0])), int(ord(yawBytes[1])), "\tP",pitch, "->", int(ord(pitchBytes[0])) , int(ord(pitchBytes[1]))  

    message = bytearray([TURRET_COMMAND_FRAME_START, TURRET_AIM_MESSAGE_INDICATOR])
    message.extend(yawBytes)
    message.extend(pitchBytes)
    #print "Raw message: " , message

    if (ser.write(message) != 2 + TURRET_AIM_MESSAGE_DATA_LENGTH):
        print "ERROR: Turret aim message wrong size!"

# Sends the Turret Fire command: "TF" (Turret Fire)
def turret_fire():
    print "SEND TURRET FIRE!"

    message = bytearray([TURRET_COMMAND_FRAME_START, TURRET_FIRE_MESSAGE_INDICATOR])
    if (ser.write(message) != 2):
        print "ERROR: Turret fire message wrong size!"

# Helper function for turret testing: Prints serial messages from arduino and waits a specific time (usually for the turret to aim somewhere or fire)
sleepTime = 2
def checkup():
    time.sleep(sleepTime)
    bytesToRead = ser.inWaiting()
    received = ser.read(bytesToRead).replace("\n", "\n\t")
    print received

# Tests the functionalities of the turret
def test_turret():
    turret_aim(0, 0)
    checkup()
    turret_aim(-45,0)
    checkup()
    turret_aim(0,0)
    checkup()
    turret_aim(45,0)
    checkup()
    turret_aim(0,0)
    checkup()
    turret_aim(0,-45)
    checkup()
    turret_aim(0,0)
    checkup()
    turret_aim(0,45)
    checkup()
    
    turret_aim(0, 0)
    checkup()

    turret_fire()
    checkup()

######################################################## MAIN PROGRAM ########################################################

# Serial setup for arduino
arduino_port = '/dev/cu.usbmodem14131'
ser = serial.Serial(arduino_port, timeout=None, baudrate=115200) # Establish the connection on a specific port

#network setup for pupil server
port = "5000"
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:"+port)
#filter by messages by stating string 'STRING'. '' receives all messages
socket.setsockopt(zmq.SUBSCRIBE, '')

calibrate()

print "STARTING LASER TRACKING..."
while True:
    test_turret()
    

ser.close()
