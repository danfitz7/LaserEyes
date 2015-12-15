import zmq
import time
import math
from scipy.interpolate import interp1d
import serial

#network setup
port = "5000"
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://127.0.0.1:"+port)
#filter by messages by stating string 'STRING'. '' receives all messages
socket.setsockopt(zmq.SUBSCRIBE, '')

# Global calibration vars
laser_upper_left = [-45.0, 40.0]
laser_upper_right = [45.0,45.0]
laser_lower_left = [-45.0,-45.0]
laser_lower_right = [45.0,-45.0]
laser_calibration_points = [laser_upper_left, laser_lower_right]
pupil_calibration_points = [[0.0,0.0,],[0.0,0.0]] # These are set by calibration

def map_ranges(val, from_a, from_b, to_a, to_b):
    return (val - from_a) * (to_b - to_a)/(from_b-from_a) + to_a

def interp_x(pupil_x):
    return map_ranges(pupil_x, pupil_calibration_points[0][0], pupil_calibration_points[1][0], laser_calibration_points[0][0], laser_calibration_points[1][0])

def interp_y(pupil_y):
    return map_ranges(pupil_y, pupil_calibration_points[0][1], pupil_calibration_points[1][1], laser_calibration_points[0][1], laser_calibration_points[1][1])

def getTurretAngleForPupilPos(pupil_pos):
    return (interp_x(pupil_pos[0]), interp_y(pupil_pos[1]))

calibrating = True
calib_registration_time = 2.0
calib_registration_deviation = 0.1

ser = serial.Serial('/dev/cu.usbmodem1411', 9600) # Establish the connection on a specific port
#ser.open()
offset = 75 # we use the high 180 values of [0-255] (s0 [75-255] = [0-180])
def floatToCommandByte(angle):
    return chr(int(min(max(round(angle +   90 + offset), 0), 255)))

def sendLaserTurretPosition(yawPitch):
    print "LASER (", yawPitch[0], ",", yawPitch[1], ")"
    yaw = yawPitch[0]
    pitch = yawPitch[1]

    yawByte =   floatToCommandByte(yaw)
    pitchByte = floatToCommandByte(pitch)

    print "Y", yaw, "->", int(ord(yawByte)), "\tP",pitch, "->", int(ord(pitchByte))

    message = bytearray([' ', yawByte, pitchByte])
    print "Raw message: " , message
    ser.write(message)

def testTurret():
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

def laserTrackPupil(pupil_pos):
    turret_pos = getTurretAngleForPupilPos(pupil_pos)
    sendLaserTurretPosition(turret_pos)

def blink():
    print "BLINK!"

def handlePupilFrame(pupilPosStr):
    x, y = map(float, pupilPosStr.strip('()').split(','))
    print "PUPIL (", x, ", ", y, ")"
    if (x==0.0 and y==0.0):
        blink()
    else:
        laserTrackPupil((x,y))

def handleGazeFrame(gazePosStr, conf):
    norm_x, norm_y = map(float, gazePosStr.strip('()').split(','))
    print "GAZE AT: (", norm_x, ", ", norm_y, ") P(", conf, ")"

def getNextMessage():
    msg = socket.recv()
    items = msg.split("\n") 
    msg_type = items.pop(0)
    items = dict([i.split(':',1) for i in items[:-1] ])
    return msg_type, items

def getNextPupilMessageItems():
    msg_type, items = getNextMessage()
    while (msg_type != "Pupil"):
        msg_type, items = getNextMessage()
    return items

def getNextPupilPos():
    items = getNextPupilMessageItems();
    try:
        x, y = map(float, items['norm_pos'].strip('()').split(','))

        # ignore pupil not found (blink)
        while (x == 0.0 and y == 0.0):
            items = getNextPupilMessageItems();
            x, y = map(float, items['norm_pos'].strip('()').split(','))

        return x,y
    except KeyError:
        pass

def distance(a, b):
    diff_x = a[0] - b[0]
    diff_y = a[1] - b[1]
    return math.sqrt(diff_x * diff_x + diff_y * diff_y)

def sum(a,b):
    return a[0] + b[0], a[1] + b[1]

def normalize(a, d):
    return a[0]/d, a[1]/d

def calibrate():
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
                    

print "PUPIL READER\nwaiting for server..."
calibrate()

print "Running..."
while True:
    msg = socket.recv()
    #print "raw msg:\n", msg

    items = msg.split("\n") 
    msg_type = items.pop(0)
    items = dict([i.split(':',1) for i in items[:-1] ])

    print "\n"
    if msg_type == 'Pupil':
        try:
            #print "Pupil:\nnorm_pos:\t%s\ndiameter:\t%s" %(items['norm_pos'], items['diameter'])
            handlePupilFrame(items['norm_pos'])
        except KeyError:
            pass
 #   elif msg_type == 'Gaze':
 #       try:
 #           #print "Gaze:\nnorm_pos:\t%s" %(items['norm_pos'])
 #           handleGazeFrame(items['norm_pos'], items['confidence'])
 #       except KeyError:
 #           pass
    else:
        # process non gaze position events from plugins here
        pass

ser.close()
