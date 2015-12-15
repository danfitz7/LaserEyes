#import zmq
import time
import math
#from scipy.interpolate import interp1d
import serial
import struct

arduino_port = '/dev/cu.usbmodem14131'

def map_ranges(val, from_a, from_b, to_a, to_b):
    return (val - from_a) * (to_b - to_a)/(from_b-from_a) + to_a

ser = serial.Serial(arduino_port, timeout=None, baudrate=115200) # Establish the connection on a specific port

# Sends a turret aim command: "TAYYPP" where YY is the yaw and PP is the pitch (both 2-byte signed shorts)
def turret_aim(yaw, pitch):
    print "SEND TURRET AIM (", yaw, ",", pitch, ")"

    #yawByte =   floatToCommandByte(yaw)
    #pitchByte = floatToCommandByte(pitch)

    yawBytes = struct.pack(">h", yaw)
    pitchBytes = struct.pack(">h", pitch)

    print "Y", yaw, "->", int(ord(yawBytes[0])), int(ord(yawBytes[1])), "\tP",pitch, "->", int(ord(pitchBytes[0])) , int(ord(pitchBytes[1]))  

    message = bytearray(['T', 'A'])
    message.extend(yawBytes)
    message.extend(pitchBytes)
    #print "Raw message: " , message

    if (ser.write(message) != 5):
        print "ERROR: Turret aim message wrong size!"

# Sends the Turret Fire command: "TF" (Turret Fire)
def turret_fire():
    print "SEND TURRET FIRE!"

    message = bytearray(['T', 'F'])
    if (ser.write(message) != 2):
        print "ERROR: Turret fire message wrong size!"

# Prints serial messages from arduino and waits a specific time (usually for the turret to aim somewhere or fire)
def checkup():
    bytesToRead = ser.inWaiting()
    received = ser.read(bytesToRead).replace("\n", "\n\t")
    print received
    time.sleep(sleepTime)

sleepTime = 5
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
    
    turret_fire():
    checkup()

    
# MAIN LOOP
print "STARTING LASER TRACKING..."
while True:
    test_turret()
    

ser.close()
