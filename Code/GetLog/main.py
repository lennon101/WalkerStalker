'''
Created on 15/05/2014

@author: Leenix
'''

import serial
import datetime
from serial.serialutil import SerialException

BAUD_RATE = 57600
outputFileName = "logOutput.csv"

# Packet variables
global remainingPacketBytes
remainingPacketBytes = 0

stream = serial.Serial()

# Communications
serialChannel = raw_input("Serial channel: ")
while True:
    try:
        global stream
        stream = serial.Serial(serialChannel, BAUD_RATE)
        break
    except SerialException:
        print("Cannot open serial port")
        serialChannel = raw_input("Serial channel: ")
        

def readXBee():
    # Read in the next byte from the XBee
    c = ord(stream.read())
    
    # If the received byte is an escape character, perform the escape
    if (c == 0x7D):
        c = ord(stream.read())
        c ^= 0x20
    
    global remainingPacketBytes
    remainingPacketBytes -= 1
    
    return int(c)

while True:
    try:
        line = ""
        c = ''
        length = 0
        remainingPacketBytes = 0
        
        # Wait for a start-of-frame character
        while (c != '\x7E'):
            c = stream.read()
        print("Time", datetime.datetime.now())
        
        # Length
        length = readXBee() * 256
        length += readXBee()
        remainingPacketBytes = length
        
               
        # Frame Type
        frameType = format(readXBee(), '02X')
              
                
        # Address
        address = ""
        for x in range(0, 8):
            address += format(readXBee(), '02X')
        print("Address", address)
            
        shortAddress = ""
        for x in range(0, 2):
            shortAddress += format(readXBee(), '02X')

        
        # Receive Options
        receiveOptions = format(readXBee(), '02X')
        
        # Data
        data = ""
        while (remainingPacketBytes > 0):
            data += format(readXBee(), '02X')
        print("Data",data)
        
        # Checksum
        checksum = readXBee()
     
     
        # Write to file
        f = open(outputFileName, 'ab')
        f.write(data)
        f.write('\n')
        f.close()
        
    except KeyboardInterrupt:
        break
        
stream.close()
        

 
