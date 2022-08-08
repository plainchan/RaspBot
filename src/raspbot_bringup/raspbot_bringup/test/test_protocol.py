#!/usr/bin/env python3

import sys
sys.path.append('..')

from protocol.serialprotocol import SerialProtocol
import serial,time


if __name__ =='__main__':

    try:
        sp = serial.Serial(port='/dev/ttyUSB0',baudrate=115200)
        protocol_decode = SerialProtocol()
        try:
            while True:
                    count = sp.inWaiting()
                    data = bytes()
                    msg = None
                    if count >0:
                        data = sp.read(count)
                        print(data)
                    for byte in data:
                        msg = protocol_decode.parse_stream(byte)
                    if msg is not None:
                        print(msg)
                    time.sleep(0.01)
                    
        except serial.serialutil.SerialException:  
            sp.close()                          # Close  serial port
    except serial.SerialException as e:
        print(e.strerror)

    
