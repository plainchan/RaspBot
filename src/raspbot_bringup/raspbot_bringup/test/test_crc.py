#!/usr/bin/env python3

import sys
sys.path.append('..')

from crc.crc import *
import random



if __name__ == "__main__":
    
    while True:
        bytes =[]
        for i in range (256):
            bytes.append(random.randint(0,255))
        x1 = crc8(bytes)  
        y1 = crc8_lookup(bytes)
        x2 = crc16(bytes)
        y2 = crc16_lookup(bytes)
        print("crc8:",x1==y1)
        print("crc16:",x2==y2)
        if x1 != y1 or x2 != y2:
            print("error crc8:",x1, y1)
            print("error crc16:",x2,y2)