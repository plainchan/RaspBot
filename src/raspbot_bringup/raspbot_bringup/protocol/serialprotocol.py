#
# MIT License
#
# Copyright (c) 2022 plainchan
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#!/usr/bin/env python3

'''
/****************************************************************************************
*                             Serial Protocol                                           *
*                                                                                       *
*  ,------+------+------+-------+- - - - - -+- - - - - -+- - - - -+- - - - -+           *
*  | Type | SOF  |  LEN |  CRC  |    DPKG   |    DPKG   |   ....  |   CRC   |           *
*  ,------+------+------+-------+-----------+-----------+---------+---------+           *
*  | size |  2   |   1  |   1   |    ...    |    ...    |   ...   |    2    |           *
*  '----- +------+------+-------+- - - - - -+- - - - - -+- - - - -+- - - - -+           *
*  SOF  .........  start of frame, 2 bytes                                              *
*  LEN  .........  number of data package in the frame,only contain dpkg                *
*                  LEN = DPKG + DPKG + ...                                              *
*  CRC  .........  Head of Frame's cyclic redundancy check                              *
*  DPKG .........  data package,include DATA_TAG,DATA                                   *
*  CRC  .........  DATA cyclic redundancy check                                         *   
*                                                                                       *
*                                                                                       *
*  ,------+--------------+---------+---------+--------+                                 *
*  | Type |   element    |  size   |  offset |   seq  |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  |      | frame  head1 | 1 BYTE  |    0    |    1   |                                 *
*  | SOF  ,--------------+---------+---------+--------+                                 *
*  |      | frame  head2 | 1 BYTE  |    1    |    2   |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  | LEN  |              | 1 BYTE  |    2    |    3   |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  | CRC  |              | 1 BYTES |    3    |    4   |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  |      |   DATA_TAG   | 1 BYTE  |    4    |    5   |                                 *
*  | DPKG ,--------------+---------+---------+--------+                                 *
*  |      |   DATA       | n BYTES |    5    |    6   |                                 *
*  ,---------------------+---------+---------+--------+                                 *
*  | CRC  |              | 2 BYTES |    4+n  |   n+5  |                                 *
*  '------'--------------+---------+---------+--------+                                 *  
*                                                                                       *
*  Endian: Little-Endian                                                                *
*****************************************************************************************/
'''

from ctypes.wintypes import BOOL
import struct
from crc import *



HEADER=(0xFE,0xEF)

FRAME_HEAD_CRC_BYTES  = 1     
FRAME_DPKG_CRC_BYTES  = 2
FRAME_DPKG_LEN_BYTES  = 1     
FRAME_INFO_SIZE       = 2 + FRAME_DPKG_LEN_BYTES + FRAME_HEAD_CRC_BYTES
# FRAME_CALCU_CRC_BYTES = 2 + FRAME_DPKG_LEN_BYTES

FRAME_HEAD_OFFSET     = 2
FRAME_DPKG_LEN_OFFSET = FRAME_HEAD_OFFSET + FRAME_DPKG_LEN_BYTES
FRAME_HEAD_CRC_OFFSET = FRAME_INFO_SIZE


robot_tag      = 0xA0
speed_tag      = 0xB0
encoder_tag    = 0xC0
imu_tag        = 0xD0
imu_sensor_tag = 0xD1
imu_raw_tag    = 0xD2
voltage_tag    = 0xE0  
pid_tag        = 0x70

speed_dpkg_len  = 0x05
pid_dpkg_len    = 0x07



class SerialProtocol():
    def __init__(self,useIMUMag = False) -> None:
        self.__streamBuffer = bytearray()
        self.__dpkgLen = 0
        self.__useIMUMag = useIMUMag
        
    def setUseIMUMag(self,en:BOOL)->None:
        self.__useIMUMag = en
        
    
    def parse_stream(self,byte):
        self.__streamBuffer.append(byte)
        
        length = len(self.__streamBuffer)
        
        # frame header
        if length == FRAME_HEAD_OFFSET:
            if self.__streamBuffer[length - 2] != HEADER[0] or self.__streamBuffer[length - 1] != HEADER[1]:
                self.__streamBuffer[length - 2] =self.__streamBuffer[length - 1]
                del self.__streamBuffer[length-1:]
      
        # frame dpkg length
        elif length == FRAME_DPKG_LEN_OFFSET:
            self.__dpkgLen = self.__streamBuffer[length - 1]
            
        # frame header crc
        elif length == FRAME_HEAD_CRC_OFFSET:
            headerCRC = self.__streamBuffer[length-1]
            if headerCRC != crc8(self.__streamBuffer[:length-1]):
                print("frame header crc error",headerCRC,crc8(self.__streamBuffer[:length-1]))
                self.__streamBuffer.clear()
            else:
                print("frame header crc right",headerCRC,crc8(self.__streamBuffer[:length-1]))
            
        # one frame of data receieving is completed 
        elif length >= FRAME_INFO_SIZE + self.__dpkgLen + FRAME_DPKG_CRC_BYTES:
            dpkgCRC = self.__streamBuffer[length-1] << 8 | self.__streamBuffer[length-2]
            # dpkgCRC = struct.unpack('<H',self.__streamBuffer[length-2:length])[0]
            dpkgFeild = self.__streamBuffer[FRAME_INFO_SIZE:FRAME_INFO_SIZE + self.__dpkgLen]
            self.__streamBuffer.clear()
            if  dpkgCRC == crc16(dpkgFeild):
                print("dpkg crc right",dpkgCRC,crc16(dpkgFeild))
                return self.__decode(dpkgFeild)
            else:
                print("dpkg crc error",dpkgCRC,crc16(dpkgFeild))
        
        return None
        
        
    # A frame data don't contain repeated data's tags
    def __decode(self,dpkgFeild):
        size = len(dpkgFeild)
        i = 0
        msg_raw = {}
        while i<size:
            if dpkgFeild[i] == robot_tag:
                i+=1
                if self.__useIMUMag:
                    format = '<B2h12f'
                else:
                    format = '<B2h9f'
                offset = struct.calcsize(format)
                msg_raw[robot_tag] = struct.unpack(format,dpkgFeild[i:i+offset])
                i+=offset
            elif dpkgFeild[i] == encoder_tag:
                i+=1
                format = '<2h'
                offset = struct.calcsize(format)
                msg_raw[encoder_tag] = struct.unpack(format,dpkgFeild[i:i+offset])   #robot_msgs.l_encoder_pulse
                i+=offset
           
            elif dpkgFeild[i] == imu_tag:
                i+=1
                if self.__useIMUMag:
                    format = '<12f'
                else:
                    format = '<9f'
                offset = struct.calcsize(format)
                msg_raw[imu_tag] = struct.unpack(format,dpkgFeild[i:i+offset])
                i+=offset
            elif dpkgFeild[i] == imu_sensor_tag:
                i+=1
                if self.__useIMUMag:
                    format = '<12h'
                else:
                    format = '<9h'
                offset = struct.calcsize(format)
                msg_raw[imu_sensor_tag] = struct.unpack(format,dpkgFeild[i:i+offset])
                i+=offset
           
            elif dpkgFeild[i] == imu_raw_tag:
                i+=1
                if self.__useIMUMag:
                    format = '<9h'
                else:
                    format = '<6h'
                offset = struct.calcsize(format)
                msg_raw[imu_raw_tag] = struct.unpack(format,dpkgFeild[i:i+offset])
                i+=offset
            elif dpkgFeild[i] == voltage_tag:
                i+=1
                format = '<B'
                offset = struct.calcsize(format)
                msg_raw[voltage_tag] = struct.unpack(format,dpkgFeild[i:i+offset])    #robot_msgs.voltage 
                i+=offset
            else:
                i+=1
        return msg_raw



