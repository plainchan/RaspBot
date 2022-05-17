# 1.简介
raspbot是一个基于ROS的小车。它采用树莓派4B作为微型电脑，STM32F103C8T6作为下位机控制小车移动，两者之间依靠串口进行通信。

# 2.串口协议
```
/****************************************************************************************
*                             Serial Protocol                                           *
*                                                                                       *
*  ,------+------+-------+- - - - - -+- - - - - -+- - - - -+                            *
*  | SOF  |  LEN | CRC16 |    DPKG   |    DPKG   |   ....  |                            *
*  |  2   |   1  |   2   |    ...    |    ...    |   ...   |                            *
*  '----- +------+-------+- - - - - -+- - - - - -+- - - - -+                            *
*  SOF  .........  start of frame, 2 bytes                                              *
*  LEN  .........  number of data package in the frame                                  *
*  CRC  .........  cyclic redundancy check                                              *
*  DPKG .........  data package,include DATA_TAG,DATA                                   *
*                                                                                       *
*  ,------+--------------+---------+---------+                                          *
*  |      |    Type      |  size   |  offset |                                          *
*  ,------,--------------+---------+---------+                                          *
*  |      | frame  head1 | 1 BYTE  |    0    |                                          *
*  | SOF  ,--------------+---------+---------+                                          *
*  |      | frame  head2 | 1 BYTE  |    1    |                                          *
*  ,------,--------------+---------+---------+                                          *
*  | LEN  |              | 1 BYTE  |    2    |                                          *
*  ,------,--------------+---------+---------+                                          *
*  | CRC  |              | 2 BYTES |    3    |                                          *
*  ,------,--------------+---------+---------+                                          *
*  |      | DATA_TAG     | 1 BYTE  |    5    |                                          *
*  | DPKG ,--------------+---------+---------+                                          *
*  |      | DATA         | n BYTES |    6+n  |                                          *
*  ,---------------------+---------+---------+                                          *
*  |      | DATA_TAG     | 1 BYTE  |    7    |                                          *
*  | DPKG ,--------------+---------+---------+                                          *
*  |      | DATA         | m BYTES |   7+m   |                                          *
*  ----------------------+---------+---------+                                          *
*                                                                                       *
*  Endian: Little-Endian                                                                *
*****************************************************************************************/
```
