#ifndef _RASPBOT_COMM_HPP_
#define _RASPBOT_COMM_HPP_

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
*  ,--------------+---------+                                                           *
*  |    Type      |  size   |                                                           *
*  ,--------------+---------+                                                           *
*  | frame  head1 | 1 BYTE  |                                                           *
*  ,--------------+---------+                                                           *
*  | frame  head2 | 1 BYTE  |                                                           *
*  ,--------------+---------+                                                           *
*  | LEN          | 1 BYTE  |                                                           *
*  ,--------------+---------+                                                           *
*  | CRC          | 2 BYTES |                                                           *
*  ,--------------+---------+                                                           *
*  | DATA_TAG     | 1 BYTE  |                                                           *
*  ,--------------+---------+                                                           *
*  | DATA         | n BYTES |                                                           *
*  ---------------+---------+                                                           *
*  | DATA_TAG     | 1 BYTE  |                                                           *
*  ,--------------+---------+                                                           *
*  | DATA         | n BYTES |                                                           *
*  ---------------+---------+                                                           *
*                                                                                       *
*                                                                                       *
*****************************************************************************************/

#include <cstdint>
#include <vector>
#include <cstring>

#define MAX_BUFF_SIZE  0xFA               //250

#define Header1                  0xFE
#define Header2                  0xEF

#define robot_tag                0xA0
#define speed_tag                0xB0


/**
 * @brief status of robot 
 *        
 * 
 */
typedef struct 
{
    int8_t     data_tag;
    uint8_t    voltage;                 //real voltage = voltage/10
    uint16_t   l_encoder_pulse;
    uint16_t   r_encoder_pulse;
    float      acc[3];
    float      gyr[3];
    float      mag[3];
    float      elu[3];
}Robot_msgs;

typedef struct 
{
    uint8_t      len;
    Robot_msgs robot_msgs;
}Frame_Robot_msgs;

/*****************************************************************************/
#pragma pack(1) //1 字节对齐
typedef struct 
{
    int8_t  data_tag;
    int16_t velocity;             //real velocity = velocity/1000
    int16_t yaw;                  //real yaw = yaw/1000      -3141<=yaw<=3141                 1 deg = 0.017453292 rad
}Speed_msgs;

typedef struct 
{
    uint8_t      header[2];
    uint8_t      len;
    uint16_t     crc;
    Speed_msgs   speed;
}Frame_Speed_msgs;
#pragma pack()  //结束字节对齐








void pasrse_data()
{

}



















/**
 * @brief 
 * 
 * @tparam T 
 * @param t 
 * @return uint8_t* 
 */
template <typename T>
std::vector<uint8_t> structPack_Bytes(T &T_struct)
{
    int size = sizeof(T_struct);
    std::vector<uint8_t> Bytes(size);
    memcpy(Bytes.data(),&T_struct,size);
    return Bytes;
}

#endif
