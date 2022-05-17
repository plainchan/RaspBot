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
*  ,------+--------------+---------+---------+--------+                                 *
*  | Type |  elem        |  size   |  offset |  byte  |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  |      | frame  head1 | 1 BYTE  |    0    |    1   |                                 *
*  | SOF  ,--------------+---------+---------+--------+                                 *
*  |      | frame  head2 | 1 BYTE  |    1    |    2   |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  | LEN  |              | 1 BYTE  |    2    |    3   |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  | CRC  |              | 2 BYTES |    3    |    5   |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  |      |   DATA_TAG   | 1 BYTE  |    5    |    6   |                                 *
*  | DPKG ,--------------+---------+---------+--------+                                 *
*  |      |   DATA       | n BYTES |   5+n   |   6+n  |                                 *
*  ,---------------------+---------+---------+--------+                                 *
*  |      |   DATA_TAG   | 1 BYTE  |   6+n   |   7+n  |                                 *
*  | DPKG ,--------------+---------+---------+--------+                                 *
*  |      |   DATA       | m BYTES |  6+m+n  |  7+m+n |                                 *
*  ----------------------+---------+---------+--------+                                 *
*                                                                                       *
*  Endian: Little-Endian                                                                *
*****************************************************************************************/

#include <cstdint>
#include <vector>
#include <cstring>

/**
 * 串口传输缓冲区最大大小
 * 
 */
#define MAX_RxBUFF_SIZE   0x01FE                   //510

/**
 *  帧缓冲区最大大小
 * 
 */
#define MAX_BUFF_SIZE      0xFF                             //255
#define FRAME_INFO_SIZE    0x05                             //5  非数据域
#define MAX_DPKG_SIZE    (MAX_BUFF_SIZE-FRAME_INFO_SIZE)    //250

#define Header1                  0xFE
#define Header2                  0xEF

#define robot_tag                0xA0
#define speed_tag                0xB0

typedef union 
{
    float   number;
    uint8_t bytes[4];
}Bytes2Float;

typedef union 
{
    uint16_t   number;
    uint8_t    bytes[2];
}Bytes2U16;
typedef union 
{
    int16_t    number;
    uint8_t    bytes[2];
}Bytes2INT16;
typedef union 
{
    uint32_t   number;
    uint8_t    bytes[4];
}Bytes2U32;

/**
 * @brief status of robot 
 *         
 * 
 */
typedef struct Robot_State_Info_msgs
{
    int8_t     data_tag;
    float      voltage;                 //real voltage = voltage/10
    int16_t    l_encoder_pulse;
    int16_t    r_encoder_pulse;
    float      acc[3];
    float      gyr[3];
    float      mag[3];
    float      elu[3];
}Robot_msgs;

typedef struct receiveStream
{
    uint8_t        len;
    uint16_t       crc;

    uint8_t        stream_buff[MAX_BUFF_SIZE];
    Robot_msgs     robot_msgs;

}Stream_msgs;


/**
 * 发送
 * 
 */
#pragma pack(1) //1 字节对齐
typedef struct 
{
    int8_t  data_tag;
    int16_t velocity;             //real velocity = velocity/1000
    int16_t yaw;                  //real yaw = yaw/1000      -3141<=yaw<=3141                 1 deg = 0.017453292 rad
}Speed_msgs;
#pragma pack()  //结束字节对齐

#pragma pack(1) //1 字节对齐
typedef struct 
{
    uint8_t      header[2];
    uint8_t      len;
    uint16_t     crc;
    Speed_msgs   speed;
}Frame_Speed_msgs;
#pragma pack()  //结束字节对齐



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
