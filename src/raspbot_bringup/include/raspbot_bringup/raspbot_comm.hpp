/*
 * MIT License
 *
 * Copyright (c) 2022 plainchan
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#ifndef _RASPBOT_COMM_HPP_
#define _RASPBOT_COMM_HPP_

/****************************************************************************************
*                             Serial Protocol                                           *
*                                                                                       *
*  ,------+------+------+-------+- - - - - -+- - - - - -+- - - - -+- - - - -+           *
*  | Type | SOF  |  LEN |  CRC  |    DPKG   |    DPKG   |   ....  |   CRC   |           *
*  ,------+------+------+-------+-----------+-----------+---------+---------+           *
*  | size |  2   |   1  |   1   |    ...    |    ...    |   ...   |    2    |           *
*  '----- +------+------+-------+- - - - - -+- - - - - -+- - - - -+- - - - -+           *
*  SOF  .........  start of frame, 2 bytes                                              *
*  LEN  .........  number of data package in the frame                                  *
*  CRC  .........  Head of Frame's cyclic redundancy check                              *
*  DPKG .........  data package,include DATA_TAG,DATA                                   *
*  CRC  .........  DATA cyclic redundancy check                                         *   
*                                                                                       *
*                                                                                       *
*  ,------+--------------+---------+---------+--------+                                 *
*  | Type |   element    |  size   |  offset |  byte  |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  |      | frame  head1 | 1 BYTE  |    0    |    1   |                                 *
*  | SOF  ,--------------+---------+---------+--------+                                 *
*  |      | frame  head2 | 1 BYTE  |    1    |    2   |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  | LEN  |              | 1 BYTE  |    2    |    3   |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  | CRC  |              | 1 BYTES |    3    |    5   |                                 *
*  ,------,--------------+---------+---------+--------+                                 *
*  |      |   DATA_TAG   | 1 BYTE  |    5    |    6   |                                 *
*  | DPKG ,--------------+---------+---------+--------+                                 *
*  |      |   DATA       | n BYTES |   5+n   |   6+n  |                                 *
*  ,---------------------+---------+---------+--------+                                 *
*  |      |   DATA_TAG   | 1 BYTE  |   6+n   |   7+n  |                                 *
*  | DPKG ,--------------+---------+---------+--------+                                 *
*  |      |   DATA       | m BYTES |  6+m+n  |  7+m+n |                                 *
*  '------,--------------+---------+---------+--------+                                 *
*  | CRC  |              | 1 BYTES |    3    |    5   |                                 *
*  '------'--------------+---------+---------+--------+                                 *  
*                                                                                       *
*  Endian: Little-Endian                                                                *
*****************************************************************************************/

#include <cstdint>
#include <vector>
#include <cstring>
#include <functional>

// #define imu_mag      //IMU??????????????????


/**
 * @brief ?????????????????????????????????
 */
#define MAX_RxBUFF_SIZE    510

/**
 * @brief 
 */
     
#define FRAME_HEAD_CRC_BYTES        1     
#define FRAME_DPKG_CRC_BYTES        2
#define FRAME_DPKG_LEN_BYTES        1     
#define FRAME_INFO_SIZE             (2+FRAME_DPKG_LEN_BYTES+FRAME_HEAD_CRC_BYTES)
#define FRAME_CALCU_CRC_BYTES       (2 + FRAME_DPKG_LEN_BYTES)
/**
 * @brief ????????????????????????
 */
#define MAX_BUFF_SIZE      255                                //255
#define MAX_DPKG_SIZE      (MAX_BUFF_SIZE-FRAME_INFO_SIZE)    //251


/**
 * @brief ???Byte?????????
 * 
 */
#define FRAME_HEAD_OFFSET               2
#define FRAME_DPKG_LEN_OFFSET           (FRAME_HEAD_OFFSET + FRAME_DPKG_LEN_BYTES)
#define FRAME_HEAD_CRC_OFFSET           FRAME_INFO_SIZE




/**
 * @brief ??????
 */
#define Header1                  0xFE
#define Header2                  0xEF

/**
 * @brief ???????????????
 */
#define robot_tag                0xA0
#define speed_tag                0xB0
#define encoder_tag              0xC0
#define imu_tag                  0xD0
#define imu_sensor_tag           0xD1
#define imu_raw_tag              0xD2
#define voltage_tag              0xE0  
#define pid_tag                  0x70

/**
 * @brief ???????????????
 */
#define speed_dpkg_len  (uint8_t)0x05
#define pid_dpkg_len    (uint8_t)0x07


/***********???????????????????????????***********/

/**
 * @brief status of robot 
 */
typedef struct Robot_State_Info_msgs
{
    float      voltage;                 //real voltage = voltage/10
    int16_t    l_encoder_pulse;
    int16_t    r_encoder_pulse;
    float      acc[3];
    float      gyr[3];
#ifdef     imu_mag
    float      mag[3];
#endif
    float      elu[3];
}Robot_msgs;

/**
 * @brief 
 */
typedef struct receiveStream
{
    uint8_t        len;
    uint32_t       crc;

    uint8_t        stream_buff[MAX_BUFF_SIZE];
}Stream_msgs;

//----------------------------------------






/**
 * @brief  ??????????????????N bytes???????????????????????????????????????
 * 
 * @tparam T  ????????????
 * @tparam N  ??????????????????
 */
template <typename T,std::size_t N>
union BytesConv
{
    uint8_t bytes[N];
    T       number;
};

/**
 * @brief  ???N bytes?????????????????????????????????
 * 
 * @tparam T ????????????
 * @tparam N ??????????????????
 * @param p  ??????????????????
 * @return T ???????????????????????????????????????????????????
 */
template <typename T,std::size_t N>
inline T Bytes2Num(const uint8_t* p)
{
    BytesConv<T,N> conv;
    for(int i=0;i<N;++i)
       conv.bytes[i]=p[i];
    return conv.number;
}

/*  ????????????  */

std::function <float(const uint8_t*)> Byte2Float = Bytes2Num<float,4>;
std::function <int16_t(const uint8_t*)> Byte2INT16 = Bytes2Num<int16_t,2>;
std::function <uint16_t(const uint8_t*)> Byte2U16 = Bytes2Num<uint16_t,2>;



/**
 * @brief ??????????????????????????????
 * 
 * @tparam T   ???????????????
 * @param[in]  T_struct  ???????????????
 * @return std::vector<uint8_t> 
 */
template <typename T>
inline std::vector<uint8_t> structPack_Bytes(T &T_struct,int size=sizeof(T))
{
    std::vector<uint8_t> Bytes(size);
    memcpy(Bytes.data(),&T_struct,size);
    return Bytes;
}

/**
 * @brief 
 * 
 * @tparam T 
 * @param buff 
 * @param T_struct 
 * @param size 
 * @return uint8_t* 
 */
template <typename T>
inline uint8_t* structPack_Bytes(uint8_t* buff,T &T_struct,int size=sizeof(T))
{
    memcpy(buff,&T_struct,size);
    return buff;
}

/**
 * @brief Set the Buff CRC Value 
 * 
 * @param buff 
 * @param offset 
 * @param value 
 */
inline void setBuffCRCValue(std::vector<uint8_t> &buff,uint8_t offset,uint8_t value)
{
    buff[offset] = value;
}

/**
 * @brief Set the Buff CRC Value 
 * 
 * @param buff 
 * @param offset 
 * @param value 
 */
inline void setBuffCRCValue(uint8_t* buff,uint8_t offset,uint8_t value)
{
    buff[offset] = value;
}

/**
 * @brief Set the Buff CRC Value
 * 
 * @tparam T 
 * @tparam N 
 * @param buff 
 * @param offset 
 * @param value 
 */
template <typename T,std::size_t N>
inline void setBuffCRCValue(uint8_t* buff,uint8_t offset,T value)
{
    BytesConv<T,N> conv;
    conv.number = value;
    for(int i=0;i<N;++i)
        buff[offset+i] = conv.bytes[i];
}

/**
 * @brief Set the Buff C R C Value object
 * 
 * @tparam T 
 * @tparam N 
 * @param buff 
 * @param offset 
 * @param value 
 */
template <typename T,std::size_t N>
inline void setBuffCRCValue(std::vector<uint8_t> &buff,uint8_t offset,T value)
{
    BytesConv<T,N> conv;
    conv.number = value;
    for(int i=0;i<N;++i)
        buff[offset+i] = conv.bytes[i];
}




/***********???????????????????????????***********/

#pragma pack(1) //1 ????????????

/**
 * @brief ?????????
 */
typedef struct 
{
    uint8_t      header[2];
    uint8_t      len;
    uint8_t      crc;
}Frame_Info;


/**
 * @brief ???????????????
 */
typedef struct 
{
    int8_t  data_tag;
    int16_t velocity;             //real velocity = velocity/1000
    int16_t angular;                  //real yaw = yaw/1000      -3141<=yaw<=3141                 1 deg = 0.017453292 rad
}Speed_dpkg;
/**
 * @brief ???????????????
 */
typedef struct 
{
    uint8_t      header[2];
    uint8_t      len;
    uint8_t      crc_head;
    Speed_dpkg   speed;
    uint16_t     crc_dpkg;
}Frame_Speed_dpkg;

/**
 * @brief PID?????????
 */
typedef struct 
{
    int8_t      data_tag;
    int16_t     P;
    int16_t     I;            
    int16_t     D;                 
}PID_dpkg;

/**
 * @brief ??????PID???
 */
typedef struct 
{
    uint8_t      header[2];
    uint8_t      len;
    uint8_t      crc_head;
    PID_dpkg     pid;
    uint16_t     crc_dpkg;
}Frame_PID_dpkg;

#pragma pack()  //??????????????????

#endif

