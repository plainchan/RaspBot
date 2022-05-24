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

#define imu_mag      //IMU是否带磁力计


/**
 * @brief 串口传输缓冲区最大大小
 */
#define MAX_RxBUFF_SIZE    510

/**
 * @brief 
 */
     
#define FRAME_HEAD_CRC_BYTES        1     
#define FRAME_DPKG_CRC_BYTES        2
#define FRAME_DPKG_LEN_BYTES        1     
#define FRAME_INFO_SIZE             (2+FRAME_DPKG_LEN_BYTES+FRAME_HEAD_CRC_BYTES)

/**
 * @brief 帧缓冲区最大大小
 */
#define MAX_BUFF_SIZE      255                                //255
#define MAX_DPKG_SIZE      (MAX_BUFF_SIZE-FRAME_INFO_SIZE)    //251


/**
 * @brief 帧Byte偏移量
 * 
 */
#define FRAME_HEAD_OFFSET               2
#define FRAME_DPKG_LEN_OFFSET           (FRAME_HEAD_OFFSET + FRAME_DPKG_LEN_BYTES)
#define FRAME_HEAD_CRC_OFFSET           FRAME_INFO_SIZE




/**
 * @brief 帧头
 */
#define Header1                  0xFE
#define Header2                  0xEF

/**
 * @brief 数据包标签
 */
#define robot_tag                0xA0
#define speed_tag                0xB0
#define encoder_tag              0xC0
#define imu_tag                  0xD0
#define imu_sensor_tag           0xD1
#define voltage_tag              0xE0  

/**
 * @brief 数据包长度
 */
#define speed_dpkg_len  (uint8_t)0x05


/***********数据解包结构体定义***********/

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
 * @brief  共享内存，将N bytes内存数据转换成特定数据
 * 
 * @tparam T  数据类型
 * @tparam N  数据类型大小
 */
template <typename T,std::size_t N>
union BytesConv
{
    uint8_t bytes[N];
    T       number;
};
/**
 * @brief  对共用体赋值
 * 
 * @tparam T 数据类型
 * @tparam N 数据类型大小
 * @param p  内存数据地址
 * @return T 返回内存数据转换的制定数据类型的值
 */
template <typename T,std::size_t N>
T Bytes2Num(const uint8_t* p)
{
    BytesConv<T,N> conv;
    for(int i=0;i<N;++i)
       conv.bytes[i]=p[i];
    return conv.number;
}
/**
 * @brief 将结构体转换成字节流
 * 
 * @tparam T   结构体类型
 * @param[in]  T_struct  结构体变量
 * @return std::vector<uint8_t> 
 */
template <typename T>
std::vector<uint8_t> structPack_Bytes(T &T_struct)
{
    int size = sizeof(T_struct);
    std::vector<uint8_t> Bytes(size);
    memcpy(Bytes.data(),&T_struct,size);
    return Bytes;
}

/**
 * @brief 将结构体转换成字节流
 * 
 * @tparam T   结构体类型
 * @param[in]  T_struct  结构体变量
 * @return std::vector<uint8_t> 
 */
template <typename T>
std::vector<uint8_t> structPack_Bytes(T &T_struct,int size)
{
    std::vector<uint8_t> Bytes(size);
    memcpy(Bytes.data(),&T_struct,size);
    return Bytes;
}





/***********数据封包结构体定义***********/

#pragma pack(1) //1 字节对齐

/**
 * @brief 帧信息
 */
typedef struct 
{
    uint8_t      header[2];
    uint8_t      len;
    uint8_t      crc;
}Frame_Info;


/**
 * @brief 速度数据包
 */
typedef struct 
{
    int8_t  data_tag;
    int16_t velocity;             //real velocity = velocity/1000
    int16_t yaw;                  //real yaw = yaw/1000      -3141<=yaw<=3141                 1 deg = 0.017453292 rad
}Speed_dpkg;
/**
 * @brief 封装速度帧
 */
typedef struct 
{
    uint8_t      header[2];
    uint8_t      len;
    uint8_t      crc_head;
    Speed_dpkg   speed;
    uint8_t      crc_dpkg;
}Frame_Speed_dpkg;

#pragma pack()  //结束字节对齐

#endif

