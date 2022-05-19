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

// #define imu_mag      //IMU是否带磁力计


/**
 * @brief 串口传输缓冲区最大大小
 */
#define MAX_RxBUFF_SIZE   0x01FE                   //510

/**
 * @brief 帧缓冲区最大大小
 */
#define MAX_BUFF_SIZE      0xFF                             //255
#define FRAME_INFO_SIZE    0x05                             //5  非数据域
#define MAX_DPKG_SIZE      (MAX_BUFF_SIZE-FRAME_INFO_SIZE)    //250

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
#define imu_6axis_tag            0xD6
#define imu_9axis_tag            0xD9
#define voltage_tag              0xE0  

/**
 * @brief 数据包长度
 */
<<<<<<< HEAD
#define speed_dpkg_len   (uint8_t)0x05
=======
#define speed_dpkg_len  (uint8_t)0x05
>>>>>>> 80bc1b28c8a9bc65b9213c10c96c11168d8458ae


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
    uint16_t       crc;

    uint8_t        stream_buff[MAX_BUFF_SIZE];
    Robot_msgs     robot_msgs;
<<<<<<< HEAD
=======
    Encoder_msg    encoder_msg;
    IMU_msg        imu_msg;
    IMU_6Axis_msg  imu_6axis_msg;
    IMU_9Axis_msg  imu_9axis_msg;
>>>>>>> 80bc1b28c8a9bc65b9213c10c96c11168d8458ae
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





/***********数据封包结构体定义***********/

#pragma pack(1) //1 字节对齐

/**
 * @brief 帧信息
 */
typedef struct 
{
    uint8_t      header[2];
    uint8_t      len;
    uint16_t     crc;
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
    uint16_t     crc;

    Speed_dpkg   speed;
}Frame_Speed_dpkg;

#pragma pack()  //结束字节对齐

#endif

