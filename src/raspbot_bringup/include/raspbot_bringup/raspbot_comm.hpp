#ifndef _RASPBOT_COMM_H_
#define _RASPBOT_COMM_H_

#include <cstdint>
#include <vector>
#include <cstring>

#define Header1                  0xFF
#define Header2                  0xEE
#define speed_tag                0xA0
#define frame_speed_len          0x04


union float_byte_convert
{
    float  f_data;
    char   byte[4];
};

union endian
{
    int  a;
    char b;
};

#pragma pack(1) //1 字节对齐
struct Data_msgs
{
    int8_t  header1;
    int8_t  header2;
    int8_t  data_tag;
    int8_t  length;

    uint8_t  voltage;                 //real voltage = voltage/10
    uint16_t l_encoder_pulse;
    uint16_t r_encoder_pulse;
    float acc[3];
    float gyr[3];
    float mag[3];
    float elu[3];
};


struct Speed_msgs
{
    int8_t  header1;
    int8_t  header2;
    int8_t  data_tag;
    int8_t  length;

    int16_t velocity;             //real velocity = velocity/1000
    int16_t yaw;                  //real yaw = yaw/1000      -3141<=yaw<=3141                 1 deg = 0.017453292 rad
};
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