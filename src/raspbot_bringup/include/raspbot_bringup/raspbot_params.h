#ifndef  __RASPBOT_PARAMS_H__
#define  __RASPBOT_PARAMS_H__



#define reduction_Ratio                  30       // 电机减速比
#define encoder_line                     13       // 编码器线数
#define multiplier_factor                4        // A/B相倍频因子
#define PPR                   (reduction_Ratio*encoder_line*multiplier_factor)

#define wheelTrack                0.203f       //轮距
#define wheelRadius               0.0325f       //轮胎半径

#define intervalTimer             0.01            //单片机定时器中断，数据采集间隔

#define deg_to_rad               0.017453292

/* imu  */

#define  accRatio           (double)2*16/65536                  // ±2 ±4 ±8 ±16
#define  gyrRatio           (double)deg_to_rad*2*2000/65536     // ±250 ±500 ±1000 ±2000
#define  magRatio           (double)2*4800/16384                // ±4800
#define  eluRatio           (double)deg_to_rad*2*180/65536      // ±180



#endif