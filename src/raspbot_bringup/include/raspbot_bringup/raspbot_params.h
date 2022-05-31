#ifndef  __RASPBOT_PARAMS_H__
#define  __RASPBOT_PARAMS_H__



#define reduction_Ratio                  30       //电机减速比
#define encoder_line                     13       //
#define multiplier_factor                4        // A/B相倍频因子
#define PPR                   (reduction_Ratio*encoder_line)

#define wheelTrack               (float)0.1       //轮距
#define wheelRadius                    0.02       //轮胎半径

#define intervalTimer             0.01            //单片机定时器中断











#endif