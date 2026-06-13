#ifndef __COM_FILTER_H
#define __COM_FILTER_H

#include "stdint.h"
#include "math.h"

typedef struct
{
    float LastP;   // 上一时刻的状态方差（或协方差）  状态方差：对于预测状态的确定度，P越小越相信预测结果
    float Now_P;   // 当前时刻的状态方差（或协方差）
    float out;     // 滤波器的输出值，即估计的状态
    float Kg;      // 卡尔曼增益，用于调节预测值和测量值之间的权重
    float Q;       // 过程噪声的方差，反映系统模型的不确定性
    float R;       // 测量噪声的方差，反映测量过程的不确定性
} KalmanFilter_Struct;

extern KalmanFilter_Struct kfs[3];


// 4维卡尔曼：纬度、经度、速度、航向,车载GPS
typedef struct {
    float X[4];     // 状态 [lat, lon, v, heading]
    float P[4][4];  // 协方差
    float Q[4][4];  // 过程噪声
    float R[4][4];  // 观测噪声
} Kalman_GPS_t;

extern Kalman_GPS_t kf;







int16_t Com_Filter_LowPass(int16_t newData, int16_t lastData);
double  Common_Filter_KalmanFilter(KalmanFilter_Struct *kf, double input);

void KalmanGPS_Update(float lat, float lon, float v, float heading, float dt);

#endif

