#include "com_filter.h"

#define ALPHA 0.9

/*一节低通滤波，互补*/
int16_t Com_Filter_LowPass(int16_t newData, int16_t lastData)  
{
    return ALPHA * lastData + (1 - ALPHA) * newData;
}


KalmanFilter_Struct kfs[3] = {
    {0.02, 0, 0, 0, 0.001, 0.543},
    {0.02, 0, 0, 0, 0.001, 0.543},
    {0.02, 0, 0, 0, 0.001, 0.543}};
double Common_Filter_KalmanFilter(KalmanFilter_Struct *kf, double input)   //此处为1维简单卡尔曼，所以H为1，R和Q为超参数，需要手动调节配置
{
    kf->Now_P = kf->LastP + kf->Q;   //预测计算，用于套用不同情况
	
    kf->Kg    = kf->Now_P / (kf->Now_P + kf->R);
    kf->out   = kf->out + kf->Kg * (input - kf->out);
    kf->LastP = (1 - kf->Kg) * kf->Now_P;
    return kf->out;
}


Kalman_GPS_t kf;

/*初始化GPS滤波参数*/
void KalmanGPS_Init(void)
{
    // 初始状态
    kf.X[0] = 0.0f;   // lat
    kf.X[1] = 0.0f;   // lon
    kf.X[2] = 0.0f;   // speed
    kf.X[3] = 0.0f;   // heading

    // 初始协方差
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf.P[i][j] = 0;
        }
    }
    kf.P[0][0] = 10;    //初始置信度   lat：5~25
    kf.P[1][1] = 10;     //lon:5~25
    kf.P[2][2] = 1;      //speed:0.1~2
    kf.P[3][3] = 5;      //heading:2~10

    // 过程噪声（小车运动平滑）
    kf.Q[0][0] = 0.1f;
    kf.Q[1][1] = 0.1f;
    kf.Q[2][2] = 0.2f;
    kf.Q[3][3] = 1.0f;

    // 观测噪声（GPS噪声大）
    kf.R[0][0] = 5.0f;
    kf.R[1][1] = 5.0f;
    kf.R[2][2] = 2.0f;
    kf.R[3][3] = 8.0f;
}

/*卡尔曼四维滤波*/
void KalmanGPS_Update(float lat, float lon, float v, float heading, float dt)
{
	KalmanGPS_Init();
    // 预测    X(k|k-1) = AX(k-1|k-1) + BU(k)  A为单位矩阵
    float rad = heading * 3.1415926 / 180.0f;
    float dx = v * dt * sin(rad) / 111320.0f;   // 经度变化  1纬度为111320米
    float dy = v * dt * cos(rad) / 111320.0f;   // 纬度变化

    float pre_lat = kf.X[0] + dy;
    float pre_lon = kf.X[1] + dx;
    float pre_v   = kf.X[2];
    float pre_h   = kf.X[3];

    // 预测协方差  P(k|k-1) = AP(k-1|k-1)A' + Q  A为单位矩阵
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) 
		{
            kf.P[i][j] += kf.Q[i][j];
        }
    }

    // 卡尔曼增益  Kg(k) = P(k|k-1)H' / (HP(k|k-1)H' + R)  H为单位矩阵
    float K[4];
    for (int i = 0; i < 4; i++) 
	{
        K[i] = kf.P[i][i] / (kf.P[i][i] + kf.R[i][i]);
    }

    // 更新
    kf.X[0] = pre_lat + K[0] * (lat - pre_lat);
    kf.X[1] = pre_lon + K[1] * (lon - pre_lon);
    kf.X[2] = pre_v   + K[2] * (v - pre_v);
    kf.X[3] = pre_h   + K[3] * (heading - pre_h);

    // 修正协方差
    for (int i = 0; i < 4; i++) {
        kf.P[i][i] = (1 - K[i]) * kf.P[i][i];
    }
}



