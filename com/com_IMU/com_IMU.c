#include "com_IMU.h"


/* 计算欧拉角用到的3个参数 */
float RtA = 57.2957795f;   // 弧度->度
// 陀螺仪初始化量程+-2000度/秒于 1/(65536 / 4000) = 0.03051756*2
// float Gyro_G = 0.03051756f * 2;
float Gyro_G = 4000.0 / 65536;   // 度/s
// 度每秒,转换弧度每秒则 2*0.03051756 * 0.0174533f = 0.0005326*2
// float Gyro_Gr = 0.0005326f * 2;
float Gyro_Gr = 4000.0 / 65536 / 180 * 3.1415926;   // 弧度/s

/* 
度/秒 = 原始值 * 量程 / 32768   
弧度 = 度 × π / 180
弧度=原始值 * 量程 / 32768 /180 *π


*/

#define squa(Sq) (((float)Sq) * ((float)Sq))        /* 计算平方 */
/**
 * @description: 快速计算 1/sqrt(num)
 * @param {float} number
 */
static float Q_rsqrt(float number)
{
    long        i;
    float       x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = *(long *)&y;
    i  = 0x5f3759df - (i >> 1);
    y  = *(float *)&i;
    y  = y * (threehalfs - (x2 * y * y));   // 1st iteration （第一次牛顿迭代）
    return y;
}

static double normAccz; /* z轴上的加速度 */
/**
 * @description: 根据mpu的6轴数据, 获取表征姿态的欧拉角，适用于飞行器
 * @param {GyroAccel_Struct} *gyroAccel mpu的6轴数据
 * @param {EulerAngle_Struct} *EulerAngle 计算后得到的欧拉角
 * @param {float} dt 采样周期 (单位s)
 * @return {*}
 */
void Common_IMU_GetEulerAngle(GyroAccel_Struct  *gyroAccel,
                              EulerAngle_Struct *eulerAngle,
                              float              dt)
{
    volatile struct V
    {
        float x;
        float y;
        float z;
    } Gravity, Acc, Gyro, AccGravity;

    static struct V          GyroIntegError = {0};
    static float             KpDef          = 0.8f;      //kp和ki都为超调量，需要自己调配，用来修正误差。kp为比例修正，用于快速修正当前误差
    static float             KiDef          = 0.0003f;    //ki为积分修正，用于缓慢修正慢漂移
    static Quaternion_Struct NumQ           = {1, 0, 0, 0};  //设定四元数初始值，水平姿态，没有倾斜。q0旋转角度 q1/q2/q3 旋转轴x/y/z的分量
    float                    q0_t, q1_t, q2_t, q3_t;
    // float NormAcc;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    // 提取等效旋转矩阵中的重力分量
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);   //根据当前估计姿态，推算出重力的朝向(z轴）,以及x、y的加速度方向
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);  
    // 加速度归一化
    NormQuat = Q_rsqrt(squa(gyroAccel->accel.accelX) +
                       squa(gyroAccel->accel.accelY) +
                       squa(gyroAccel->accel.accelZ));

    Acc.x = gyroAccel->accel.accelX * NormQuat;     //根据传感器测量值，计算当前实际重力的朝向，以及x、y的加速度方向
    Acc.y = gyroAccel->accel.accelY * NormQuat;
    Acc.z = gyroAccel->accel.accelZ * NormQuat;
    // 向量差乘得出的值
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);   //通过向量计算，得到x、y、z轴的偏移量即sinθ≈θ
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
    // 再做加速度积分补偿角速度的补偿值
    GyroIntegError.x += AccGravity.x * KiDef;     //ki用于积分修正
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;
    // 角速度融合加速度积分补偿值
    Gyro.x = gyroAccel->gyro.gyroX * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x;   // 弧度制，将当前传感器测得的角度转化为弧度，并通过PI减小误差
    Gyro.y = gyroAccel->gyro.gyroY * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = gyroAccel->gyro.gyroZ * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;

    // 一阶龙格库塔法, 更新四元数
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;   //更新四元数
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;

    // 四元数归一化
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));   //将四元数化为向量方向
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    /*机体坐标系下的Z方向向量，得到物体实际Z轴在，现实坐标轴xyz中的方向*/
    float vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3;     /*矩阵(3,1)项*/
    float vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;     /*矩阵(3,2)项*/
    float veczZ = 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2; /*矩阵(3,3)项*/

    float yaw_G = gyroAccel->gyro.gyroZ * Gyro_G;   // 将Z轴角速度陀螺仪值 转换为Z角度/秒      Gyro_G陀螺仪初始化量程+-2000度每秒于1 / (65536 / 4000) = 0.03051756*2
    if((yaw_G > 0.5f) || (yaw_G < -0.5))            // 数据太小可以认为是干扰，不是偏航动作
    {
        eulerAngle->yaw += yaw_G * dt;   // 角速度积分成偏航角
    }

    eulerAngle->pitch = asin(vecxZ) * RtA;   // 俯仰角

    eulerAngle->roll = atan2f(vecyZ, veczZ) * RtA;   // 横滚角

    normAccz = gyroAccel->accel.accelX * vecxZ + gyroAccel->accel.accelY * vecyZ + gyroAccel->accel.accelZ * veczZ; /*Z轴垂直方向上的加速度，此值涵盖了倾斜时在Z轴角速度的向量和，不是单纯重力感应得出的值*/
}



/**
 * @description: 根据mpu的6轴数据, 获取表征姿态的车头转向角，适用于四轮车
 * @param {GyroAccel_Struct} *gyroAccel mpu的6轴数据
 * @param {EulerAngle_Struct} *EulerAngle 计算后得到的欧拉角
 * @param {float} dt 采样周期 (单位s)
 * @return {*}
 */
void Common_IMU_GetYawOnly(GyroAccel_Struct  *gyroAccel,
                           EulerAngle_Struct *eulerAngle,
                           float              dt)
{
    // 仅计算偏航角 YAW（四轮车唯一需要的角度）
    static float yaw = 0;

    // 陀螺仪 Z 轴原始值 → 弧度/秒
    float gyro_z = gyroAccel->gyro.gyroZ * Gyro_G;

    // 死区：过滤微小噪声（防漂移）
    if (fabs(gyro_z) > 0.5f)
    {
        yaw += gyro_z * dt;
    }

    eulerAngle->yaw = yaw;  // 输出车头转向角度
}

