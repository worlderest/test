#include "imu_process.h"
#include <math.h>
//#include "Calibration.h"
// ==========================================
// 模块私有结构体 (完全对外部隐藏)
// ==========================================
#pragma pack(1)
typedef struct {
    uint8_t  header;    
    uint8_t  node_id;   
    uint16_t seq_num;   
    int16_t  acc_x;     
    int16_t  acc_y;
    int16_t  acc_z;
    int16_t  gyro_x;    
    int16_t  gyro_y;
    int16_t  gyro_z;
} ImuPayload_t;
#pragma pack()

#define Kp 2.0f     // 比例增益：信任加速度计的程度 (如果画面抖，可以改小)
#define Ki 0.005f   // 积分增益：消除陀螺仪零偏漂移
#define halfT 0.01f // 你是 50Hz(20ms)，所以周期一半是 0.01 秒

// 核心姿态：四元数 (初始化为绝对平放，q0=1，其余为0)
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f; // 误差积分
// ==========================================
// 模块私有变量 (加上 static，极其安全)
// ==========================================
static float phy_acc[3] = {0.0f, 0.0f, 0.0f};   // 单位: g
static float phy_gyro[3] = {0.0f, 0.0f, 0.0f};  // 单位: dps

static inline float invSqrt(float x) {
    return 1.0f / sqrtf(x); // 编译器会自动翻译为 VSQRT.F32 硬件指令
}

static void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // 将陀螺仪的 度/秒(dps) 转换为 弧度/秒(rad/s)
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // 如果加速度计有有效数据
    if(ax != 0.0f || ay != 0.0f || az != 0.0f) {
        // 归一化加速度计测量值
        norm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= norm;  ay *= norm;  az *= norm;

        // 提取当前四元数推算出的重力方向
        vx = 2.0f * (q1 * q3 - q0 * q2);
        vy = 2.0f * (q0 * q1 + q2 * q3);
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // 计算误差：测量重力方向与推算重力方向的叉积
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // 误差积分
        if(Ki > 0.0f) {
            exInt += ex * Ki;  eyInt += ey * Ki;  ezInt += ez * Ki;
        } else {
            exInt = 0.0f;  eyInt = 0.0f;  ezInt = 0.0f;
        }

        // PI 修正陀螺仪角速度
        gx += Kp * ex + exInt;
        gy += Kp * ey + eyInt;
        gz += Kp * ez + ezInt;
    }

    // 整合四元数率并积分
    gx *= halfT;  gy *= halfT;  gz *= halfT;
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;  q1 *= norm;  q2 *= norm;  q3 *= norm;
}
// ==========================================
// 核心逻辑实现
// ==========================================
void IMU_Parse_Data(uint8_t *raw_payload)
{
	
    // 指针强转魔法
    ImuPayload_t *p_imu = (ImuPayload_t *)raw_payload;

    // 二次防御校验
    if(p_imu->header != 0xAA) return;

    // 加速度系数：0.122 mg/LSB -> g
    phy_acc[0] = p_imu->acc_x * 0.122f / 1000.0f;
    phy_acc[1] = p_imu->acc_y * 0.122f / 1000.0f;
    phy_acc[2] = p_imu->acc_z * 0.122f / 1000.0f;

    // 陀螺仪系数：70.0 mdps/LSB -> dps
    phy_gyro[0] = p_imu->gyro_x * 70.0f / 1000.0f;
    phy_gyro[1] = p_imu->gyro_y * 70.0f / 1000.0f;
    phy_gyro[2] = p_imu->gyro_z * 70.0f / 1000.0f;
	
	

    float ax = phy_acc[0];
    float ay = phy_acc[1];
    float az = phy_acc[2];

    float gx = phy_gyro[0];
    float gy = phy_gyro[1];
    float gz = phy_gyro[2];
    MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
}

void IMU_Get_Quaternion(float *q) {
    q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;
}

void IMU_Get_EulerAngles(float *pitch, float *roll, float *yaw) {
    *pitch = asinf(-2.0f * (q1 * q3 - q0 * q2)) * 57.29578f; 
    *roll  = atan2f(2.0f * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * 57.29578f;
    *yaw   = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.29578f;
}
// ==========================================
// 提供给外部的读取接口 (Getters)
// ==========================================
float* IMU_Get_Acc(void)  { return phy_acc; }
float* IMU_Get_Gyro(void) { return phy_gyro; }