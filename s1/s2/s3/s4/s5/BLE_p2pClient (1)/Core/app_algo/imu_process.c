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

#define Kp 2.0f//2.0f     // 比例增益：信任加速度计的程度 (如果画面抖，可以改小)
#define Ki 0.005f//0.005f   // 积分增益：消除陀螺仪零偏漂移
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

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
	
	gx*=0.017453f;
	gy*=0.017453f;
	gz*=0.017453f;
    // 1. 加速度计向量归一化 (只有方向有用)
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // 避免除零
    ax /= norm;
    ay /= norm;
    az /= norm;

    // 2. 提取当前四元数所代表的“理论重力方向” (从地理坐标系转到机体坐标系)
    // 这是四元数旋转矩阵的第三列
    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 3. 计算误差 (测量到的重力方向 与 预测方向 的叉积)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // 4. 误差积分 (用于消除陀螺仪漂移)
    if (Ki > 0.0f) {
        exInt += ex * Ki;
        eyInt += ey * Ki;
        ezInt += ez * Ki;
    } else {
        exInt = 0.0f; eyInt = 0.0f; ezInt = 0.0f;
    }

    // 5. 反馈修正：调整角速度
    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;

    // 6. 一阶龙格-库塔法更新四元数微分方程
    // q_new = q_old + 0.5 * dt * q_old * omega
    float q0_last = q0, q1_last = q1, q2_last = q2, q3_last = q3;
    q0 += (-q1_last * gx - q2_last * gy - q3_last * gz) * halfT;
    q1 += ( q0_last * gx + q2_last * gz - q3_last * gy) * halfT;
    q2 += ( q0_last * gy - q1_last * gz + q3_last * gx) * halfT;
    q3 += ( q0_last * gz + q1_last * gy - q2_last * gx) * halfT;

    // 7. 四元数归一化 (防止积分过程中由于精度舍入失去单位特性)
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
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

}

void IMU_Get_Quaternion(float *q) {
    q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;
}

void IMU_Get_EulerAngles(float *pitch, float *roll, float *yaw) {
    // 1. 计算常用的平方项，节约重复计算的算力
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    // 2. Roll (横滚角) - 弧度转角度
    // 公式：atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
    *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), q0q0 - q1q1 - q2q2 + q3q3) * 57.29578f;

    // 3. Pitch (俯仰角) - 弧度转角度
    // 公式：asin(2*(q0*q2 - q1*q3))
    // 注意：asin 的输入必须在 [-1, 1] 之间，否则会报错 NaN
    float sinp = 2.0f * (q0 * q2 - q1 * q3);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(1.570796f, sinp) * 57.29578f; // 限制在 ±90度
    else
        *pitch = asinf(sinp) * 57.29578f;

    // 4. Yaw (偏航角) - 弧度转角度
    // 公式：atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
    // 注意：没有磁力计的情况下，Yaw 会随时间缓慢漂移，仅供参考
    *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), q0q0 + q1q1 - q2q2 - q3q3) * 57.29578f;
}
// ==========================================
// 提供给外部的读取接口 (Getters)
// ==========================================
float* IMU_Get_Acc(void)  { return phy_acc; }
float* IMU_Get_Gyro(void) { return phy_gyro; }