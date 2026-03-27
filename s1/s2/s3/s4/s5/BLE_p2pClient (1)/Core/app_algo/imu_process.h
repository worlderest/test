#ifndef IMU_PROCESS_H
#define IMU_PROCESS_H

#include <stdint.h>

// 1. 核心解析函数：供蓝牙模块在收到 16 字节数据后调用
void IMU_Parse_Data(uint8_t *raw_payload);

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

void IMU_Get_Quaternion(float *q);

void IMU_Get_EulerAngles(float *pitch, float *roll, float *yaw);
// 2. 数据获取接口：供 main 函数或串口发送模块随时获取算好的物理量
float* IMU_Get_Acc(void);
float* IMU_Get_Gyro(void);

#endif /* IMU_PROCESS_H */