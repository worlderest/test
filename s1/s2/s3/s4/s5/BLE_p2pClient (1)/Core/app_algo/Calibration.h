#ifndef __CALIBRATION_H
#define __CALIBRATION_H

void gyro_Calibration(float *gyro_x,float *gyro_y,float *gyro_z);
void gyro_DeadZone_execute(float *gyro_x,float *gyro_y,float *gyro_z);

#endif