#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "Calibration.h"
#include "log_module.h"

#define ODR 50
#define CALI_SAMPLES  200
#define CALI_EFFECTIVE_SAMPLES 150
#define CALI_NOISE_THRESHOLD 1.0f

#define Suggested_Calibration_X -0.01590f
#define Suggested_Calibration_Y -0.28385f
#define Suggested_Calibration_Z -0.38815f

//float gyro_x_bias,gyro_y_bias,gyro_z_bias=0;
static uint8_t is_calibrated=0;
static uint16_t cali_count=0;
static uint16_t cali_effc_count=0;
float gyro_sum[3]={0};
float gyro_bias[3]={Suggested_Calibration_X,Suggested_Calibration_Y,Suggested_Calibration_Z};

// get bias while initializing
void gyro_Calibration(float *gyro_x,float *gyro_y,float *gyro_z)
{
    if(is_calibrated == 0)   
    {
        if(fabsf(*gyro_x) < CALI_NOISE_THRESHOLD && fabsf(*gyro_y) < CALI_NOISE_THRESHOLD && fabsf(*gyro_z) < CALI_NOISE_THRESHOLD)
        {
            gyro_sum[0]+=*gyro_x;
            gyro_sum[1]+=*gyro_y;
            gyro_sum[2]+=*gyro_z;
            cali_effc_count++;
        }
		*gyro_x-=gyro_bias[0];
		*gyro_y-=gyro_bias[1];
		*gyro_z-=gyro_bias[2];
        cali_count++;
        if (cali_count >= CALI_SAMPLES)
        {
            if (cali_effc_count >= CALI_EFFECTIVE_SAMPLES)
            {
                gyro_bias[0] = gyro_sum[0] / (float)cali_effc_count;
                gyro_bias[1] = gyro_sum[1] / (float)cali_effc_count;
                gyro_bias[2] = gyro_sum[2] / (float)cali_effc_count;
                is_calibrated = 1;
				
				LOG_INFO_APP("this is the suggested calibration\r\n");
				LOG_INFO_APP("CALI Acc X: %.5f g\r\n", gyro_bias[0]);
				LOG_INFO_APP("CALI Acc Y: %.5f g\r\n", gyro_bias[1]);
				LOG_INFO_APP("CALI Acc Z: %.5f g\r\n", gyro_bias[2]);
            }
            else
            {
                cali_count = 0;
                cali_effc_count = 0;
                gyro_sum[0] = 0.0f;
                gyro_sum[1] = 0.0f;
                gyro_sum[2] = 0.0f;
            }
        }
    }else if(is_calibrated == 1){
        *gyro_x-=gyro_bias[0];
        *gyro_y-=gyro_bias[1];
        *gyro_z-=gyro_bias[2];
    }
}