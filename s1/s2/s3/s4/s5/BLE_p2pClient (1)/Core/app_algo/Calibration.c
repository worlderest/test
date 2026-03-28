#include <stdint.h>
#include <math.h>
#include "Calibration.h"
#include "log_module.h"

#define ODR 50
#define CALI_SAMPLES  200
//#define CALI_EFFECTIVE_SAMPLES 150
#define CALI_NOISE_THRESHOLD 0.3f

#define Suggested_Calibration_X -0.13215f
#define Suggested_Calibration_Y -0.0805f
#define Suggested_Calibration_Z -0.36745f

#define Suggested_DeadZone_Range 0.10f

//float gyro_x_bias,gyro_y_bias,gyro_z_bias=0;
static uint8_t is_calibrated=0;
static uint16_t cali_count=0;
//static uint16_t cali_effc_count=0;
float gyro_sum[3]={0};
float gyro_bias[3]={Suggested_Calibration_X,Suggested_Calibration_Y,Suggested_Calibration_Z};
float gyro_sum_sq[3]={0};
float gyro_deadzone[3]={Suggested_DeadZone_Range,Suggested_DeadZone_Range,Suggested_DeadZone_Range};

// get bias while initializing
void gyro_Calibration(float *gyro_x,float *gyro_y,float *gyro_z)
{
	static float raw_gyro_x=0,raw_gyro_y=0,raw_gyro_z=0;
    if(is_calibrated == 0)   
    {
        if(fabsf(*gyro_x - raw_gyro_x) < CALI_NOISE_THRESHOLD && 
		   fabsf(*gyro_y - raw_gyro_y) < CALI_NOISE_THRESHOLD && 
		   fabsf(*gyro_z - raw_gyro_z) < CALI_NOISE_THRESHOLD)
        {
            gyro_sum[0]+=*gyro_x;
            gyro_sum[1]+=*gyro_y;
            gyro_sum[2]+=*gyro_z;
			
			gyro_sum_sq[0]+=(*gyro_x)*(*gyro_x);
			gyro_sum_sq[1]+=(*gyro_y)*(*gyro_y);
			gyro_sum_sq[2]+=(*gyro_z)*(*gyro_z);
			
			cali_count++;            
        }
		else
        {
			cali_count = 0;
			gyro_sum[0] = 0.0f;
			gyro_sum[1] = 0.0f;
			gyro_sum[2] = 0.0f;
			gyro_sum_sq[0]=0.0f;
			gyro_sum_sq[1]=0.0f;
			gyro_sum_sq[2]=0.0f;
        }
		//update the raw gyro
		raw_gyro_x=*gyro_x;
		raw_gyro_y=*gyro_y;
		raw_gyro_z=*gyro_z;
		
		*gyro_x-=gyro_bias[0];
		*gyro_y-=gyro_bias[1];
		*gyro_z-=gyro_bias[2];

		
        if (cali_count >= CALI_SAMPLES)
        {	
			for(uint8_t i=0;i<3;i++)
			{
				gyro_bias[i] = gyro_sum[i] / (float)cali_count;
				
				float variance=(gyro_sum_sq[i] /(float) cali_count) - (gyro_bias[i]) * (gyro_bias[i]);
				//it will run error if variance is an ultra small negative...
				if(variance < 0.0f) variance=0.0f;
				gyro_deadzone[i]=3.0f * sqrtf(variance);
					
			}
			is_calibrated = 1;
			
//			LOG_INFO_APP("------------------------ Auto Calibration Success ---------------\r\n");
//			LOG_INFO_APP("Bias X: %.5f | DeadZone X: %.5f\r\n", gyro_bias[0], gyro_deadzone[0]);
//			LOG_INFO_APP("Bias Y: %.5f | DeadZone Y: %.5f\r\n", gyro_bias[1], gyro_deadzone[1]);
//			LOG_INFO_APP("Bias Z: %.5f | DeadZone Z: %.5f\r\n", gyro_bias[2], gyro_deadzone[2]);
		
        }
    }else if(is_calibrated == 1){
        *gyro_x-=gyro_bias[0];
        *gyro_y-=gyro_bias[1];
        *gyro_z-=gyro_bias[2];
    }
}

void gyro_DeadZone_execute(float *gyro_x,float *gyro_y,float *gyro_z)
{
	if(fabsf(*gyro_x) < gyro_deadzone[0]) *gyro_x=0.0f;
	if(fabsf(*gyro_y) < gyro_deadzone[1]) *gyro_y=0.0f;
	if(fabsf(*gyro_z) < gyro_deadzone[2]) *gyro_z=0.0f;
}