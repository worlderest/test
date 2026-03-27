#include <stdint.h>
#include <math.h>
#include "Filter.h"

#define Fre_Sample 50
#define Fre_CutOff 5

#define PI 3.14159265358979323846

//EMA_Alpha ranges 0~1 ,the more approach 0, the filter is more strong.
static float EMA_Alpha=2.0f * PI * Fre_CutOff /(Fre_Sample + 2.0f * PI * Fre_CutOff);

void EMA_Filter(float *acc_x,float *acc_y,float *acc_z)
{
	static float output_x=0.0f;
	static float output_y=0.0f;
	static float output_z=0.0f;
	
	static uint8_t is_first_run=1;
	if(is_first_run == 1)
	{
		is_first_run=0;
	}
	else
	{
		*acc_x=(*acc_x) * EMA_Alpha + output_x * (1.0f - EMA_Alpha);
		*acc_y=(*acc_y) * EMA_Alpha + output_y * (1.0f - EMA_Alpha);
		*acc_z=(*acc_z) * EMA_Alpha + output_z * (1.0f - EMA_Alpha);
	}
	
	//update the output saved in the function
	output_x=*acc_x;
	output_y=*acc_y;
	output_z=*acc_z;
}