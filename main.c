#include <stdio.h>
#include "GY_86.h"



int main()
{
	MPU6050_t MPU6050;
	MPU6050_Init(2,2);
	MPU6050_Bypass(); // Turn on Bypass mode to setup HMC5883L
	HMC5883L_Setup(1);
	MPU6050_Master(5); // Turn off Bypass and turn on Master Mode to read value
	MPU6050_Slave_Read();
	printf("Done setting\n");

	while(1){
		MPU6050_Read_All_Raw(&MPU6050);
		show(&MPU6050);
		delay(1000);
	}

	return 0;
}
