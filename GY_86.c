#include "GY_86.h"
#include "wiringPi.h"
#include "wiringPiI2C.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>


int fd;
int fd_HMC;
uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f
};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

uint8_t MPU6050_rx;
uint8_t MPU6050_rx_buf[20];
uint8_t MPU6050_tx;
float MPU6050_Gyro_LSB;
float MPU6050_Acc_LSB;
float Mag_FS;

uint8_t MPU6050_Init(uint8_t Gyro_FS, uint8_t Acc_FS){
	switch(Gyro_FS)
	{
	case 0: //250dps
		MPU6050_Gyro_LSB = 131.0;
		break;
	case 1: //500dps
		MPU6050_Gyro_LSB = 65.5;
		break;
	case 2: //1000dps
		MPU6050_Gyro_LSB = 32.8;
		break;
	case 3: //2000dps
		MPU6050_Gyro_LSB = 16.4;
		break;
	default:
		break;
	}

	switch(Acc_FS)
	{
	case 0: //2g
		MPU6050_Acc_LSB = 16384.0;
		break;
	case 1: //4g
		MPU6050_Acc_LSB = 8192.0;
		break;
	case 2: //8g
		MPU6050_Acc_LSB = 4096.0;
		break;
	case 3: //16g
		MPU6050_Acc_LSB = 2048.0;
		break;
	default:
		break;
	}

	fd = wiringPiI2CSetup(MPU6050_ADDR);
	if(fd == -1){
		return 1;
	}

	MPU6050_rx = wiringPiI2CReadReg8(fd,WHO_AM_I_REG);
	MPU6050_tx = 0; //Will return this value if code ends here

	if(MPU6050_rx == 0x68){

		// Reset Device
		MPU6050_tx = 0x80;
		wiringPiI2CWriteReg8(fd, PWR_MGMT_1_REG, MPU6050_tx);
		delay(100);

		// Wake device up and Clock source = Internal 8MHz oscillator (default)
		MPU6050_tx = 0;
		wiringPiI2CWriteReg8(fd, PWR_MGMT_1_REG, MPU6050_tx);
		delay(10);

		// Set clock source to PLL with X-axis Gyro 8MHz
		MPU6050_tx = 1;
		wiringPiI2CWriteReg8(fd, PWR_MGMT_1_REG, MPU6050_tx);
		delay(10);

		MPU6050_tx = 7; // Set SMPLRT_DIV = 0 -> sample rate = 1 KHz
		wiringPiI2CWriteReg8(fd, SMPLRT_DIV_REG, MPU6050_tx);
		delay(10);

		MPU6050_tx = 0x03; // Setting Digital Low-Pass Filter depending on sample rate above
		wiringPiI2CWriteReg8(fd, CONFIG_REG, MPU6050_tx);
		delay(10);

		MPU6050_tx = Gyro_FS << 3; // Gyro configure 
		wiringPiI2CWriteReg8(fd, GYRO_CONFIG_REG, MPU6050_tx);
		delay(10);

		MPU6050_tx = Acc_FS << 3; // Acc configure 
		wiringPiI2CWriteReg8(fd, ACCEL_CONFIG_REG, MPU6050_tx);
		delay(10);

		return 0;
	}
	return 1;
}

int MPU6050_Bypass()
{
	MPU6050_tx = 0b00000000; // Precondition to enable Bypass Mode
	wiringPiI2CWriteReg8(fd, USER_CTRL_REG, MPU6050_tx);
	delay(10);

	MPU6050_tx = 0b00000010; // Enable Bypass ModeINT_PIN_CFG
	wiringPiI2CWriteReg8(fd, INT_PIN_CFG, MPU6050_tx);
	delay(10);
	return 0;
}

int MPU6050_Master(uint8_t clk_div) // range of clk_div from 0 to 15
{

	MPU6050_tx = 0x00; // Disable Bypass ModeINT_PIN_CFG
	wiringPiI2CWriteReg8(fd, INT_PIN_CFG, MPU6050_tx);
	delay(10);

	MPU6050_tx = 0b00100010; // Enable I2C Master Mode
	wiringPiI2CWriteReg8(fd, USER_CTRL_REG, MPU6050_tx);
	delay(10);

	MPU6050_tx = clk_div; // You should choose clk_div = 13 <=> I2C Master Clock Speed = 400kHz(fast mode of I2C)
	if(clk_div > 15 || clk_div < 0) return 1;
	wiringPiI2CWriteReg8(fd, I2C_MST_CTRL, MPU6050_tx);
	delay(10);

	MPU6050_tx = 1; // Set clock source to PLL with X-axis Gyro 8MHz
	wiringPiI2CWriteReg8(fd, PWR_MGMT_1_REG, MPU6050_tx);
	delay(10);

	return 0;
}

int MPU6050_Slave_Read()
{

	MPU6050_tx = HMC5883L_ADDRESS | 0x80; //Access Slave into read mode
	wiringPiI2CWriteReg8(fd, I2C_SLV0_ADDR, MPU6050_tx);
	delay(10);

	MPU6050_tx = HMC5883L_REG_OUT_X_M; // Address which HMC5883L's data transfer start
	wiringPiI2CWriteReg8(fd, I2C_SLV0_REG, MPU6050_tx);
	delay(10);

	MPU6050_tx = 0x80 | 0x06; 
	//Enable Slave 0 for data transfer operations and
	//specifies number of bytes transfered (6 bytes respectively x,y and z axis in this case)
	wiringPiI2CWriteReg8(fd, I2C_SLV0_CTRL, MPU6050_tx);
	delay(10);

	return 0;
}

int HMC5883L_Setup(int Mag_Range){
	// mGa/LSB 
	switch(Mag_Range){
	case 0: // 0.88 Ga
		Mag_FS = 0.73;
		break;
	case 1: // 1.3 Ga
		Mag_FS = 0.92;
		break;		
	case 2: // 1.9 Ga
		Mag_FS = 1.22;
		break;
	case 3: // 2.5 Ga
		Mag_FS = 1.52;
		break;
	case 4: // 4 Ga
		Mag_FS = 2.27;
		break;
	case 5: // 4.7 Ga
		Mag_FS = 2.56;
		break;		
	case 6: // 5.6 Ga
		Mag_FS = 3.03;
		break;
	case 7: // 8.1 Ga
		Mag_FS = 4.35;
		break;
	default:
		break;
	}
	fd_HMC = wiringPiI2CSetup(HMC5883L_ADDRESS);
	if(fd_HMC == -1){
		return 1;
	}

	MPU6050_tx = 0b00011000; // Data Output rate 75Hz
	wiringPiI2CWriteReg8(fd_HMC, HMC5883L_REG_CONFIG_A, MPU6050_tx); 
	delay(10);

	MPU6050_tx = Mag_Range<<5; // Set Range of sensor
	wiringPiI2CWriteReg8(fd_HMC, HMC5883L_REG_CONFIG_B, MPU6050_tx);
	delay(10);

	MPU6050_tx = 0b00000000; // Continuous mode
	wiringPiI2CWriteReg8(fd_HMC, HMC5883L_REG_MODE, MPU6050_tx);
	delay(10);

	return 0;
}

void MPU6050_Read_All_Raw(MPU6050_t *DataStruct)
{
	DataStruct->Accel_X_RAW = (int16_t)((wiringPiI2CReadReg8(fd, ACCEL_XOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, ACCEL_XOUT_L_REG));
	DataStruct->Accel_Y_RAW = (int16_t)((wiringPiI2CReadReg8(fd, ACCEL_YOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, ACCEL_YOUT_L_REG));
	DataStruct->Accel_Z_RAW = (int16_t)((wiringPiI2CReadReg8(fd, ACCEL_ZOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, ACCEL_ZOUT_L_REG));

	DataStruct->TEMP_RAW = (int16_t)((wiringPiI2CReadReg8(fd, TEMP_OUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, TEMP_OUT_L_REG));

	DataStruct->Gyro_X_RAW = (int16_t)((wiringPiI2CReadReg8(fd, GYRO_XOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, GYRO_XOUT_L_REG));
	DataStruct->Gyro_Y_RAW = (int16_t)((wiringPiI2CReadReg8(fd, GYRO_YOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, GYRO_YOUT_L_REG));
	DataStruct->Gyro_Z_RAW = (int16_t)((wiringPiI2CReadReg8(fd, GYRO_ZOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, GYRO_ZOUT_L_REG));
	
	DataStruct->Mag_X_RAW = (int16_t)((wiringPiI2CReadReg8(fd, REG_OUT_MAG_X_M) << 8) | wiringPiI2CReadReg8(fd, REG_OUT_MAG_X_L));
	DataStruct->Mag_Z_RAW = (int16_t)((wiringPiI2CReadReg8(fd, REG_OUT_MAG_Z_M) << 8) | wiringPiI2CReadReg8(fd, REG_OUT_MAG_Z_L));
	DataStruct->Mag_Y_RAW = (int16_t)((wiringPiI2CReadReg8(fd, REG_OUT_MAG_Y_M) << 8) | wiringPiI2CReadReg8(fd, REG_OUT_MAG_Y_L));



	DataStruct->Gx = (DataStruct->Gyro_X_RAW / MPU6050_Gyro_LSB)* D2R;
	DataStruct->Gy = (DataStruct->Gyro_Y_RAW / MPU6050_Gyro_LSB)* D2R;
	DataStruct->Gz = (DataStruct->Gyro_Z_RAW / MPU6050_Gyro_LSB)* D2R;

	DataStruct->Temperature = (float)(DataStruct->TEMP_RAW / (float)340.0 + (float)36.53); // Unit is degrees

	DataStruct->Ax = DataStruct->Accel_X_RAW / MPU6050_Acc_LSB;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / MPU6050_Acc_LSB;
	DataStruct->Az = DataStruct->Accel_Z_RAW / MPU6050_Acc_LSB;
}

void MPU6050_Read_All_Kalman(MPU6050_t *DataStruct){

	DataStruct->Accel_X_RAW = (int16_t)((wiringPiI2CReadReg8(fd, ACCEL_XOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, ACCEL_XOUT_L_REG));
	DataStruct->Accel_Y_RAW = (int16_t)((wiringPiI2CReadReg8(fd, ACCEL_YOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, ACCEL_YOUT_L_REG));
	DataStruct->Accel_Z_RAW = (int16_t)((wiringPiI2CReadReg8(fd, ACCEL_ZOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, ACCEL_ZOUT_L_REG));

	DataStruct->TEMP_RAW = (int16_t)((wiringPiI2CReadReg8(fd, TEMP_OUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, TEMP_OUT_L_REG));

	DataStruct->Gyro_X_RAW = (int16_t)((wiringPiI2CReadReg8(fd, GYRO_XOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, GYRO_XOUT_L_REG));
	DataStruct->Gyro_Y_RAW = (int16_t)((wiringPiI2CReadReg8(fd, GYRO_YOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, GYRO_YOUT_L_REG));
	DataStruct->Gyro_Z_RAW = (int16_t)((wiringPiI2CReadReg8(fd, GYRO_ZOUT_H_REG) << 8) | wiringPiI2CReadReg8(fd, GYRO_ZOUT_L_REG));
	
	DataStruct->Mag_X_RAW = (int16_t)((wiringPiI2CReadReg8(fd, REG_OUT_MAG_X_M) << 8) | wiringPiI2CReadReg8(fd, REG_OUT_MAG_X_L));
	DataStruct->Mag_Z_RAW = (int16_t)((wiringPiI2CReadReg8(fd, REG_OUT_MAG_Z_M) << 8) | wiringPiI2CReadReg8(fd, REG_OUT_MAG_Z_L));
	DataStruct->Mag_Y_RAW = (int16_t)((wiringPiI2CReadReg8(fd, REG_OUT_MAG_Y_M) << 8) | wiringPiI2CReadReg8(fd, REG_OUT_MAG_Y_L));

	DataStruct->Gyro_X_RAW -= DataStruct->Gyro_X_Offset;
	DataStruct->Gyro_Y_RAW -= DataStruct->Gyro_Y_Offset;
	DataStruct->Gyro_Z_RAW -= DataStruct->Gyro_Z_Offset;

	DataStruct->Mag_X_RAW -= DataStruct->Mag_X_Offset;
	DataStruct->Mag_Y_RAW -= DataStruct->Mag_Y_Offset;
	DataStruct->Mag_Z_RAW -= DataStruct->Mag_Z_Offset;

	DataStruct->Gx = (DataStruct->Gyro_X_RAW / MPU6050_Gyro_LSB)* D2R;
	DataStruct->Gy = (DataStruct->Gyro_Y_RAW / MPU6050_Gyro_LSB)* D2R;
	DataStruct->Gz = (DataStruct->Gyro_Z_RAW / MPU6050_Gyro_LSB)* D2R;

	DataStruct->Temperature = (float)(DataStruct->TEMP_RAW / (float)340.0 + (float)36.53);

	DataStruct->Ax = DataStruct->Accel_X_RAW / MPU6050_Acc_LSB;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / MPU6050_Acc_LSB;
	DataStruct->Az = DataStruct->Accel_Z_RAW / MPU6050_Acc_LSB;

	DataStruct->Mx = DataStruct->Mag_X_RAW * Mag_FS;
	DataStruct->My = DataStruct->Mag_Y_RAW * Mag_FS;
	DataStruct->Mz = DataStruct->Mag_Z_RAW * Mag_FS;


	// Kalman angle solve
    double dt = (double)(millis() - timer) / 1000.0;
    timer = millis(); // Update timestamp
    double roll;
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};

void show(MPU6050_t *data){
	printf("MPU6050 Sensor Data:\n");
    printf("--------------------\n");
    printf("Acceleration (RAW): X=%d, Y=%d, Z=%d\n", data->Accel_X_RAW, data->Accel_Y_RAW, data->Accel_Z_RAW);
    printf("Temperature (RAW): %d\n", data->TEMP_RAW);
    printf("Gyroscope (RAW): X=%d, Y=%d, Z=%d\n", data->Gyro_X_RAW, data->Gyro_Y_RAW, data->Gyro_Z_RAW);
    printf("Magnetometer (RAW): X=%d, Y=%d, Z=%d\n", data->Mag_X_RAW, data->Mag_Y_RAW, data->Mag_Z_RAW);
    printf("--------------------\n");
}

