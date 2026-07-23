#include "stm32f10x.h"                  // Device header
#include "MyI2C.h"
#include "QMI8658_Reg.h"

#define QMI8658_ADDRESS		0xD6

void QMI8658_Init(void)
{
	MyI2C_Init();
	
	MyI2C_WriteReg(QMI8658_ADDRESS, QMI8658_CTRL1, 0x60);		//通信接口、中断、芯片使能
	MyI2C_WriteReg(QMI8658_ADDRESS, QMI8658_CTRL2, 0x33);		//加速度计自测、满量程、速率
	MyI2C_WriteReg(QMI8658_ADDRESS, QMI8658_CTRL3, 0x73);		//陀螺仪自测，满量程，速率
	MyI2C_WriteReg(QMI8658_ADDRESS, QMI8658_CTRL5, 0x01);		//陀螺仪和加速度计滤波
	MyI2C_WriteReg(QMI8658_ADDRESS, QMI8658_CTRL7, 0x03);		//陀螺仪和加速度计使能
	MyI2C_WriteReg(QMI8658_ADDRESS, QMI8658_CTRL8, 0x00);		//运动检测
	MyI2C_WriteReg(QMI8658_ADDRESS, QMI8658_CTRL9, 0x00);		//控制命令
}

uint8_t QMI8658_GetID(void)
{
	return MyI2C_ReadReg(QMI8658_ADDRESS, QMI8658_WHO_AM_I);
}

void QMI8658_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t Data[12];
	
	MyI2C_ReadRegs(QMI8658_ADDRESS, QMI8658_AX_L, Data, 12);
	
	*AccX = (Data[1] << 8) | Data[0];
	*AccY = (Data[3] << 8) | Data[2];
	*AccZ = (Data[5] << 8) | Data[4];
	
	*GyroX = (Data[7] << 8) | Data[6];
	*GyroY = (Data[9] << 8) | Data[8];
	*GyroZ = (Data[11] << 8) | Data[10];
}
