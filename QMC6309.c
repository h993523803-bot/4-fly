#include "stm32f10x.h"                  // Device header
#include "MyI2C.h"
#include "QMC6309_Reg.h"
#include "OLED.h"

#define QMC6309_ADDRESS		0xF8


void QMC6309_Init(void)
{
	MyI2C_Init();
	
//	MyI2C_WriteReg(QMC6309_ADDRESS, QMC6309_CTRL2, 0x80);
//	MyI2C_WriteReg(QMC6309_ADDRESS, QMC6309_CTRL2, 0x00);
	
	MyI2C_WriteReg(QMC6309_ADDRESS, QMC6309_CTRL2, 0x40);
	MyI2C_WriteReg(QMC6309_ADDRESS, QMC6309_CTRL1, 0x01);
}

uint8_t QMC6309_GetID(void)
{
	return MyI2C_ReadReg(QMC6309_ADDRESS, QMC6309_CHIP_ID);
}

void QMC6309_GetData(int16_t *MagX, int16_t *MagY, int16_t *MagZ)
{
	uint8_t Data[6];
	
	MyI2C_ReadRegs(QMC6309_ADDRESS, QMC6309_XOUT_L, Data, 6);
		
	*MagX = (int16_t)((Data[1] << 8) | Data[0]);
	*MagY = (int16_t)((Data[3] << 8) | Data[2]);
	*MagZ = (int16_t)((Data[5] << 8) | Data[4]);

}
