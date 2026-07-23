#include "stm32f10x.h"                  // Device header
#include "MyI2C.h"
#include "SPA06_Reg.h"
#include <math.h>

#define SPA06_ADDRESS		0xEC

#define SPA06_SCALE_1		524288.0f
#define SPA06_SCALE_2		1572864.0f
#define SPA06_SCALE_4		3670016.0f
#define SPA06_SCALE_8		7864320.0f
#define SPA06_SCALE_16		253952.0f
#define SPA06_SCALE_32		516096.0f
#define SPA06_SCALE_64		1040384.0f
#define SPA06_SCALE_128		2088960.0f

static int16_t c0, c1;						//12bit
static int32_t c00, c10;					//20bit
static int16_t c01, c11, c20, c21, c30;		//16bit
static int16_t c31, c40;					//12bit

void SPA06_ReadCali(void)
{
	uint8_t Cali[21];
	
	MyI2C_ReadRegs(SPA06_ADDRESS, SPA06_COEF, Cali, 21);
	
	c0 = (Cali[0] << 4) | (Cali[1] >> 4);
	if(c0 & 0x0800) c0 |= 0xF000;
	
    c1 = ((Cali[1] & 0x0F) << 8) | Cali[2];
    if(c1 & 0x0800) c1 |= 0xF000;
	
    c00 = (Cali[3] << 12) | (Cali[4] << 4) | (Cali[5] >> 4);
    if(c00 & 0x00080000) c00 |= 0xFFF00000;
	
    c10 = ((Cali[5] & 0x0F) << 16) | (Cali[6] << 8) | Cali[7];
    if(c10 & 0x00080000) c10 |= 0xFFF00000;
	
    c01 = (Cali[8] << 8) | Cali[9];
    c11 = (Cali[10] << 8) | Cali[11];
    c20 = (Cali[12] << 8) | Cali[13];
    c21 = (Cali[14] << 8) | Cali[15];
    c30 = (Cali[16] << 8) | Cali[17];
	
    c31 = (Cali[18] << 4) | (Cali[19] >> 4);
	if(c31 & 0x0800) c31 |= 0xF000;
	
    c40 = ((Cali[19] & 0x0F) << 8) | Cali[20];
	if(c40 & 0x0800) c40 |= 0xF000;
}

void SPA06_Init(void)
{
	MyI2C_Init();
	
	SPA06_ReadCali();
	
	MyI2C_WriteReg(SPA06_ADDRESS, SPA06_PRS_CFG, 0x54);
	MyI2C_WriteReg(SPA06_ADDRESS, SPA06_TMP_CFG, 0x50);
	MyI2C_WriteReg(SPA06_ADDRESS, SPA06_CFG_REG, 0x04);
	MyI2C_WriteReg(SPA06_ADDRESS, SPA06_MEAS_CFG, 0x07);	
}

uint8_t SPA06_GetID(void)
{
	return MyI2C_ReadReg(SPA06_ADDRESS, SPA06_ID);
}

void SPA06_GetData(float *P, float *T)
{
	uint8_t Data[6];
	int32_t Praw, Traw;
	float Praw_sc, Traw_sc;
	float Pcomp, Tcomp;
	
	MyI2C_ReadRegs(SPA06_ADDRESS, SPA06_PSR_B2, Data, 6);
	
	Praw = (Data[0] << 16) | (Data[1] << 8) | (Data[2]);
	Traw = (Data[3] << 16) | (Data[4] << 8) | (Data[5]);
	
	if (Praw & 0x00800000) {Praw |= 0xFF000000;}
	if (Traw & 0x00800000) {Traw |= 0xFF000000;}
	
	Praw_sc = Praw / SPA06_SCALE_16;
	Traw_sc = Traw / SPA06_SCALE_1;
	
	float P2raw_sc = Praw_sc * Praw_sc;
	float P3raw_sc = P2raw_sc * Praw_sc;
	float P4raw_sc = P3raw_sc * Praw_sc;
	
	Pcomp = c00 + c10 * Praw_sc + c20 * P2raw_sc + c30 * P3raw_sc + c40 * P4raw_sc
		  + Traw_sc * (c01 + c11 * Praw_sc + c21 * P2raw_sc + c31 * P3raw_sc);
	
	Tcomp = c0 * 0.5f + c1 * Traw_sc;
	
	*P = Pcomp;
	*T = Tcomp;
}
