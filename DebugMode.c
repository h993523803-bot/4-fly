#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "LED.h"
#include "OLED.h"
#include "Timer.h"
#include "Key.h"
#include "PWM.h"
#include "AD.h"
#include "NRF24L01.h"
#include "Serial.h"
#include "MyI2C.h"
#include "QMI8658.h"
#include "QMC6309.h"
#include "SPA06.h"

#include "VL53L1X.h"
#include "PMW3901.h"
#include "PAA3905.h"

#include "Store.h"
#include "IMU.h"
#include "PID.h"
#include "Utils.h"
#include "DebugMode.h"
#include "Menu.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

extern uint8_t DebugFlag;
extern int16_t AXOffset, AYOffset, AZOffset, GXOffset, GYOffset, GZOffset;
extern uint8_t RxData;
extern uint8_t RxMode;
extern void LoadParam(void);

void DebugMode(void)
{
	ListStack[0] = &MainMenu;
	
	Key_Clear();
	OLED_Clear();
	
	while (DebugFlag)
	{
		if (Key_Check(KEY_1, KEY_SINGLE))
		{
			Menu_Down();
		}
		if (Key_Check(KEY_1, KEY_LONG))
		{
			Menu_Enter();
		}
		
		if (NRF24L01_Receive() == 1)
		{
			uint8_t ID = NRF24L01_RxPacket[0];
			if (ID == 0x10 || ID == 0x11)
			{
				uint8_t KEY = NRF24L01_RxPacket[5];
				if (KEY == 5)
				{
					Menu_Up();
				}
				if (KEY == 6)
				{
					Menu_Down();
				}
				if (KEY == 7)
				{
					Menu_Back();
				}
				if (KEY == 8)
				{
					Menu_Enter();
				}
			}
		}
		
		Menu_Display();
	}
	
	Key_Clear();
	OLED_Clear();
}

uint16_t DelayCount;

void DebugMode_Tick(void)
{
	if (DelayCount > 0)
	{
		DelayCount --;
	}
}

void DebugMode_Back(void)
{
	DebugFlag = 0;
}

uint8_t TestBreak(void)
{
	if (Key_Check(KEY_1, KEY_SINGLE) || Key_Check(KEY_1, KEY_LONG))
	{
		return 1;
	}
		
	if (NRF24L01_Receive() == 1)
	{
		uint8_t ID = NRF24L01_RxPacket[0];
		if (ID == 0x10 || ID == 0x11)
		{
			uint8_t KEY = NRF24L01_RxPacket[5];
			if (KEY == 5 || KEY == 6 || KEY == 7 || KEY == 8)
			{
				return 1;
			}
		}
	}
	
	return 0;
}

void LED_Test(void)
{
	uint8_t n = 1;
	
	Key_Clear();
	OLED_Clear();
	OLED_Printf(0, 0, OLED_8X16, "LED");
	OLED_Update();
	
	while (1)
	{
		if (TestBreak()) break;
		
		if (DelayCount == 0)
		{
			DelayCount = 1000;
			
			LED_SetMode(1, LED_OFF);
			LED_SetMode(2, LED_OFF);
			LED_SetMode(3, LED_OFF);
			LED_SetMode(4, LED_OFF);
			LED_SetMode(n, LED_ON);
			
			OLED_Printf(0, 16, OLED_8X16, "ON:%1d", n);
			OLED_Update();
			
			n ++;
			if (n > 4) n = 1;
		}
	}
	
	LED_Set(1, LED_OFF);
	LED_Set(2, LED_OFF);
	LED_Set(3, LED_OFF);
	LED_Set(4, LED_OFF);
	DelayCount = 0;
}

void Motor_Test(void)
{
	uint8_t n = 1;
	
	Key_Clear();
	OLED_Clear();
	OLED_Printf(0, 0, OLED_8X16, "Motor");
	OLED_Update();
	
	while (1)
	{
		if (TestBreak()) break;
		
		if (DelayCount == 0)
		{
			DelayCount = 1000;
			
			PWM_SetCompare1(0);
			PWM_SetCompare2(0);
			PWM_SetCompare3(0);
			PWM_SetCompare4(0);
			if (n == 1) PWM_SetCompare1(20);
			else if (n == 2) PWM_SetCompare2(20);
			else if (n == 3) PWM_SetCompare3(20);
			else if (n == 4) PWM_SetCompare4(20);
			
			OLED_Printf(0, 16, OLED_8X16, "Run:%1d", n);
			OLED_Update();
			
			n ++;
			if (n > 4) n = 1;
		}
	}
	
	PWM_SetCompare1(0);
	PWM_SetCompare2(0);
	PWM_SetCompare3(0);
	PWM_SetCompare4(0);
	DelayCount = 0;
}

void AD_Test(void)
{
	Key_Clear();
	OLED_Clear();
	OLED_Printf(0, 0, OLED_8X16, "AD");
	OLED_Update();
	
	while (1)
	{
		if (TestBreak()) break;
		
		OLED_Printf(0, 16, OLED_8X16, "ADValue:%04d", AD_GetValue());
		OLED_Printf(0, 32, OLED_8X16, "Voltage:%4.2fV", AD_GetBatteryVoltage());
		OLED_Update();
	}
}

void NRF24L01_Test(void)
{
	uint8_t WriteArray[5] = {0};
	uint8_t ReadArray[5] = {0};
	
	Key_Clear();
	OLED_Clear();
	OLED_Printf(0, 0, OLED_8X16, "NRF24L01");
	OLED_Update();
	
	while (1)
	{
		if (TestBreak()) break;
		
		if (DelayCount == 0)
		{
			DelayCount = 1000;
			
			WriteArray[0] += 1;
			WriteArray[1] += 2;
			WriteArray[2] += 3;
			WriteArray[3] += 4;
			WriteArray[4] += 5;
			NRF24L01_WriteRegs(NRF24L01_TX_ADDR, WriteArray, 5);
			
			NRF24L01_ReadRegs(NRF24L01_TX_ADDR, ReadArray, 5);
			
			OLED_Printf(0, 16, OLED_8X16, "W:%02X %02X %02X %02X %02X", 
				WriteArray[0], WriteArray[1], WriteArray[2], WriteArray[3], WriteArray[4]);
			OLED_Printf(0, 32, OLED_8X16, "R:%02X %02X %02X %02X %02X", 
				ReadArray[0], ReadArray[1], ReadArray[2], ReadArray[3], ReadArray[4]);
			OLED_Update();
		}
	}
	DelayCount = 0;
}

void Serial_Test(void)
{
	uint8_t TxData = 0;
	
	Key_Clear();
	OLED_Clear();
	OLED_Printf(0, 0, OLED_8X16, "Serial");
	OLED_Update();
	
	while (1)
	{
		if (TestBreak()) break;
		
		if (DelayCount == 0)
		{
			DelayCount = 1000;
			
			TxData ++;
			
			Serial_SendByte(TxData);
						
			OLED_Printf(0, 16, OLED_8X16, "TxData:%02X", TxData);
		}
		
		OLED_Printf(0, 32, OLED_8X16, "RxData:%02X", RxData);
		OLED_Update();
	}
	DelayCount = 0;
}

void QMI8658_Test(void)
{
	int16_t AX, AY, AZ, GX, GY, GZ;
	
	Key_Clear();
	OLED_Clear();
	OLED_Printf(0, 0, OLED_8X16, "QMI8658 ID:%02X", QMI8658_GetID());
	OLED_Update();
	
	while (1)
	{
		if (TestBreak()) break;
		
		QMI8658_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		OLED_Printf(0, 16, OLED_8X16, "%+06d %+06d", AX, GX);
		OLED_Printf(0, 32, OLED_8X16, "%+06d %+06d", AY, GY);
		OLED_Printf(0, 48, OLED_8X16, "%+06d %+06d", AZ, GZ);
		
		OLED_Update();
	}
}

void QMC6309_Test(void)
{
	int16_t MX, MY, MZ;
	
	Key_Clear();
	OLED_Clear();
	OLED_Printf(0, 0, OLED_8X16, "QMC6309 ID:%02X", QMC6309_GetID());
	OLED_Update();
	
	while (1)
	{
		if (TestBreak()) break;
		
		QMC6309_GetData(&MX, &MY, &MZ);
		
		OLED_Printf(0, 16, OLED_8X16, "MX:%+06d", MX);
		OLED_Printf(0, 32, OLED_8X16, "MY:%+06d", MY);
		OLED_Printf(0, 48, OLED_8X16, "MZ:%+06d", MZ);
		
		OLED_Update();
	}
}

void SPA06_Test(void)
{
	float BaroPress, BaroTemp, BaroAlt;
	
	Key_Clear();
	OLED_Clear();
	OLED_Printf(0, 0, OLED_8X16, "SPA06 ID:%02X", SPA06_GetID());
	OLED_Update();
	
	while (1)
	{
		if (TestBreak()) break;
		
		SPA06_GetData(&BaroPress, &BaroTemp);
		
		BaroAlt = 44330.0f * (1.0f - powf(BaroPress / 101325.0f, 1.0f / 5.255f));
		
		OLED_Printf(0, 16, OLED_8X16, "P:%09.2fPa", BaroPress);
		OLED_Printf(0, 32, OLED_8X16, "A:%+09.2fm", BaroAlt);
		
		OLED_Update();
	}
}

void VL53L1X_Test(void)
{
	int16_t LaserDistance;
	
	Key_Clear();
	OLED_Clear();
	OLED_Printf(0, 0, OLED_8X16, "VL53L1X ID:%04X", VL53L1X_GetID());
	OLED_Update();
	
	while (1)
	{
		if (TestBreak()) break;
		
		LaserDistance = VL53L1X_GetData();
		
		OLED_Printf(0, 16, OLED_8X16, "D:%+05dmm", LaserDistance);
		
		OLED_Update();
	}
}

void PAA3905_Test(void)
{
	int16_t DeltaX, DeltaY;
	int32_t X = 0, Y = 0;
	
	Key_Clear();
	OLED_Clear();
	OLED_Printf(0, 0, OLED_8X16, "PAA3905 ID:%02X", PAA3905_GetID());
	OLED_Update();
	
	while (1)
	{
		if (TestBreak()) break;
		
		PAA3905_GetData(&DeltaX, &DeltaY);
		X += DeltaX;
		Y += DeltaY;
		
		OLED_Printf(0, 16, OLED_8X16, "DX:%+04d DY:%+04d", DeltaX, DeltaY);
		OLED_Printf(0, 32, OLED_8X16, "X:%+011d", X);
		OLED_Printf(0, 48, OLED_8X16, "Y:%+011d", Y);
		
		OLED_Update();
	}
}

void Acc_Cali(void)
{
	uint8_t i, CaliFlag;
	int16_t AX, AY, AZ, GX, GY, GZ;
	static float AXBuffer[50], AYBuffer[50], AZBuffer[50];
	int16_t AXFilter, AYFilter, AZFilter;
	
	Key_Clear();
	OLED_Clear();
	
	i = 5;
	DelayCount = 1000;
	
	while (1)
	{
		if (TestBreak())
		{
			CaliFlag = 0;
			break;
		}
		if (DelayCount == 0)
		{
			i --;
			if (i == 0)
			{
				CaliFlag = 1;
				OLED_Clear();
				break;
			}
			DelayCount = 1000;
		}
		
		QMI8658_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		
		AverageFilter(AXBuffer, 50, AX);
		AverageFilter(AYBuffer, 50, AY);
		AverageFilter(AZBuffer, 50, AZ);
		
		OLED_Printf(0, 0, OLED_8X16, "Acc  准备校准  %d", i);
		OLED_Printf(0, 32, OLED_8X16, "请保持水平且静止");
		
		OLED_Update();
	}
	
	i = 5;
	DelayCount = 1000;
	
	while (CaliFlag == 1)
	{
		if (TestBreak())
		{
			CaliFlag = 0;
			break;
		}
		if (DelayCount == 0)
		{
			i --;
			if (i == 0)
			{
				CaliFlag = 1;
				break;
			}
			DelayCount = 1000;
		}
		
		QMI8658_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		
		AXFilter = AverageFilter(AXBuffer, 50, AX);
		AYFilter = AverageFilter(AYBuffer, 50, AY);
		AZFilter = AverageFilter(AZBuffer, 50, AZ);
		
		OLED_Printf(0, 0, OLED_8X16, "Acc  正在校准  %d", i);
		OLED_Printf(0, 16, OLED_8X16, "AX:%+06d", AXFilter);
		OLED_Printf(0, 32, OLED_8X16, "AY:%+06d", AYFilter);
		OLED_Printf(0, 48, OLED_8X16, "AZ:%+06d", AZFilter);
		
		OLED_Update();
	}
	
	if (CaliFlag == 1)
	{
		AXOffset = AXFilter;
		AYOffset = AYFilter;
		AZOffset = AZFilter - 2048;
		
		Store_Data[1] = AXOffset;
		Store_Data[2] = AYOffset;
		Store_Data[3] = AZOffset;
		
		Store_Save();
		
		OLED_Printf(0, 0, OLED_8X16, "Acc  校准完成   ");
		
		OLED_Update();
	}
	else
	{
		OLED_Printf(0, 0, OLED_8X16, "Acc  校准取消   ");
		
		OLED_Update();
	}
	
	while (!TestBreak());
	
	DelayCount = 0;
}

void Gyro_Cali(void)
{
	uint8_t i, CaliFlag;
	int16_t AX, AY, AZ, GX, GY, GZ;
	static float GXBuffer[50], GYBuffer[50], GZBuffer[50];
	int16_t GXFilter, GYFilter, GZFilter;
	
	Key_Clear();
	OLED_Clear();
	
	i = 5;
	DelayCount = 1000;
	
	while (1)
	{
		if (TestBreak())
		{
			CaliFlag = 0;
			break;
		}
		if (DelayCount == 0)
		{
			i --;
			if (i == 0)
			{
				CaliFlag = 1;
				OLED_Clear();
				break;
			}
			DelayCount = 1000;
		}
		
		QMI8658_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		
		AverageFilter(GXBuffer, 50, GX);
		AverageFilter(GYBuffer, 50, GY);
		AverageFilter(GZBuffer, 50, GZ);
		
		OLED_Printf(0, 0, OLED_8X16, "Gyro 准备校准  %d", i);
		OLED_Printf(0, 32, OLED_8X16, "   请保持静止");
		
		OLED_Update();
	}
		
	i = 5;
	DelayCount = 1000;
	
	while (CaliFlag == 1)
	{
		if (TestBreak())
		{
			CaliFlag = 0;
			break;
		}
		if (DelayCount == 0)
		{
			i --;
			if (i == 0)
			{
				CaliFlag = 1;
				break;
			}
			DelayCount = 1000;
		}
		
		QMI8658_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		
		GXFilter = AverageFilter(GXBuffer, 50, GX);
		GYFilter = AverageFilter(GYBuffer, 50, GY);
		GZFilter = AverageFilter(GZBuffer, 50, GZ);
		
		OLED_Printf(0, 0, OLED_8X16, "Gyro 正在校准  %d", i);
		OLED_Printf(0, 16, OLED_8X16, "GX:%+06d", GXFilter);
		OLED_Printf(0, 32, OLED_8X16, "GY:%+06d", GYFilter);
		OLED_Printf(0, 48, OLED_8X16, "GZ:%+06d", GZFilter);
		
		OLED_Update();
	}
	
	if (CaliFlag == 1)
	{
		GXOffset = GXFilter;
		GYOffset = GYFilter;
		GZOffset = GZFilter;
		
		Store_Data[4] = GXOffset;
		Store_Data[5] = GYOffset;
		Store_Data[6] = GZOffset;
		
		Store_Save();
		
		OLED_Printf(0, 0, OLED_8X16, "Gyro 校准完成   ");
		
		OLED_Update();
	}
	else
	{
		OLED_Printf(0, 0, OLED_8X16, "Gyro 校准取消   ");
		
		OLED_Update();
	}
	
	while (!TestBreak());
	
	DelayCount = 0;
}

void Reset_Cali(void)
{
	Key_Clear();
	OLED_Clear();
	
	Store_Clear();
	LoadParam();
	
	OLED_Printf(0, 16, OLED_8X16, " 已重置校准参数");
	OLED_Update();
	
	while (!TestBreak());
}

void Bluetooth_Reload(void)
{
	uint32_t BaudRateArray[] = {9600, 115200, 19200, 38400, 57600, 1000000};
	
	Serial_ChangeMode(1);
	
	Key_Clear();
	OLED_Clear();
	
	for (uint8_t i = 0; i < 6; i ++)
	{
		Serial_Init(BaudRateArray[i]);
		OLED_Printf(0, 0, OLED_8X16, "STM32:%d", BaudRateArray[i]);
		OLED_Update();
		Delay_ms(500);
		
		Serial_RxFlag = 0;
		Serial_Printf("AT...\r\n");
		OLED_Printf(0, 16, OLED_8X16, "进入AT模式");
		OLED_Update();
		Delay_ms(500);
		
		if (Serial_RxFlag && strcmp(Serial_RxPacket, "OK") == 0)
		{
			OLED_Printf(112, 16, OLED_8X16, "OK");
			OLED_Update();
			Delay_ms(500);
		}
		else
		{
			OLED_Printf(112, 16, OLED_8X16, "ER");
			OLED_Update();
			Delay_ms(500);
			
			OLED_Clear();
			
			continue;
		}
		
		Serial_RxFlag = 0;
		Serial_Printf("AT+RELOAD\r\n");
		OLED_Printf(0, 32, OLED_8X16, "重置参数");
		OLED_Update();
		Delay_ms(500);
		
		if (Serial_RxFlag && strcmp(Serial_RxPacket, "OK") == 0)
		{
			OLED_Printf(112, 32, OLED_8X16, "OK");
			OLED_Update();
			Delay_ms(500);
			
			break;
		}
		else
		{
			OLED_Printf(112, 32, OLED_8X16, "ER");
			OLED_Update();
			Delay_ms(500);
			
			break;
		}
	}
	
	Serial_RxFlag = 0;
	Serial_Printf("AT+RESET\r\n");
	OLED_Printf(0, 48, OLED_8X16, "重启蓝牙");
	OLED_Update();
	Delay_ms(500);
	
	if (Serial_RxFlag && strcmp(Serial_RxPacket, "OK") == 0)
	{
		OLED_Printf(112, 48, OLED_8X16, "OK");
		OLED_Update();
		Delay_ms(500);
	}
	else
	{
		OLED_Printf(112, 48, OLED_8X16, "ER");
		OLED_Update();
		Delay_ms(500);
	}
	
	Serial_RxFlag = 0;
	
	while (!TestBreak());
	
	Serial_ChangeMode(0);
}

void Bluetooth_Config9600(void)
{
	uint32_t BaudRateArray[] = {9600, 115200, 19200, 38400, 57600, 1000000};
	
	Serial_ChangeMode(1);
	
	Key_Clear();
	OLED_Clear();
	
	for (uint8_t i = 0; i < 6; i ++)
	{
		Serial_Init(BaudRateArray[i]);
		OLED_Printf(0, 0, OLED_8X16, "STM32:%d   ", BaudRateArray[i]);
		OLED_Update();
		Delay_ms(500);
		
		Serial_RxFlag = 0;
		Serial_Printf("AT...\r\n");
		OLED_Printf(0, 16, OLED_8X16, "进入AT模式");
		OLED_Update();
		Delay_ms(500);
		
		if (Serial_RxFlag && strcmp(Serial_RxPacket, "OK") == 0)
		{
			OLED_Printf(112, 16, OLED_8X16, "OK");
			OLED_Update();
			Delay_ms(500);
		}
		else
		{
			OLED_Printf(112, 16, OLED_8X16, "ER");
			OLED_Update();
			Delay_ms(500);
			
			OLED_Clear();
			
			continue;
		}
		
		Serial_RxFlag = 0;
		Serial_Printf("AT+UART=9600,8,1,0,0\r\n");
		OLED_Printf(0, 32, OLED_8X16, "设波特率9600");
		OLED_Update();
		Delay_ms(500);
		
		if (Serial_RxFlag && strcmp(Serial_RxPacket, "OK") == 0)
		{
			OLED_Printf(112, 32, OLED_8X16, "OK");
			OLED_Update();
			Delay_ms(500);
			
			Serial_Init(9600);
			OLED_Printf(0, 0, OLED_8X16, "STM32:%d   ", 9600);
			
			break;
		}
		else
		{
			OLED_Printf(112, 32, OLED_8X16, "ER");
			OLED_Update();
			Delay_ms(500);
			
			break;
		}
	}
	
	Serial_RxFlag = 0;
	Serial_Printf("AT+RESET\r\n");
	OLED_Printf(0, 48, OLED_8X16, "重启蓝牙");
	OLED_Update();
	Delay_ms(500);
	
	if (Serial_RxFlag && strcmp(Serial_RxPacket, "OK") == 0)
	{
		OLED_Printf(112, 48, OLED_8X16, "OK");
		OLED_Update();
		Delay_ms(500);
	}
	else
	{
		OLED_Printf(112, 48, OLED_8X16, "ER");
		OLED_Update();
		Delay_ms(500);
	}
	
	Serial_RxFlag = 0;
	
	while (!TestBreak());
	
	Serial_ChangeMode(0);
}
