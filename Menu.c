#include "stm32f10x.h"                  // Device header
#include "Menu.h"
#include "OLED.h"
#include "LED.h"
#include "Key.h"
#include "DebugMode.h"

struct Menu_List MainMenu = {
	.Name = "   [调试模式]   ",
	.BackFunction = DebugMode_Back,
	
	.WindowIndex = 0,
	.WindowLength = 3,
	
	.ItemIndex = 0,
	.ItemLength = 4,
	.Items = {
		{.Name = " < 返回", .Function = DebugMode_Back},
		{.Name = " 1.硬件测试", .NextList = &HardwareTestMenu},
		{.Name = " 2.IMU校准", .NextList = &CalibrationMenu},
		{.Name = " 3.蓝牙配置", .NextList = &BluetoothMenu},
	}
};

struct Menu_List HardwareTestMenu = {
	.Name = "   [硬件测试]   ",
	
	.WindowIndex = 0,
	.WindowLength = 3,
	
	.ItemIndex = 0,
	.ItemLength = 11,
	.Items = {
		{.Name = " < 返回", .Function = Menu_Back},
		{.Name = " 1.LED", .Function = LED_Test},
		{.Name = " 2.Motor", .Function = Motor_Test},
		{.Name = " 3.AD", .Function = AD_Test},
		{.Name = " 4.NRF24L01", .Function = NRF24L01_Test},
		{.Name = " 5.Serial", .Function = Serial_Test},
		{.Name = " 6.QMI8658", .Function = QMI8658_Test},
		{.Name = " 7.QMC6309", .Function = QMC6309_Test},
		{.Name = " 8.SPA06", .Function = SPA06_Test},
		{.Name = " 9.VL53L1X", .Function = VL53L1X_Test},
		{.Name = " 10.PAA3905", .Function = PAA3905_Test},
	}
};

struct Menu_List CalibrationMenu = {
	.Name = "   [IMU校准]    ",
	
	.WindowIndex = 0,
	.WindowLength = 3,
	
	.ItemIndex = 0,
	.ItemLength = 4,
	.Items = {
		{.Name = " < 返回", .Function = Menu_Back},
		{.Name = " 1.重置校准参数", .Function = Reset_Cali},
		{.Name = " 2.加速度计校准", .Function = Acc_Cali},
		{.Name = " 3.陀螺仪校准", .Function = Gyro_Cali},
	}
};

struct Menu_List BluetoothMenu = {
	.Name = "   [蓝牙配置]    ",
	
	.WindowIndex = 0,
	.WindowLength = 3,
	
	.ItemIndex = 0,
	.ItemLength = 3,
	.Items = {
		{.Name = " < 返回", .Function = Menu_Back},
		{.Name = " 1.重置蓝牙参数", .Function = Bluetooth_Reload},
		{.Name = " 2.设波特率9600", .Function = Bluetooth_Config9600},
	}
};

struct Menu_List *ListStack[32];
int16_t pListStack = 0;

void Menu_Display(void)
{
	OLED_Clear();
	
	struct Menu_List *pList = ListStack[pListStack];
	
	OLED_Printf(0, 0, OLED_8X16, "%s", pList->Name);
	
	for (uint8_t i = 0; i < pList->WindowLength; i ++)
	{
		int16_t j = pList->WindowIndex + i;
		
		struct Menu_Item Item = pList->Items[j];
		
		OLED_Printf(0, 16 * i + 16, OLED_8X16, "%s", Item.Name);
	}
	
	int16_t Cursor = pList->ItemIndex - pList->WindowIndex;
	
	OLED_ReverseArea(0, 16 * Cursor + 16, 128, 16);
		
	OLED_Update();
}

void Menu_Up(void)
{
	struct Menu_List *pList = ListStack[pListStack];
	
	pList->ItemIndex --;
	
	if (pList->ItemIndex < 0)
	{
		pList->ItemIndex = pList->ItemLength - 1;
		pList->WindowIndex = pList->ItemLength - pList->WindowLength;
	}
	else
	{
		int16_t Cursor = pList->ItemIndex - pList->WindowIndex;
		if (Cursor < 0)
		{
			pList->WindowIndex --;
		}
	}
	
}

void Menu_Down(void)
{
	struct Menu_List *pList = ListStack[pListStack];
	
	pList->ItemIndex ++;
	
	if (pList->ItemIndex >= pList->ItemLength)
	{
		pList->ItemIndex = 0;
		pList->WindowIndex = 0;
	}
	else
	{
		int16_t Cursor = pList->ItemIndex - pList->WindowIndex;
		if (Cursor >= pList->WindowLength)
		{
			pList->WindowIndex ++;
		}
	}
}

void Menu_Enter(void)
{
	struct Menu_Item *Item = &ListStack[pListStack]->Items[ListStack[pListStack]->ItemIndex];
	
	if (Item->Function != 0)
	{
		Item->Function();
	}
	if (Item->NextList != 0)
	{
		pListStack ++;
		ListStack[pListStack] = Item->NextList;
	}
}

void Menu_Back(void)
{
	void (*Function)(void);
	Function = ListStack[pListStack]->BackFunction;
	
	if (Function != 0)
	{
		Function();
	}
	if (pListStack > 0)
	{
		pListStack --;
	}
}
