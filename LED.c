#include "stm32f10x.h"                  // Device header
#include "LED.h"

uint8_t LED_FlashFlag[5];

void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7);
}

void LED_SetMode(uint8_t n, uint8_t Mode)
{
	if (Mode == LED_FLASH)
	{
		LED_FlashFlag[n] = 1;
	}
	else
	{
		LED_FlashFlag[n] = 0;
		LED_Set(n, Mode);
	}
}

void LED_Set(uint8_t n, uint8_t ON_OFF)
{
	if (n == 1)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, (BitAction)(!ON_OFF));
	}
	else if (n == 2)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, (BitAction)(!ON_OFF));
	}
	else if (n == 3)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_6, (BitAction)(!ON_OFF));
	}
	else if (n == 4)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_7, (BitAction)(!ON_OFF));
	}
}

void LED_Tick(void)
{
	static uint16_t Count;
	
	Count ++;
	Count %= 500;
	
	for (uint8_t i = 1; i <= 4; i ++)
	{
		if (LED_FlashFlag[i])
		{
			if (Count < 250)
			{
				LED_Set(i, LED_ON);
			}
			else
			{
				LED_Set(i, LED_OFF);
			}
		}
			
	}
}
