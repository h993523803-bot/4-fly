#ifndef __LED_H
#define __LED_H

#define LED_OFF			0
#define LED_ON			1
#define LED_FLASH		2

void LED_Init(void);
void LED_SetMode(uint8_t n, uint8_t Mode);
void LED_Set(uint8_t n, uint8_t ON_OFF);
void LED_Tick(void);

#endif
