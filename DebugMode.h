#ifndef __DEBUG_MODE_H
#define __DEBUG_MODE_H

void DebugMode(void);
void DebugMode_Tick(void);

void DebugMode_Back(void);
void LED_Test(void);
void Motor_Test(void);
void AD_Test(void);
void NRF24L01_Test(void);
void Serial_Test(void);
void QMI8658_Test(void);
void QMC6309_Test(void);
void SPA06_Test(void);
void VL53L1X_Test(void);
void PAA3905_Test(void);

void Acc_Cali(void);
void Gyro_Cali(void);
void Reset_Cali(void);

void Bluetooth_Reload(void);
void Bluetooth_Config9600(void);

#endif
