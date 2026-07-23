#ifndef __QMI8658_H
#define __QMI8658_H

void QMI8658_Init(void);
uint8_t QMI8658_GetID(void);
void QMI8658_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#endif
