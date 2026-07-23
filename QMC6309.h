#ifndef __QMC6309_H
#define __QMC6309_H

void QMC6309_Init(void);
uint8_t QMC6309_GetID(void);
void QMC6309_GetData(int16_t *MagX, int16_t *MagY, int16_t *MagZ);

#endif
