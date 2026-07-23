#ifndef __QMI8658_REG_H
#define __QMI8658_REG_H

/*通用寄存器*/
#define QMI8658_WHO_AM_I			0x00
#define QMI8658_REVISION_ID			0x01

/*设置和控制寄存器*/
#define QMI8658_CTRL1				0x02
#define QMI8658_CTRL2				0x03
#define QMI8658_CTRL3				0x04
//#define QMI8658_Reserved			0x05
#define QMI8658_CTRL5				0x06
//#define QMI8658_Reserved			0x07
#define QMI8658_CTRL7				0x08
#define QMI8658_CTRL8				0x09
#define QMI8658_CTRL9				0x0A

/*主机控制的校准寄存器*/
#define QMI8658_CAL1_L				0x0B 
#define QMI8658_CAL1_H				0x0C 
#define QMI8658_CAL2_L				0x0D 
#define QMI8658_CAL2_H				0x0E 
#define QMI8658_CAL3_L				0x0F 
#define QMI8658_CAL3_H				0x10
#define QMI8658_CAL4_L				0x11
#define QMI8658_CAL4_H				0x12

/*FIFO寄存器*/
#define QMI8658_FIFO_WTM_TH			0x13
#define QMI8658_FIFO_CTRL			0x14
#define QMI8658_FIFO_SMPL_CNT		0x15
#define QMI8658_FIFO_STATUS			0x16
#define QMI8658_FIFO_DATA			0x17

/*状态寄存器*/
#define QMI8658_STATUSINT			0x2D 
#define QMI8658_STATUS0				0x2E 
#define QMI8658_STATUS1				0x2F

/*时间戳寄存器*/
#define QMI8658_TIMESTAMP_LOW		0x30
#define QMI8658_TIMESTAMP_MID		0x31
#define QMI8658_TIMESTAMP_HIGH		0x32

/*数据输出寄存器*/
#define QMI8658_TEMP_L				0x33
#define QMI8658_TEMP_H				0x34
#define QMI8658_AX_L				0x35
#define QMI8658_AX_H				0x36
#define QMI8658_AY_L				0x37
#define QMI8658_AY_H				0x38
#define QMI8658_AZ_L				0x39
#define QMI8658_AZ_H				0x3A 
#define QMI8658_GX_L				0x3B 
#define QMI8658_GX_H				0x3C 
#define QMI8658_GY_L				0x3D 
#define QMI8658_GY_H				0x3E 
#define QMI8658_GZ_L				0x3F 
#define QMI8658_GZ_H				0x40

/*按需校正指示和通用寄存器*/
#define QMI8658_COD_STATUS			0x46
#define QMI8658_dQW_L				0x49
#define QMI8658_dQW_H				0x4A 
#define QMI8658_dQX_L				0x4B 
#define QMI8658_dQX_H				0x4C 
#define QMI8658_dQY_L				0x4D 
#define QMI8658_dQY_H				0x4E 
#define QMI8658_dQZ_L				0x4F 
#define QMI8658_dQZ_H				0x50
#define QMI8658_dVX_L				0x51
#define QMI8658_dVX_H				0x52
#define QMI8658_dVY_L				0x53
#define QMI8658_dVY_H				0x54
#define QMI8658_dVZ_L				0x55
#define QMI8658_dVZ_H				0x56

/*活动检测输出寄存器*/
#define QMI8658_TAP_STATUS			0x59

/*复位寄存器*/
#define QMI8658_RESET				0x60

#endif
