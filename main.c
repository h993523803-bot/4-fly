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

#include <math.h>
#include <string.h>
#include <stdlib.h>

/*运行状态*/
#define STATE_STOP			0		//停止状态，控制程序不工作，电机停转
#define STATE_READY			1		//就绪状态，按启动键后进入，电机暂时不转，上推左摇杆后切换到工作状态
#define STATE_RUNNING		2		//工作状态，控制程序工作

uint8_t RunState = STATE_STOP;		//默认停止状态

/*飞行模式*/
#define MODE_STAB			0		//自稳模式，手动控制油门，手动控制水平位置
#define MODE_ALT			1		//定高模式，自动控制油门（气压计定高），手动控制水平位置
#define MODE_POS			2		//定点模式，自动控制油门（激光定高），自动控制水平位置（光流定点）

uint8_t FlyMode = MODE_ALT;			//默认定高模式

/*姿态相关数据*/
uint8_t IMUFlag;					//姿态初始化标志位，置1后，自动用加速度计初始化一次姿态四元数
int16_t AX, AY, AZ, GX, GY, GZ;		//加速度计陀螺仪原始数据，坐标系为芯片自身坐标系
int16_t AXOffset, AYOffset, AZOffset, GXOffset, GYOffset, GZOffset;		//原始数据零偏，用于零偏校准
float ax, ay, az, gx, gy, gz;		//加速度计陀螺仪处理后的数据，坐标系为机体坐标系（前X，右Y，下Z）
float Roll, Pitch, Yaw;				//姿态解算后的欧拉角

/*定高和定点相关数据*/
uint8_t BaroFlag;					//气压计初始化标志位，置1后，将当前海拔高度设置为零偏，即认为当前相对高度为0
float BaroPress, BaroTemp;			//气压计原始数据
float BaroAlt;						//气压计海拔高度，单位：m
float BaroAltOffset;				//气压计海拔高度零偏，单位：m
float BaroHeight;					//气压计相对高度，单位：cm

float LaserHeight;					//激光测距得到的高度，单位：cm

float Height;						//高度，定高模式时等于BaroHeight，定点模式时等于LaserHeight，单位：cm

uint8_t RedefineHeightFlag;			//重定高度标志位，用于定高模式和定点模式切换时高度的平滑过渡

int16_t DeltaX, DeltaY;				//光流原始数据

float ForwardAcc, ForwardVel;		//向前的加速度，向前的速度
float ForwardAccCor;				//向前加速度的校正量

float RightAcc, RightVel;			//向右的加速度，向右的速度
float RightAccCor;					//向右加速度的校正量

float DownAcc, DownVel, DownPos;	//向下的加速度，向下的速度，向下的位置
float DownAccCor;					//向下加速度的校正量

/*PID相关数据*/
PID_t RollRatePID  = {.Kp = 0.15, .Ki = 0.10, .Kd = 0.00, .OutMax = 100, .OutMin = -100};	//横滚角速度PID
PID_t PitchRatePID = {.Kp = 0.15, .Ki = 0.10, .Kd = 0.00, .OutMax = 100, .OutMin = -100};	//俯仰角速度PID
PID_t YawRatePID   = {.Kp = 0.30, .Ki = 0.00, .Kd = 0.00, .OutMax = 100, .OutMin = -100};	//偏航角速度PID

PID_t RollPID  = {.Kp = 3.00, .Ki = 0.00, .Kd = 0.00, .OutMax = 1000, .OutMin = -1000};		//横滚角度PID
PID_t PitchPID = {.Kp = 3.00, .Ki = 0.00, .Kd = 0.00, .OutMax = 1000, .OutMin = -1000};		//俯仰角度PID
PID_t YawPID   = {.Kp = 6.00, .Ki = 0.00, .Kd = 0.00, .OutMax = 1000, .OutMin = -1000};		//偏航角度PID

PID_t ForwardRatePID = {.Kp = 0.20, .Ki = 0.05, .Kd = 0.00, .OutMax = 10, .OutMin = -10};	//向前速度PID
PID_t RightRatePID   = {.Kp = 0.20, .Ki = 0.05, .Kd = 0.00, .OutMax = 10, .OutMin = -10};	//向右速度PID
PID_t HeightRatePID  = {.Kp = 1.50, .Ki = 2.00, .Kd = 0.10, .OutMax = 100, .OutMin = -100};	//高度/向下速度PID

/*控制相关数据*/
int16_t PWM1, PWM2, PWM3, PWM4;		//4个电机的PWM占空比，范围：0~100
int16_t Throttle;					//油门，范围：0~100
int8_t LH, LV, RH, RV;				//摇杆值，范围：-100~100
float RollTrim, PitchTrim;			//角度微调，当四轴悬停偏飞时可以用遥控器适当调整此值

/*其他数据*/
float BatteryVoltage;				//电池电压

uint8_t ModuleExist;				//激光+光流模块是否存在
uint16_t RemoteWatchDog;			//遥控器断连监测，连续3秒未收到遥控数据，则自动降落
uint16_t LaserWatchDog;				//激光超过测距范围监测，连续3秒激光返回错误，则自动切换到定高模式

#define UNDER_VOLTAGE_THRESHOLD		2.8		//欠压自动降落的判定阈值
uint8_t UnderVoltageFlag;			//欠压标志位，电池电压低于判定阈值时置1

float Plot1, Plot2, Plot3, Plot4;	//绘图临时变量

uint8_t DebugFlag;					//调试标志位，进入调试模式时置1
uint16_t TimerCount;				//定时中断执行时间
uint16_t TimerCountMax;				//定时中断执行时间的最大值

/*加载掉电不丢失的参数*/
void LoadParam(void)
{
	/*以下参数修改后，会存入FLASH，掉电不丢失，在上电时会回读这些参数*/
	AXOffset = Store_Data[1];				//加速度计零偏
	AYOffset = Store_Data[2];
	AZOffset = Store_Data[3];
	GXOffset = Store_Data[4];				//陀螺仪零偏
	GYOffset = Store_Data[5];
	GZOffset = Store_Data[6];
	RollTrim = *(float *)&Store_Data[7];	//横滚微调
	PitchTrim = *(float *)&Store_Data[9];	//俯仰微调
}

int main(void)
{
	OLED_Init();		//OLED初始化
	
	OLED_Clear();		//显示程序信息
	OLED_ShowString(0, 0,  "   [江协科技]   ", OLED_8X16);
	OLED_ShowString(0, 16, " 四轴飞行器程序 ", OLED_8X16);
	OLED_ShowString(0, 32, "      V1.0      ", OLED_8X16);
	OLED_ShowString(0, 48, "                ", OLED_8X16);
	OLED_Update();
	
	Delay_ms(100);
	
	OLED_Clear();
	
	LED_Init();			//LED初始化
	Key_Init();			//按键初始化
	PWM_Init();			//PWM初始化，用于驱动4个电机
	AD_Init();			//AD初始化，用于读取电池电压
	NRF24L01_Init();	//NRF24L01初始化，用于遥控器通信
	Serial_Init(9600);	//串口初始化，用于蓝牙串口通信
	QMI8658_Init();		//QMI8658初始化，加速度计陀螺仪
	QMC6309_Init();		//QMC6309初始化，磁力计
						//磁力计使用较为繁琐，后续将在视频中演示使用方法，此程序暂时未使用磁力计数据
	SPA06_Init();		//SPA06初始化，气压计
	VL53L1X_Init();		//VL53L1X初始化，激光测距
	PAA3905_Init();		//PAA3905初始化，光流
	
	/*判断激光+光流模块是否存在*/
	if (VL53L1X_GetID() == 0xEACC && PAA3905_GetID() == 0xA2)
	{
		ModuleExist = 1;	//如果读取ID正确，则置标志位为1
	}
	
	Store_Init();		//FLASH存储初始化
	LoadParam();		//上电后加载FLASH中保存的参数
	
	IMUFlag = 1;		//初始化一次姿态四元数
	BaroFlag = 1;		//初始化气压高度，设置当前相对高度为0
	
	Timer_Init();		//定时器初始化，TIM1，定时中断1ms
	
	while (1)
	{
		/*电池电压*/
		static float BatteryVoltageBuffer[10]; 		//读取电池电压，进行均值滤波，减小数值波动
		BatteryVoltage = AverageFilter(BatteryVoltageBuffer, 10, AD_GetBatteryVoltage());
		
		/*状态指示*/
		/*如果当前是自稳模式，则LED1闪烁，否则，停止状态时LED1熄灭，就绪/工作状态时，LED1常亮*/
		LED_SetMode(1, FlyMode == MODE_STAB ? LED_FLASH : (RunState == STATE_STOP ? LED_OFF : LED_ON));
		/*如果当前是定高模式，则LED2闪烁，否则，停止状态时LED2熄灭，就绪/工作状态时，LED2常亮*/
		LED_SetMode(2, FlyMode == MODE_ALT  ? LED_FLASH : (RunState == STATE_STOP ? LED_OFF : LED_ON));
		/*如果当前是定点模式，则LED3闪烁，否则，停止状态时LED3熄灭，就绪/工作状态时，LED3常亮*/
		LED_SetMode(3, FlyMode == MODE_POS  ? LED_FLASH : (RunState == STATE_STOP ? LED_OFF : LED_ON));
		/*如果当前电池电量过低，则LED4闪烁，否则，停止状态时LED4熄灭，就绪/工作状态时，LED4常亮*/
		LED_SetMode(4, UnderVoltageFlag     ? LED_FLASH : (RunState == STATE_STOP ? LED_OFF : LED_ON));
		
		/*按键操作*/
		if (Key_Check(KEY_1, KEY_SINGLE))	//K1按键单击
		{
			//无操作
		}
		if (Key_Check(KEY_1, KEY_LONG))		//K1按键长按
		{
			/*进入调试模式*/
			DebugFlag = 1;					//调试标志位置1
			OLED_Clear();
			Key_Clear();
			LED_SetMode(1, LED_OFF);		//LED熄灭
			LED_SetMode(2, LED_OFF);
			LED_SetMode(3, LED_OFF);
			LED_SetMode(4, LED_OFF);
			PWM_SetCompare1(0);				//电机停转
			PWM_SetCompare2(0);
			PWM_SetCompare3(0);
			PWM_SetCompare4(0);
			
			DebugMode();	//执行调试模式函数，程序会阻塞在此函数中，直到调试模式退出
		}
		
		/*数据接收*/
		if (NRF24L01_Receive() == 1)
		{
			RemoteWatchDog = 3000;			//喂狗
			
			uint8_t ID = NRF24L01_RxPacket[0];		//遥控数据包第0个字节为ID
			if (ID == 0x10 || ID == 0x11)	//如果ID是0x10或0x11，则执行操作
			{
				if (ID == 0x11)				//如果ID是0x11，则向遥控器回传数据
				{
					NRF24L01_TxPacket[0] = 0x12;						//回传数据的ID是0x12
					NRF24L01_TxPacket[1] = FlyMode;						//飞行模式
					*(int16_t *)&NRF24L01_TxPacket[2] = Throttle;		//油门
					*(float *)&NRF24L01_TxPacket[4] = BatteryVoltage;	//电池电压
					*(float *)&NRF24L01_TxPacket[8] = Roll;				//横滚角
					*(float *)&NRF24L01_TxPacket[12] = Pitch;			//俯仰角
					*(float *)&NRF24L01_TxPacket[16] = Yaw;				//偏航角
					*(float *)&NRF24L01_TxPacket[20] = Height;			//高度
					
					NRF24L01_Send();		//发送回传数据
				}
				
				/*接收遥控器数据*/
				LH = NRF24L01_RxPacket[1];			//左摇杆横向
				LV = NRF24L01_RxPacket[2];			//左摇杆纵向
				RH = NRF24L01_RxPacket[3];			//右摇杆横向
				RV = NRF24L01_RxPacket[4];			//右摇杆纵向
				uint8_t KEY = NRF24L01_RxPacket[5];	//遥控器按键键码
				
				/*执行遥控操作*/
				if (UnderVoltageFlag == 0)			//如果当前电量充足
				{
					if (FlyMode == MODE_STAB)				//自稳模式下
					{
						Throttle = LV;						//左摇杆纵向控制油门
						RollPID.Target = RH / 5.0;			//右摇杆横向控制横滚PID目标值
						PitchPID.Target = -RV / 5.0;		//右摇杆纵向控制俯仰PID目标值
					}
					else if (FlyMode == MODE_ALT)			//定高模式下
					{
						HeightRatePID.Target = LV / 2.0;	//左摇杆纵向控制高度PID目标值
						RollPID.Target = RH / 5.0;			//右摇杆横向控制横滚PID目标值
						PitchPID.Target = -RV / 5.0;		//右摇杆纵向控制俯仰PID目标值
					}
					else if (FlyMode == MODE_POS)			//定点模式下
					{
						HeightRatePID.Target = LV / 2.0;	//左摇杆纵向控制高度PID目标值
						RightRatePID.Target = RH * 4;		//右摇杆横向控制向右速度PID目标值
						ForwardRatePID.Target = RV * 4;		//右摇杆纵向控制向前速度PID目标值
					}
				}
				else								//当前电量不足
				{
					if (FlyMode == MODE_STAB)				//自稳模式下
					{
						Throttle = LV;						//正常控制
						RollPID.Target = RH / 5.0;
						PitchPID.Target = -RV / 5.0;
					}
					else if (FlyMode == MODE_ALT)			//定高模式下
					{
						HeightRatePID.Target = -50;			//强制降落
						RollPID.Target = RH / 5.0;
						PitchPID.Target = -RV / 5.0;
					}
					else if (FlyMode == MODE_POS)			//定点模式下
					{
						HeightRatePID.Target = -50;			//强制降落
						RightRatePID.Target = RH * 4;
						ForwardRatePID.Target = RV * 4;
					}
				}
				
				/*先按K1，进入就绪状态，随后上推左摇杆起飞*/
				if (LV > 10 && RunState == STATE_READY)		//就绪状态下，上推了左摇杆
				{
					/*PID初始化*/
					PID_Init(&RollRatePID);
					PID_Init(&PitchRatePID);
					PID_Init(&YawRatePID);
					PID_Init(&RollPID);
					PID_Init(&PitchPID);
					PID_Init(&YawPID);
					PID_Init(&ForwardRatePID);
					PID_Init(&RightRatePID);
					PID_Init(&HeightRatePID);
					
					/*变量初始化*/
					DownAccCor = 0;
					DownVel = 0;
					DownPos = 0;
					
					YawPID.Target = 0;
					IMUFlag = 1;			//初始化一次姿态四元数
					BaroFlag = 1;			//初始化气压高度，设置当前相对高度为0
					
					RunState = STATE_RUNNING;		//进入工作状态
				}
				
				if (KEY == 1)		//遥控器K1按下
				{
					/*变量初始化*/
					HeightRatePID.Target = 0;
					UnderVoltageFlag = 0;
					
					RunState = STATE_READY;		//进入就绪状态
				}
				if (KEY == 2)		//遥控器K2按下
				{
					RunState = STATE_STOP;		//进入停止状态
				}
				if (KEY == 3)		//遥控器K3按下
				{
					/*飞行模式-*/
					if (FlyMode == MODE_POS)
					{
						FlyMode = MODE_ALT;
					}
					else if (FlyMode == MODE_ALT)
					{
						FlyMode = MODE_STAB;
					}
					
					/*重定高度，用于定高模式和定点模式切换时高度的平滑过渡*/
					RedefineHeightFlag = 1;
				}
				if (KEY == 4)		//遥控器K4按下
				{
					/*飞行模式+*/
					if (FlyMode == MODE_STAB)
					{
						FlyMode = MODE_ALT;
					}
					else if (FlyMode == MODE_ALT && ModuleExist == 1)
					{
						LaserWatchDog = 3000;
						FlyMode = MODE_POS;		//激光+光流模块存在时，才能进入定点模式
					}
					
					/*重定高度，用于定高模式和定点模式切换时高度的平滑过渡*/
					RedefineHeightFlag = 1;
				}
				if (KEY == 5)		//遥控器K5按下
				{
					/*向左微调*/
					PitchTrim += 0.5f;
					if (PitchTrim > 10.0f)
					{
						PitchTrim = 10.0f;
					}
					*(float *)&Store_Data[9] = PitchTrim;
					Store_Save();	//保存微调参数
				}
				if (KEY == 6)		//遥控器K6按下
				{
					/*向右微调*/
					PitchTrim -= 0.5f;
					if (PitchTrim < -10.0f)
					{
						PitchTrim = -10.0f;
					}
					*(float *)&Store_Data[9] = PitchTrim;
					Store_Save();	//保存微调参数
				}
				if (KEY == 7)		//遥控器K7按下
				{
					/*向前微调*/
					RollTrim += 0.5f;
					if (RollTrim < -10.0f)
					{
						RollTrim = -10.0f;
					}
					*(float *)&Store_Data[7] = RollTrim;
					Store_Save();	//保存微调参数
				}
				if (KEY == 8)		//遥控器K8按下
				{
					/*向后微调*/
					RollTrim -= 0.5f;
					if (RollTrim > 10.0f)
					{
						RollTrim = 10.0f;
					}
					*(float *)&Store_Data[7] = RollTrim;
					Store_Save();	//保存微调参数
				}
			}
		}
		
		/*异常处理*/
		if (RemoteWatchDog == 0)	//连续3秒未收到遥控数据，遥控器断连
		{
			if (FlyMode == MODE_STAB)
			{
				Throttle = 30;				//强制油门为30，降落
				RollPID.Target = 0;			//水平位置控制归零
				PitchPID.Target = 0;
			}
			else if (FlyMode == MODE_ALT)
			{
				HeightRatePID.Target = -50;	//强制降落
				RollPID.Target = 0;			//水平位置控制归零
				PitchPID.Target = 0;
			}
			else if (FlyMode == MODE_POS)
			{
				HeightRatePID.Target = -50;	//强制降落
				RightRatePID.Target = 0;	//水平位置控制归零
				ForwardRatePID.Target = 0;
			}
		}
		
		if (LaserWatchDog == 0)			//连续3秒激光返回错误，高度过高或激光失效
		{
			if (FlyMode == MODE_POS)	//如果是定点模式
			{
				FlyMode = MODE_ALT;		//则自动切换到定高模式
				RedefineHeightFlag = 1;
			}
		}
		
		/*OLED显示*/
		OLED_Clear();
		if (FlyMode == MODE_STAB)		//显示飞行模式
		{
			OLED_Printf(96, 0, OLED_8X16, "自稳");
		}
		else if (FlyMode == MODE_ALT)
		{
			OLED_Printf(96, 0, OLED_8X16, "定高");
		}
		else if (FlyMode == MODE_POS)
		{
			OLED_Printf(96, 0, OLED_8X16, "定点");
		}
		OLED_Printf(0, 0, OLED_8X16, "%4.2fv", BatteryVoltage);				//显示电池电压
		OLED_Printf(0, 16, OLED_8X16, "R:%+04.0f  P:%+04.0f", Roll, Pitch);	//显示横滚角，俯仰角
		OLED_Printf(0, 32, OLED_8X16, "Y:%+04.0f  T:%+04d", Yaw, Throttle);	//显示偏航角，油门
		OLED_Printf(0, 48, OLED_8X16, "H:%+06.0fcm ", Height);				//显示高度
		OLED_Update();
				
		/*串口打印*/
//		Serial_Printf("[plot,%f]\r\n", Plot1);				//调试时使用
//		Serial_Printf("[plot,%f,%f]\r\n", Plot1, Plot2);
//		Serial_Printf("[plot,%f,%f,%f]\r\n", Plot1, Plot2, Plot3);
//		Serial_Printf("[plot,%f,%f,%f,%f]\r\n", Plot1, Plot2, Plot3, Plot4);
	}
}

/*定时中断，1ms执行一次*/
void TIM1_UP_IRQHandler(void)
{
	static float a_forward, a_right, a_down;
	static uint16_t Count;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		
		LED_Tick();		//LED定时驱动
		Key_Tick();		//按键定时驱动
		
		/*调试模式判断*/
		if (DebugFlag)
		{
			DebugMode_Tick();		//调试定时驱动
			return;		//调试模式时不执行后续代码
		}
		
		/*遥控器断连监测*/
		/*RemoteWatchDog是一个递减计数器，每ms自减1*/
		/*收到遥控数据后置为3000（喂狗），递减到0时，说明遥控器断连*/
		if (RemoteWatchDog > 0)
		{
			RemoteWatchDog --;
		}
		
		/*激光超过测距范围监测*/
		/*LaserWatchDog是一个递减计数器，每ms自减1*/
		/*正确读取激光数据后置为3000（喂狗），递减到0时，说明激光失效*/
		if (FlyMode == MODE_POS)		//只有在定点模式下才执行
		{
			if (LaserWatchDog > 0)
			{
				LaserWatchDog --;
			}
		}
		
		/*电池欠压监测*/
		static uint16_t UnderVoltageCount;
		if (BatteryVoltage < UNDER_VOLTAGE_THRESHOLD)	//电池电压低于设定阈值
		{
			UnderVoltageCount ++;
			if (UnderVoltageCount > 1000)	//连续1000ms欠压，避免误判
			{
				UnderVoltageCount = 0;
				UnderVoltageFlag = 1;		//置欠压标志位为1
			}
		}
		else								//偶发欠压
		{
			UnderVoltageCount = 0;			//重置计数
		}
		
		/*中断计数器*/
		/*Count在每次进中断后自增，并在0~7之间，不断循环*/
		/*根据Count值，进入以下不同的if分支*/
		/*不同if分支会错开进入，避免单次中断时间过长*/
		Count ++;
		Count %= 8;
				
		if (Count % 2 == 0)					//Count为0/2/4/6时进入if，即2ms, 500Hz
		{
			/*读取加速度计陀螺仪原始数据*/
			QMI8658_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
			
			/*减去零偏*/
			AX -= AXOffset;
			AY -= AYOffset;
			AZ -= AZOffset;
			GX -= GXOffset;
			GY -= GYOffset;
			GZ -= GZOffset;
			
			/*坐标系转换，同时缩放数据*/
			/*传感器坐标的Y轴，为机体坐标系的X轴*/
			/*传感器坐标的X轴，为机体坐标系的Y轴*/
			/*传感器坐标的Z轴，为机体坐标系的Z轴*/
			ax = -AY / 32768.0f * 16.0f;		//机体坐标系下的ax，单位：g
			ay = -AX / 32768.0f * 16.0f;		//机体坐标系下的ay，单位：g
			az = AZ / 32768.0f * 16.0f;			//机体坐标系下的az，单位：g
			gx = GY / 32768.0f * 2048.0f;		//机体坐标系下的gx，单位：°/s
			gy = GX / 32768.0f * 2048.0f;		//机体坐标系下的gy，单位：°/s
			gz = -GZ / 32768.0f * 2048.0f;		//机体坐标系下的gz，单位：°/s
			
			/*姿态初始化*/
			if (IMUFlag)
			{
				IMUFlag = 0;
				IMU_InitQuaternion(ax, ay, az);		//用加速度计初始化一次姿态四元数
			}
			
			if (RunState == STATE_RUNNING)			//处于工作状态时
			{
				/*角速度PID控制（内环）*/
				RollRatePID.Actual = gx;			//横滚角速度实际值为gx
				PitchRatePID.Actual = gy;			//俯仰角速度实际值为gy
				YawRatePID.Actual = gz;				//偏航角速度实际值为gz
				
				PID_Update(&RollRatePID, 0.002f);	//横滚角速度PID计算
				PID_Update(&PitchRatePID, 0.002f);	//俯仰角速度PID计算
				PID_Update(&YawRatePID, 0.002f);	//偏航角速度PID计算
				
				/*PID前馈*/
				/*将外环的目标值乘一个系数后，直接作用到内环的输出值，可以加快系统响应速度*/
				RollRatePID.Out += RollPID.Target * 0.1f;
				PitchRatePID.Out += PitchPID.Target * 0.1f;
				
				/*油门限幅*/
				if (Throttle > 90) {Throttle = 90;}	//油门最大限制为90
				
				/*PID输出*/
				/*油门为四个PWM的基础，3个PID输出值在这个基础上进行调整，进而控制姿态*/
				/*RollRatePID.Out 调整PWM1、PWM3和PWM2、PWM4，来控制横滚*/
				/*PitchRatePID.Out调整PWM1、PWM2和PWM3、PWM4，来控制俯仰*/
				/*YawRatePID.Out  调整PWM1、PWM4和PWM2、PWM3，来控制偏航*/
				PWM1 = Throttle + RollRatePID.Out + PitchRatePID.Out + YawRatePID.Out;
				PWM2 = Throttle - RollRatePID.Out + PitchRatePID.Out - YawRatePID.Out;
				PWM3 = Throttle + RollRatePID.Out - PitchRatePID.Out - YawRatePID.Out;
				PWM4 = Throttle - RollRatePID.Out - PitchRatePID.Out + YawRatePID.Out;
				
				/*PWM限幅*/
				if (PWM1 < 0) {PWM1 = 0;} else if (PWM1 > 100) {PWM1 = 100;}
				if (PWM2 < 0) {PWM2 = 0;} else if (PWM2 > 100) {PWM2 = 100;}
				if (PWM3 < 0) {PWM3 = 0;} else if (PWM3 > 100) {PWM3 = 100;}
				if (PWM4 < 0) {PWM4 = 0;} else if (PWM4 > 100) {PWM4 = 100;}
				
				/*PWM分别输出到4个电机*/
				PWM_SetCompare1(PWM1);
				PWM_SetCompare2(PWM2);
				PWM_SetCompare3(PWM3);
				PWM_SetCompare4(PWM4);
			}
			else		//不处于工作状态时
			{
				/*PWM输出0，电机停转*/
				PWM_SetCompare1(0);
				PWM_SetCompare2(0);
				PWM_SetCompare3(0);
				PWM_SetCompare4(0);
			}
		}
		
		if (Count % 4 == 1)					//Count为1/5时进入if，即4ms, 250Hz
		{
			/*执行6轴姿态解析，得到欧拉角*/
			IMU_6Axis_Gyro_Acc(ax, ay, az, gx, gy, gz, &Roll, &Pitch, &Yaw, 0.004f);
			
			/*四轴角度超过-70°~70°的范围，倾斜过大，自动停止*/
			if (Roll < -70.0f || Roll > 70.0f || Pitch < -70.0f || Pitch > 70.0f)
			{
				RunState = STATE_STOP;		//切换到停止状态
			}
			
			/*降落自动停机判定*/
			static uint8_t PWM1ZeroCount, PWM2ZeroCount, PWM3ZeroCount, PWM4ZeroCount;
			if (PWM1 == 0) {if (PWM1ZeroCount < 100) PWM1ZeroCount ++;} else {PWM1ZeroCount = 0;}	//PWM为0时计数
			if (PWM2 == 0) {if (PWM2ZeroCount < 100) PWM2ZeroCount ++;} else {PWM2ZeroCount = 0;}
			if (PWM3 == 0) {if (PWM3ZeroCount < 100) PWM3ZeroCount ++;} else {PWM3ZeroCount = 0;}
			if (PWM4 == 0) {if (PWM4ZeroCount < 100) PWM4ZeroCount ++;} else {PWM4ZeroCount = 0;}
			
			/*左摇杆下拉，且四个PWM持续100次为0，则判定四轴已落地，自动停止*/
			if (HeightRatePID.Target < -20.0f 
				&& PWM1ZeroCount == 100 && PWM1ZeroCount == 100
				&& PWM1ZeroCount == 100 && PWM1ZeroCount == 100)
			{
				RunState = STATE_STOP;		//切换到停止状态
			}
			
			if (RunState == STATE_RUNNING)		//处于工作状态时
			{
				/*角度PID控制（外环）*/
				RollPID.Actual = Roll + RollTrim;		//横滚角度实际值为Roll，加上横滚微调
				PitchPID.Actual = Pitch + PitchTrim;	//俯仰角度实际值为Pitch，加上俯仰微调
				YawPID.Actual = Yaw;					//偏航角度实际值为Yaw
				
				/*偏航越界衔接*/
				/*因为偏航角度的范围是-180~180，而四轴可能连续偏航旋转，这会出现从-180到180的角度跳跃问题*/
				/*因此要进行以下操作，避免角度跳跃*/
				if (YawPID.Target - YawPID.Actual < -180.0f)	//当目标值和实际值之差小于-180时
				{
					YawPID.Actual -= 360.0f;					//实际值整体减去一圈
				}
				if (YawPID.Target - YawPID.Actual > 180.0f)		//当目标值和实际值之差大于180时
				{
					YawPID.Actual += 360.0f;					//实际值整体加上一圈
				}
				
				PID_Update(&RollPID, 0.004f);		//横滚角度PID计算
				PID_Update(&PitchPID, 0.004f);		//俯仰角度PID计算
				PID_Update(&YawPID, 0.004f);		//偏航角度PID计算
				
				/*外环的输出值赋值给内环的目标值，构成串级PID*/
				RollRatePID.Target = RollPID.Out;	//横滚角度PID输出值，给横滚角速度PID目标值
				PitchRatePID.Target = PitchPID.Out;	//俯仰角度PID输出值，给俯仰角速度PID目标值
				YawRatePID.Target = YawPID.Out;		//偏航角度PID输出值，给偏航角速度PID目标值
				
				/*偏航目标值累加*/
				YawPID.Target += LH / 100.0f * 180.0f * 0.004f;	//根据左摇杆横向值，控制偏航目标值累加，实现自转
				
				/*越界处理*/
				if (YawPID.Target > 180.0f) {YawPID.Target = -180.0f;}
				if (YawPID.Target < -180.0f) {YawPID.Target = 180.0f;}
			}
		}
		
		if (Count == 3)						//Count为3时进入if，即8ms, 125Hz
		{
			if (FlyMode == MODE_ALT || FlyMode == MODE_POS)		//定高模式或定点模式时执行
			{
				/*重复的计算放在前面统一执行*/
				float SinRoll = sinf(Roll * DEG_TO_RAD);
				float CosRoll = cosf(Roll * DEG_TO_RAD);
				float SinPitch = sinf(Pitch * DEG_TO_RAD);
				float CosPitch = cosf(Pitch * DEG_TO_RAD);
				
				/*根据加速度计和欧拉角，计算得到向前、向右和向下的加速度*/
				/*计算后加速度，不受四轴倾角影响*/
				a_forward = ax * CosPitch + ay * SinRoll * SinPitch + az * CosRoll * SinPitch;
				a_right = ay * CosRoll - az * SinRoll;
				a_down = -ax * SinPitch + ay * CosPitch * SinRoll + az * CosPitch * CosRoll;
				
				/*加速度数据处理，处理后的单位是cm/s^2*/
				a_forward = a_forward * 9.8f * 100.0f;
				a_right = a_right * 9.8f * 100.0f;
				a_down = (a_down - 1.0f) * 9.8f * 100.0f;
				
				if (FlyMode == MODE_ALT)		//处于定高模式时
				{
					/*读取气压计原始数据*/
					SPA06_GetData(&BaroPress, &BaroTemp);
					
					/*根据气压->海拔公式，得到当前气压对应的海拔高度，单位为m*/
					BaroAlt = 44330.0f * (1.0f - powf(BaroPress / 101325.0f, 1.0f / 5.255f));
					
					/*气压计初始化*/
					if (BaroFlag)
					{
						BaroAltOffset = BaroAlt;	//将当前海拔高度设置为零偏，即认为当前相对高度为0
						BaroFlag = 0;
					}
					
					if (BaroAltOffset > 0.0f)		//零偏大于0时才执行，避免气压高度未初始化
					{
						/*得到气压计的相对高度，单位转换为cm*/
						BaroHeight = (BaroAlt - BaroAltOffset) * 100.0f;
					}
					
					Height = BaroHeight;			//定高模式时高度是气压计高度
				}
				else if (FlyMode == MODE_POS)		//处于定点模式时
				{
					/*读取激光测距原始数据*/
					float LaserHeightTemp = VL53L1X_GetData();
					
					if (LaserHeightTemp != -1)		//如果正确读取了激光数据
					{
						LaserWatchDog = 3000;		//喂狗
						
						/*得到激光测距的高度，单位转换为cm*/
						LaserHeight =  LaserHeightTemp / 10.0f;
					}
					
					Height = LaserHeight;			//定点模式时高度是激光高度
				}
				
				/*重定高度*/
				if (RedefineHeightFlag)
				{
					RedefineHeightFlag = 0;
					DownPos = Height;				//当前向下的位置（高度），设置为传感器高度
				}
				
				/*使用3阶互补滤波，融合加速度计积分高度和传感器实测高度，得到稳定且响应迅速的高度*/
				float Error = Height - DownPos;		//传感器高度减去修正后的向下位置，为误差
				
				if (Error > 10.0f) {Error = 10.0f;}	//误差限幅
				else if (Error < -10.0f) {Error = -10.0f;}
				
				DownAccCor += 0.01f * Error;		//误差乘一个系数，累加后作为向下加速度的修正
				DownAcc = a_down + DownAccCor;		//修正加速度计的向下加速度
				
				DownVel += DownAcc * 0.008f;		//加速度累加，得到预测的向下速度
				DownVel += 0.02f * Error;			//误差乘一个系数，作为向下速度的修正
				
				DownPos += DownVel * 0.008f;		//速度累加，得到预测的向下位置
				DownPos += 0.04f * Error;			//误差乘一个系数，作为向下位置的修正
				
				if (RunState == STATE_RUNNING)		//处于工作状态时
				{
					/*高度速度（向下速度）PID控制*/
					HeightRatePID.Actual = DownVel;			//高度速度实际值为DownVel
					PID_Update(&HeightRatePID, 0.008f);		//高度速度PID计算
					Throttle = HeightRatePID.Out;			//高度速度PID输出值作用于油门，自动控制油门，定高
				}
			}
		}
		
		if (Count == 7)						//Count为7时进入if，即8ms, 125Hz
		{
			if (FlyMode == MODE_POS)		//处于定点模式时
			{
				/*读取光流原始数据*/
				PAA3905_GetData(&DeltaX, &DeltaY);
				
				/*光流原始数据中值滤波*/
				static float DeltaXBuffer[3], DeltaYBuffer[3];
				DeltaX = MedianFilter(DeltaXBuffer, 3, DeltaX);
				DeltaY = MedianFilter(DeltaYBuffer, 3, DeltaY);
				
				/*陀螺仪角速度移相*/
				/*因为光流数据慢于陀螺仪数据，所以要适当延迟陀螺仪数据以对齐光流数据*/
				#define DELAY_N				6
				static float gxBuffer[DELAY_N], gyBuffer[DELAY_N];
				float gxDelay = Shifter(gxBuffer, DELAY_N, gx);
				float gyDelay = Shifter(gyBuffer, DELAY_N, gy);
				
				/*使用陀螺仪角度上对光流速度进行旋转补偿*/
				/*旋转补偿后，可以一定程度上消除旋转引起的假光流*/
				float ForwardFlow = - DeltaX - gyDelay * 0.10f;
				float RightFlow = - DeltaY - (-gxDelay * 0.10f);
				
				/*对旋转补偿后的光流速度进行低通滤波*/
				static float ForwardFlowLPF, RightFlowLPF;
				ForwardFlowLPF += 0.1f * (ForwardFlow - ForwardFlowLPF);
				RightFlowLPF += 0.1f * (RightFlow - RightFlowLPF);
				
				/*对激光高度进行中值滤波和低通滤波*/
				static float LaserBuffer[3];
				float LaserHeightMF = MedianFilter(LaserBuffer, 3, LaserHeight);
				static float LaserHeightLPF;
				LaserHeightLPF += 0.1f * (LaserHeightMF - LaserHeightLPF);
				
				/*对旋转补偿后的光流速度进行高度补偿*/
				float ForwardFlowVel = ForwardFlowLPF * LaserHeightLPF;
				float RightFlowVel = RightFlowLPF * LaserHeightLPF;
								
				/*加速度积分得到的速度与补偿后的光流速度进行互补滤波融合*/
				float ForwardError = ForwardFlowVel - ForwardVel;		//光流向前速度减去修正后的向前速度，为误差
				
				ForwardAccCor += 0.01f * ForwardError;					//误差乘一个系数，累加后作为向前加速度的修正
				ForwardAcc = - a_forward * 2.5f + ForwardAccCor;		//修正加速度计的向前加速度
				
				ForwardVel += ForwardAcc * 0.008f;						//加速度累加，得到预测的向前速度
				ForwardVel += 0.02f * ForwardError;						//误差乘一个系数，作为向前速度的修正
				
				float RightError = RightFlowVel - RightVel;				//光流向右速度减去修正后的向右速度，为误差
				
				RightAccCor += 0.01f * RightError;						//误差乘一个系数，累加后作为向右加速度的修正
				RightAcc = - a_right * 2.5f + RightAccCor;				//修正加速度计的向右加速度
				
				RightVel += RightAcc * 0.008f;							//加速度累加，得到预测的向右速度
				RightVel += 0.02f * RightError;							//误差乘一个系数，作为向右速度的修正
				
				if (RunState == STATE_RUNNING)		//处于工作状态时
				{
					/*向前速度PID控制*/
					ForwardRatePID.Actual = ForwardVel;			//向前速度实际值为ForwardVel
					PID_Update(&ForwardRatePID, 0.008f);		//向前速度PID计算
					PitchPID.Target = -ForwardRatePID.Out;		//向前速度PID输出值作用于俯仰目标值
					
					/*向右速度PID控制*/
					RightRatePID.Actual = RightVel;				//向右速度实际值为RightVel
					PID_Update(&RightRatePID, 0.008f);			//向右速度PID计算
					RollPID.Target = RightRatePID.Out;			//向右速度PID输出值作用于横滚目标值
				}
			}
		}
		
		/*定时中断超时判断*/
		/*进中断后，立刻清除了Update标志位，如果中断退出前此标志位又置1了，说明中断执行时间过长，需要优化程序*/
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
			
			/*中断超时执行的操作，在调试时可以执行以下代码，方便观察中断是否超时*/
//			PWM_SetCompare1(0);			//电机停转
//			PWM_SetCompare2(0);
//			PWM_SetCompare3(0);
//			PWM_SetCompare4(0);
//			while (1)					//死循环
//			{
//				LED_Set(1, LED_ON);		//LED1闪烁
//				Delay_ms(100);
//				LED_Set(1, LED_OFF);
//				Delay_ms(100);
//			}
		}
		
		/*获取当前中断执行时间，和过往期间的最大值*/
		/*在调试时，可以显示TimerCount和TimerCountMax，了解中断执行时间*/
		TimerCount = TIM_GetCounter(TIM1);		//获取当前计数器的值
		if (TimerCount > TimerCountMax)			//如果大于最大值
		{
			TimerCountMax = TimerCount;			//则当前值是最大值
		}
	}
}
