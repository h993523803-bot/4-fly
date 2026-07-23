#include <stdint.h>
#include <math.h>

/**
  * 函    数：移相器（延迟器）
  * 参    数：Buffer 缓存数组
  * 参    数：N 数据点个数（数组长度）
  * 参    数：Input 输入数据
  * 返 回 值：输出数据
  * 说    明：外部提供一个静态/全局数组，每次调用此函数，输入数据推入数组末尾，输出数据是数组头部
  *           即，输出数据是输入数据延迟N次的值
  */
float Shifter(float *Buffer, uint8_t N, float Input)
{
    uint8_t i;
    for (i = 0; i < N - 1; i++)
    {
        Buffer[i] = Buffer[i + 1];	//数组整体移动
    }
    Buffer[N - 1] = Input;			//输入数据推入数组末尾
	return Buffer[0];				//输出数据是数组头部
}

/**
  * 函    数：均值滤波器
  * 参    数：Buffer 缓存数组
  * 参    数：N 数据点个数（数组长度）
  * 参    数：Input 输入数据
  * 返 回 值：输出数据
  * 说    明：外部提供一个静态/全局数组，每次调用此函数，输入数据推入数组末尾
  *           即，缓存数组中的数据是最近N次的输入数据
  *           输出数据是最近N次输入数据的平均值
  */
float AverageFilter(float *Buffer, uint8_t N, float Input)
{
	uint8_t i;
	float Sum = 0;
    for (i = 0; i < N - 1; i++)
    {
        Buffer[i] = Buffer[i + 1];	//数组整体移动
		Sum += Buffer[i];			//求和
    }
    Buffer[N - 1] = Input;			//输入数据推入数组末尾
	Sum += Buffer[N - 1];			//求和
	
	return Sum / N;					//返回平均值
}

/**
  * 函    数：中值滤波器
  * 参    数：Buffer 缓存数组
  * 参    数：N 数据点个数（数组长度）
  * 参    数：Input 输入数据
  * 返 回 值：输出数据
  * 说    明：外部提供一个静态/全局数组，每次调用此函数，输入数据推入数组末尾
  *           即，缓存数组中的数据是最近N次的输入数据
  *           随后对缓存数组进行排序，输出数据是最近N次输入数据的中位数
  */
float MedianFilter(float *Buffer, uint8_t N, float Input)
{
    uint8_t i, j;
    for (i = 0; i < N - 1; i++)
    {
        Buffer[i] = Buffer[i + 1];	//数组整体移动
    }
    Buffer[N - 1] = Input;			//输入数据推入数组末尾

    /*复制到临时缓冲区，避免破坏原始数据*/
    float TempBuffer[N];
    for (i = 0; i < N; i++)
    {
        TempBuffer[i] = Buffer[i];
    }

    float Temp;
    /*冒泡排序*/
    for (i = 0; i < N - 1; i++)
    {
        for (j = 0; j < N - i - 1; j++)
        {
            if (TempBuffer[j] > TempBuffer[j + 1])
            {
                Temp = TempBuffer[j];
                TempBuffer[j] = TempBuffer[j + 1];
                TempBuffer[j + 1] = Temp;
            }
        }
    }

    /*返回中值*/
    return TempBuffer[N / 2];
}
