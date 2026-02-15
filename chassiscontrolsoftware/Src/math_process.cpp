/**
 * @file math_process.cpp
 * @author gjc
 * @brief 
 * @version 0.1
 * @date 2023-09-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* Includes ------------------------------------------------------------------*/


#include "../Inc/math_process.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 浮点数转化为整型数 uint16_t --> float
 * @note  
 * 
 * @param x 输入数据
 * @param x_min 下限
 * @param x_max 上限
 * @param bits 数据位数
 * @return 
 */
uint16_t float_to_uint(float x, float x_min, float x_max, uint16_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief 整型数转化为浮点数 float --> uint16_t
 * @note  
 * 
 * @param x 输入数据
 * @param x_min 下限
 * @param x_max 上限
 * @param bits 数据位数
 * @return 
 */
float uint_to_float(uint16_t x, float x_min, float x_max, uint16_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x)*span/((float)((1<<bits)-1))+offset;
}


/**
 * @brief 限制范围函数
 * @note  
 * 
 * @param x 输入数据
 * @param x_min 下限
 * @param x_max 上限
 * @return 限定的输出值
 */
float limit_range_function(float x,float x_min,float x_max)
{
    if(x>x_max) return x_max;
    else if(x<x_min) return x_min;
    else return x;
}

/**
 * @brief 角度转化为弧度
 * @note  
 * 
 * @param x 输入数据
 * @return 限定的输出值
 */
float degree2rad(float x)
{
    return x/180.0f*3.1415926f;
}

/**
 * @brief 弧度转化为角度
 * @note  
 * 
 * @param x 输入数据
 * @return 限定的输出值
 */
float rad2degree(float x)
{
    return x*180.0f/3.1415926f;
}

/**
 * @brief 最大值输出函数
 * @note  
 * 
 * @param 
 * @return 
 */
float max_function(float x,float y)
{
    return x>y?x:y;
}

/**
 * @brief 最小值输出函数
 * @note  
 * 
 * @param 
 * @return 
 */
float min_function(float x,float y)
{
    return x<y?x:y;
}

/**
 * @brief 一阶低通滤波函数
 * @note  
 * 
 * @param 
 * @return 
 */
float one_low_pass_function(float RC,float value_now,float value_last)
{
    float value_return = (1-RC) * value_last + RC * value_now;
    return value_return;
}

/**
 * @brief 数组前移一位
 * @note  
 * 
 * @param 
 * @return 
 */
void move_front(float arr[],uint16_t length)
{
    for (uint16_t i = 0; i < length-1; i++)
	{
		arr[i] = arr[i + 1];
	}
    arr[length-1] = 0.0f;
}

/**
 * @brief 浮点数大小端转化函数
 * @note  
 * 
 * @param 
 * @return 
 */
void Math_Endian_Reverse_32(uint8_t* x)
{
    uint8_t* p = (uint8_t*)x;
    uint8_t temp = p[0];
    p[0] = p[3];
    p[3] = temp;
    temp = p[1];
    p[1] = p[2];
    p[2] = temp;

}

/**
 * @brief 求和函数
 * @note  
 * 
 * @param 
 * @return 
 */
uint8_t Math_Sum_8(uint8_t* buf,uint16_t data_length)
{
    uint8_t check_sum = 0;
    for(uint16_t i = 0;i < data_length;i++)
    {
        check_sum += buf[i]; 
    }
    return check_sum;
}

/**
 * @brief 高斯约旦消元法
 * @note  
 * 
 * @param 
 * @return 
 */
void gaussJordan(float a[25][50], uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        // 找到主元
        uint16_t max = i;
        for (uint16_t j = i + 1; j < n; j++)
            if (fabsf(a[j][i]) > fabsf(a[max][i]))
                max = j;

        // 交换最大行和当前行
        if (i != max) {
            for (uint16_t k = 0; k < 2 * n; k++) {
                float temp = a[i][k];
                a[i][k] = a[max][k];
                a[max][k] = temp;
            }
        }

        // 将主对角线元素设为1
        float div = a[i][i];
        for (uint16_t k = 0; k < 2 * n; k++)
            a[i][k] /= div;

        // 将当前列的其他元素设为0
        for (uint16_t j = 0; j < n; j++) {
            if (j != i) {
                float mult = a[j][i];
                for (uint16_t k = 0; k < 2 * n; k++)
                    a[j][k] -= a[i][k] * mult;
            }
        }
    }
}

/** 
 * @brief 字节数组转化浮点数
 * @note  
 * 
 * @param 
 * @return 
 */
float bytes_to_float(uint8_t *bytes)
{
    float value;
    memcpy(&value, bytes, sizeof(float)); // 安全转换，避免指针对齐问题
    return value;
}

/**
 * @brief 绝对值函数
 * @note  
 * 
 * @param x 输入数据
 * @return 绝对值
 */
float Math_Abs(float x)
{
    if(x<0.0f) return -x;
    else return x;
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
/**
 * @brief 
 * @note  
 * 
 * @param 
 * @return 
 */
