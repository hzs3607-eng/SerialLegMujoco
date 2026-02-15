/**
 * @file math_process.h
 * @author gjc
 * @brief 
 * @version 0.1
 * @date 2023-09-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef  _MATH_PROCESS_H_
#define  _MATH_PROCESS_H_
#ifndef PI
  #define PI               3.14159265358979f
#endif
/* Includes ------------------------------------------------------------------*/
#include <cstdint>
#include <cstring>
#include "math.h"
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
uint16_t float_to_uint(float x, float x_min, float x_max, uint16_t bits);//浮点数转化为整型数
float uint_to_float(uint16_t x, float x_min, float x_max, uint16_t bits);//整型数转化为浮点数
float degree2rad(float x);//角度转化为弧度
float limit_range_function(float x,float x_min,float x_max);//限制范围函数
float rad2degree(float x);//弧度转化为角度
float max_function(float x,float y);//最大值函数
float min_function(float x,float y);//最小值函数
float one_low_pass_function(float RC,float value_now,float value_last);//一阶低通滤波函数
void move_front(float arr[],uint16_t length);//数组前移一位
void Math_Endian_Reverse_32(uint8_t* x);//32位浮点数大小端转换
uint8_t Math_Sum_8(uint8_t* buf,uint16_t data_length);//求和函数
void gaussJordan(float a[25][50], uint16_t n);//高斯约当消元法
float bytes_to_float(uint8_t *bytes);//字节数组转化浮点数
float Math_Abs(float x);//绝对值函数
#endif  

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/