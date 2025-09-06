/*
 * bsp_gpio.c
 *
 *  Created on: 2024年12月3日
 *      Author: XUQQ
 */
#include "bsp_gpio.h"
Uint16 CHARGE_FLAG=0;
Uint16 VCC5_EN_FLAG=0;
void Battery_Charge_GpioInit(void)
{
    // 1. 配置GPIO9为GPIO功能（不是复用外设功能）
       GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);  // Mux位置为0表示GPIO功能

       // 2. 配置GPIO9为输出模式，推挽模式（默认无特殊要求时选择推挽）
       GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);

       // 3. 设置GPIO9的初始状态（默认禁用外部芯片）
       GPIO_WritePin(9, 0);  // 输出低电平，禁用芯片（根据需求调整为1或0）
}

void Battery_Charge_EN(Uint16 charge_flag)
{
    //1 开 0 关
    GPIO_WritePin(9, charge_flag);
}

void VCC5_Enable_GpioInit(void)
{
    // 1. 配置GPIO8为GPIO功能（不是复用外设功能）
       GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);  // Mux位置为0表示GPIO功能

       // 2. 配置GPIO8为输出模式，推挽模式（默认无特殊要求时选择推挽）
       GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_PUSHPULL);

       // 3. 设置GPIO8的初始状态（默认禁用外部芯片）
       GPIO_WritePin(8, 0);  // 输出低电平，禁用芯片（根据需求调整为1或0）
}

void VCC5_Enable(Uint16 en_flag)
{
    //1 开 0 关
    GPIO_WritePin(8, en_flag);
}

