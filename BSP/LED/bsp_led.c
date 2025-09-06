/*
 * led.c
 *
 *  Created on: 2022��7��7��
 *      Author: Lenovo
 */

#include "bsp_led.h"




void bsp_led_init(void)
{
//    GPIO_SetupPinMux(RUN_LED, GPIO_MUX_CPU1, 0x0);
//	GPIO_SetupPinOptions(RUN_LED, GPIO_OUTPUT, GPIO_PUSHPULL);
//
//    GPIO_SetupPinMux(FLT_LED, GPIO_MUX_CPU1, 0x0);
//	GPIO_SetupPinOptions(FLT_LED, GPIO_OUTPUT, GPIO_PUSHPULL);
//
//    GPIO_SetupPinMux(DCBUS_LED, GPIO_MUX_CPU1, 0x0);
//	GPIO_SetupPinOptions(DCBUS_LED, GPIO_OUTPUT, GPIO_PUSHPULL);
//控制板上LED
    GPIO_SetupPinMux(LED2, GPIO_MUX_CPU1, 0x0);
	GPIO_SetupPinOptions(LED2, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(LED3, GPIO_MUX_CPU1, 0x0);
	GPIO_SetupPinOptions(LED3, GPIO_OUTPUT, GPIO_PUSHPULL);

//底板上LED
    GPIO_SetupPinMux(LED3C, GPIO_MUX_CPU1, 0x0);
    GPIO_SetupPinOptions(LED3C, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(LED4C, GPIO_MUX_CPU1, 0x0);
    GPIO_SetupPinOptions(LED4C, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(LED5C, GPIO_MUX_CPU1, 0x0);
    GPIO_SetupPinOptions(LED5C, GPIO_OUTPUT, GPIO_PUSHPULL);

	//PWM缓冲器使能GPIO
    GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 0x0);
    GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_WritePin(LED2, 1);
    GPIO_WritePin(LED3, 1);

    GPIO_WritePin(LED3C, 1);
    GPIO_WritePin(LED4C, 1);
    GPIO_WritePin(LED5C, 1);
}

//void GPIO_TogglePin(Uint16 gpioNumber)
//{
//    static Uint16 outVal_led2;
//    static Uint16 outVal_led3;
//    if(gpioNumber == LED2) {
//        GPIO_WritePin(gpioNumber, outVal_led2);
//        outVal_led2 = !outVal_led2;
//    }
//    if(gpioNumber == LED3) {
//        GPIO_WritePin(gpioNumber, outVal_led3);
//        outVal_led3 = !outVal_led3;
//    }
//}

//void GPIO_TogglePin(Uint16 gpio_num)
//{
//    if(gpio_num <= 31)
//    {
//        // GPIO0~31 属于 GPIOA 组
//        GpioDataRegs.GPATOGGLE.all = (1UL << gpio_num);
//    }
//    else if(gpio_num <= 63)
//    {
//        // GPIO32~63 属于 GPIOB 组
//        GpioDataRegs.GPBTOGGLE.all = (1UL << (gpio_num - 32));
//    }
//    else
//    {
//        // 暂不支持 GPIO64 以上（如需要，可继续扩展）
//    }
//}

void GPIO_TogglePin(Uint16 gpioNumber)
{
    volatile Uint32 *gpioDataReg;
    Uint32 pinMask;

    gpioDataReg = (volatile Uint32 *)&GpioDataRegs + (gpioNumber/32)*GPY_DATA_OFFSET;
    pinMask = 1UL << (gpioNumber % 32);

    gpioDataReg[GPYTOGGLE] = pinMask;
}

