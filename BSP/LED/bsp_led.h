/*
 * led.h
 *
 *  Created on: 2022��7��7��
 *      Author: Lenovo
 */

#ifndef BSP_LED_LED_H_
#define BSP_LED_LED_H_

#include "F28x_Project.h"

void GPIO_TogglePin(Uint16 gpioNumber);
void bsp_led_init(void);

#define LED2            89//控制板上
#define LED3            90//控制板上

#define LED3C          24//底板上
#define LED4C          25//底板上
#define LED5C          23//底板上


#define LED2_ON()         GPIO_WritePin(LED2, 0)
#define LED3_ON()         GPIO_WritePin(LED3, 0)
#define LED3C_ON()         GPIO_WritePin(LED3C, 0)
#define LED4C_ON()         GPIO_WritePin(LED4C, 0)
#define LED5C_ON()         GPIO_WritePin(LED5C, 0)

#define LED2_OFF()         GPIO_WritePin(LED2, 1)
#define LED3_OFF()         GPIO_WritePin(LED3, 1)
#define LED3C_OFF()         GPIO_WritePin(LED3C, 1)
#define LED4C_OFF()         GPIO_WritePin(LED4C, 1)
#define LED5C_OFF()         GPIO_WritePin(LED5C, 1)

#define LED2_TOGGLE()      GPIO_TogglePin(LED2)
#define LED3_TOGGLE()      GPIO_TogglePin(LED3)
#define LED3C_TOGGLE()      GPIO_TogglePin(LED3C)
#define LED4C_TOGGLE()      GPIO_TogglePin(LED4C)
#define LED5C_TOGGLE()      GPIO_TogglePin(LED5C)


#endif /* BSP_LED_LED_H_ */
