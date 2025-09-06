/*
 * bsp_key.h
 *
 *  Created on: 2024年12月2日
 *      Author: XUQQ
 */

#ifndef BSP_INT_BSP_KEY_H_
#define BSP_INT_BSP_KEY_H_

#include "F28x_Project.h"
#include "bsp_relay.h"

//系统上电、下电控制
extern int Turn_on_off;//上电，下电标志

//按键中断初始化。Gpio初始化
void key_Init(void);
//按键中断
interrupt void xint1_isr(void);
interrupt void xint2_isr(void);

extern Uint16 generation_again_en;   //允许再次发电标志
extern Uint16 generation_again_en;//允许再次发电使能。
extern Uint16 En_Torque_detec;//允许转矩检测
extern Uint16 Failure_flag;//故障标志

#endif /* BSP_INT_BSP_KEY_H_ */
