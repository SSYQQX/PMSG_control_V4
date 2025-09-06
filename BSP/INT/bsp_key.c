/*
 * bsp_key.c
 *
 *  Created on: 2024年12月2日
 *      Author: XUQQ
 */
#include "bsp_key.h"

interrupt void xint1_isr(void)///KEY1 关
{
//    GpioDataRegs.GPBCLEAR.all = 0x4;   // GPIO34 is low
//    Xint1Count++;

    //
    // Acknowledge this interrupt to get more from group 1
    //
    if(GpioDataRegs.GPADAT.bit.GPIO26==1)
    {
        DELAY_US(50);
        if(GpioDataRegs.GPADAT.bit.GPIO26==1)
        {
            while(GpioDataRegs.GPADAT.bit.GPIO26==1);

            Turn_on_off=0;//电容断开
            generation_again_en=0;//允许再次发电使能。
            En_Torque_detec=0;//允许转矩检测

            RELAY_1_OFF();//关继电器1
            DELAY_US(100*1000);//100ms
            RELAY_2_OFF();//关继电器2
        }

    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// xint2_isr - External Interrupt 2 ISR
//
interrupt void xint2_isr(void)///KEY2 开
{
    static int clear_failure_flag = 0;
    if(GpioDataRegs.GPADAT.bit.GPIO27==1)
    {
        DELAY_US(50);
        if(GpioDataRegs.GPADAT.bit.GPIO27==1)
        {
            while(GpioDataRegs.GPADAT.bit.GPIO27==1);

            if(Failure_flag==1)//如果有故障信号，则先清除故障信号。请先确保清除了故障源
            {
                clear_failure_flag++;
                if(clear_failure_flag==2)//按两次才能清除故障
                {
                Failure_flag=0;
                clear_failure_flag=0;
                }
            }
            else    //否则直接开启
            {
            Turn_on_off=1;//电容接入

            generation_again_en=1;//允许再次发电使能。
            En_Torque_detec=1;//允许转矩检测
            }
        }

    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void key_Init(void)
{
    //按键中断配置
        // GPIO26 and GPIO27 are inputs，作为中断触发源
        //
           EALLOW;
           GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;         // GPIO，功能复用选择
           GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;          // input
           GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 0;        // XINT1 Synch to SYSCLKOUT only

           GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;         // GPIO
           GpioCtrlRegs.GPADIR.bit.GPIO27 = 0;          // input
           GpioCtrlRegs.GPAQSEL2.bit.GPIO27 = 2;        // XINT2 Qual using 6 samples
           GpioCtrlRegs.GPACTRL.bit.QUALPRD0 = 0xFF;   // Each sampling window
                                                       // is 510*SYSCLKOUT
           EDIS;
           // GPIO26 is XINT1, GPIO27 is XINT2
           //复用引脚为中断
              GPIO_SetupXINT1Gpio(26);
              GPIO_SetupXINT2Gpio(27);
        // Configure XINT1
        //上升沿触发
          XintRegs.XINT1CR.bit.POLARITY = 1;          //0 Falling edge interrupt.1 Rising edge interrupt
          XintRegs.XINT2CR.bit.POLARITY = 1;          // Falling edge interrupt
          //// Enable XINT1 and XINT2
                     //
        XintRegs.XINT1CR.bit.ENABLE = 1;            // Enable XINT1
        XintRegs.XINT2CR.bit.ENABLE = 1;            // Enable XINT2

        //配置中断函数地址
            EALLOW; // This is needed to write to EALLOW protected registers
            PieVectTable.XINT1_INT = &xint1_isr;
            PieVectTable.XINT2_INT = &xint2_isr;
            EDIS;   // This is needed to disable write to EALLOW protected registers
            //开CPU中断
            IER |= M_INT1;//开中断1,中断1.INT4、1.INT5在其中，即按键中断。
            //开PIE中断
            PieCtrlRegs.PIEIER1.bit.INTx4 = 1;          // Enable PIE Group 1 INT4，外部中断1，XINT1
            PieCtrlRegs.PIEIER1.bit.INTx5 = 1;          // Enable PIE Group 1 INT5，外部中断2,XINT2

}

