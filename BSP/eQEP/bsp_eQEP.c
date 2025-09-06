/*
 * bsp_eQEP.c
 *
 *  Created on: 2024年12月2日
 *      Author: XUQQ
 */
#include "bsp_eQEP.h"

float32 SPEED_cal_K=0.0001*1000*60;//转速计算系数=1/10000*编码器采样频率*60秒。
int motorspeed_rpm=0;//传递转速到串口
//
//struct Motor_Para
//{
//    int DirectionQep;           //电机旋转方向
//    float32 mech_position; //实时机械位置
//    int Speed_N;
//    int Speed_H;
//    int Speed_L;
//    unsigned int long Now_position;         //变量：当前位置
//    unsigned int long Last_position;           //变量：上一次位置
//    float32 mech_scaler;    //一转的总脉冲数的倒数  1/10000
//    float32 theta_mech;
//    float32 theta_elec;
//    float32 w_elec;//电角速度
//    int pole_pairs;//极对数
//}motor;

// 定义 motor 变量
Motor_Para motor;

//编码器计算
void POSSPEED_Calc()
{
//    float32 temp_CapturePeriod=0;//捕获周期临时寄存器
    // QPOSCNT
     motor.mech_position=EQep1Regs.QPOSCNT;//读取当前位置
     motor.DirectionQep=EQep1Regs.QEPSTS.bit.QDF;//旋转方向
     //机械角度与电角度采样频率与控制频率相同，20K
     motor.theta_mech=motor.mech_position*motor.mech_scaler*2*PI;//机械角度
     motor.theta_elec=motor.theta_mech*motor.pole_pairs;//旋转角度，即电角度

//     SPEED_N=30*(motor.theta_mech-theta_mech_last)/(PI*5e-5);
//     theta_mech_last=motor.theta_mech;
     // Check an index occurrence
     ////索引事件锁存中断，即Z信号。发生时将位置计数器锁存到QPOSILAT寄存器。注意不是QPOSLAT
     if(EQep1Regs.QFLG.bit.IEL == 1)
     {
     //   p->index_sync_flag = 0x00F0;
        EQep1Regs.QCLR.bit.IEL = 1;    // Clear __interrupt flag
     }
     //转速采样频率为100Hz
     //高速计算
     //单位计时器中断
     if(EQep1Regs.QFLG.bit.UTO == 1)
     {
         motor.Now_position=EQep1Regs.QPOSLAT;//进入中断时的脉冲计数值，由QPOSCNT锁存进来
        if (motor.DirectionQep)    //正向旋转，增计数
        {
           if(motor.Last_position>motor.Now_position)
               motor.Speed_H=(10000-motor.Last_position+motor.Now_position)*SPEED_cal_K;    //0.01根据UTO的时间周期计算,0.6=1/10000*100*60
           else
               motor.Speed_H=(motor.Now_position-motor.Last_position)*SPEED_cal_K;     //0.01根据UTO的时间周期计算

        //   motor.w_elec=(float32)motor.Speed_H*PI*5.0/30.0;//电角速度计算
        }
        else
        {
           if(motor.Last_position>=motor.Now_position)
               motor.Speed_H=-(int)(motor.Last_position-motor.Now_position)*SPEED_cal_K; //0.01根据UTO的时间周期计算
           else
               motor.Speed_H=-(int)(10000-motor.Now_position+motor.Last_position)*SPEED_cal_K;    //0.01根据UTO的时间周期计算
        //   motor.w_elec=(float32)motor.Speed_H*PI*5.0/30.0;//电角速度计算
        }
        motor.Last_position=motor.Now_position;
        EQep1Regs.QCLR.bit.UTO=1;                   //清除中断标志
     }


//     //低速计算，与UTO无关
//     if(EQep1Regs.QEPSTS.bit.UPEVNT == 1)               // Unit position event，单位位置事件中断，
//     {
//         if(EQep1Regs.QEPSTS.bit.COEF == 0)             // No Capture overflow，捕获定时器溢出标志
//         {
//             temp_CapturePeriod = (unsigned long)EQep1Regs.QCPRDLAT; // temp1 = t2-t1，捕获周期值
//         }
//         else   // Capture overflow, saturate the result
//         {
//             temp_CapturePeriod = 0xFFFF;//计时器溢出，赋值给满 65535
//         }
//         motor.Speed_L=600000/temp_CapturePeriod;//转化为rpm
//         if(motor.Speed_L>6000) motor.Speed_L=6000;//当中断频率为100hZ时。最大转速/10000*100*60
//         if(motor.DirectionQep)
//         {
//             motor.Speed_L=motor.Speed_L;
//         }
//         else
//         {
//             motor.Speed_L=-motor.Speed_L;
//         }
//         EQep1Regs.QEPSTS.all = 0x88; // Clear Unit position event flag
//                                      // Clear overflow error flag
//     }
if(motor.Speed_H>5000.0) motor.Speed_H=5000.0;
if(motor.Speed_H<-5000.0) motor.Speed_H=-5000.0;

     motor.Speed_N=motor.Speed_H;
     motorspeed_rpm=motor.Speed_N;
     motor.w_elec=(float32)motor.Speed_N*PI*5.0/30.0;//电角速度计算
}




void Init_Variables(void)
{
    motor.Now_position=0;
    motor.Last_position=0;
    motor.mech_position=0;
    motor.Speed_N=0;
    motor.Speed_H=0;
    motor.Speed_L=0;
    motor.mech_scaler=1.0/10000.0;
    motor.theta_mech=0.0;
    motor.theta_elec=0.0;
    motor.w_elec=0;
    motor.pole_pairs=5;
}
void Init_EQEP1_Gpio()
{
    EALLOW;
    //    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;// 开启GPIO时钟
   // CpuSysRegs.PCLKCR1.bit.EQEP1ENCLK = 1;  // eQEP1
    CpuSysRegs.PCLKCR4.bit.EQEP1 = 1;  // eQEP1

     GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0; // Enable pull-up on GPIO50 (EQEP1A)
     GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0; // Enable pull-up on GPIO51 (EQEP1B)
     GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0; // Enable pull-up on GPIO53 (EQEP1I)

     GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 0; // Sync to SYSCLKOUT GPIO50 (EQEP1A)
     GpioCtrlRegs.GPBQSEL2.bit.GPIO51 = 0; // Sync to SYSCLKOUT GPIO51 (EQEP1B)
     GpioCtrlRegs.GPBQSEL2.bit.GPIO53 = 0; // Sync to SYSCLKOUT GPIO53 (EQEP1I)

     GpioCtrlRegs.GPBMUX2.bit.GPIO50=1; //QEPA
     GpioCtrlRegs.GPBMUX2.bit.GPIO51=1; //QEPB
     GpioCtrlRegs.GPBMUX2.bit.GPIO53=1; //QEPI

     EDIS;
}


//编码器配置
void Init_EQEP1()
{
    //停止EQEP模块
    EQep1Regs.QEPCTL.bit.QPEN = 0;
    //清除计数器和中断标志
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QCLR.all = 0xFFFF;
    //配置输入属性
    EQep1Regs.QDECCTL.bit.QSRC=0; //设定eQep的计数模式为正交模式
    EQep1Regs.QDECCTL.bit.SWAP=0; //QEPA和QEPB信号不交换
    EQep1Regs.QDECCTL.bit.QAP=0;  //QEPA信号不取反
    EQep1Regs.QDECCTL.bit.QBP=0;  //QEPB信号不取反
    EQep1Regs.QDECCTL.bit.QIP=0;  //QIP信号不取反
    EQep1Regs.QEPCTL.bit.FREE_SOFT=2;//仿真控制位：位置计数器不受影响
    //配置位置计数器运行模式、初始化方式、最大值
    EQep1Regs.QEPCTL.bit.PCRM=00; //设定PCRM=00，即QPOSCNT在每次Index脉冲都复位
    //正转复位到0，反转复位到QPOSMAX。
    EQep1Regs.QEPCTL.bit.IEI=2;     //在QEPI上升沿初始化位置计数器
    EQep1Regs.QPOSMAX =9999;
   //EQep1Regs.QPOSMAX = 0xFFFFFFFF;
    //QEP捕捉锁存模式设置为单位时间事件发生时将QPOSCNT的值锁存到QPOSLAT中
    EQep1Regs.QEPCTL.bit.QCLM=1;
    //配置UTE单元时间、中断使能、使能
    EQep1Regs.QUPRD=200000; //分频数。当SYSCLKOUT=200MHz时，2e6设定Unit Timer溢出频率为100Hz,2e5为1000hz；此时为1000hz
    EQep1Regs.QEINT.bit.UTO=1;//使能UTO中断
    EQep1Regs.QEPCTL.bit.UTE=1;   //使能UTE,使能单位定时器
//    QFRC
//    EQep1Regs.QFRC.bit.UTO=1;//QEP的中断源

    //若要用QCAP单元进行精确的速度测量，可以配置QCAP单元  一般用于低速检测
    EQep1Regs.QCAPCTL.bit.UPPS = 5;    // 1/32 预分频器//单位位置事件预分频器  n个边沿脉冲为一个事件
    EQep1Regs.QCAPCTL.bit.CCPS = 6;    // 1/64 预分频器//捕获时钟预分频  sysclk/n
    EQep1Regs.QCAPCTL.bit.CEN=1;  //使能eQEP的捕获功能
    //开启EQEP模块，并用软件初始化位置计数器
    EQep1Regs.QEPCTL.bit.SWI = 1;
    EQep1Regs.QEPCTL.bit.QPEN = 1;
}
//

