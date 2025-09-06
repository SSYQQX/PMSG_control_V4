//#include "device.h"
//永磁同步电机控制  200W发电机
#include "F28x_Project.h"
#include<stdlib.h>
#include "interrupt.h"
#include "bsp_emif.h"
#include "bsp_led.h"
#include "bsp_timer.h"
#include "bsp_epwm.h"
#include "bsp_eQEP.h"
#include "bsp_I2C.h"
#include "bsp_I2C.h"
#include "bsp_key.h"
#include "bsp_gpio.h"
#include "sysctl.h"
#include "emif.h"
#include "bsp_adc.h"
#include "bsp_relay.h"
#include "bsp_pid_ctrl.h"
#include "SPLL_1ph.h"
#include "Solar_F.h"
#include "spll_1ph_sogi_fll.h"

#include "F2837xD_Examples.h"
#include "485_ModobusRTU.h"

#define ld     103.85e-6
//#define lq      61e-6   表贴式Ld=Lq
#define lq     95.78e-6
/**
 * main.c
 */

Uint16 APhase_Current=0;
Uint16 BPhase_Current=0;
Uint16 CPhase_Current=0;
Uint16 flag=0;
Uint16 flag2=0;

Uint16 Bus_Voltage_AD=0;
Uint16 Bus_Current_AD=0;
Uint16 Supercapacitor_Voltage_AD=0;

float32 K_norm=0.05;


float32 Ua_Voltage=0;
float32 Ub_Voltage=0;
float32 Uc_Voltage=0;
float32 Ud_Voltage=0;
float32 Uq_Voltage=0;

float32 Ia_Current=0;
float32 Ib_Current=0;
float32 Ic_Current=0;
float32 Id_Current=0;
float32 Iq_Current=0;

float32 Ia_Current_norm=0;//归一化
float32 Ib_Current_norm=0;
float32 Ic_Current_norm=0;

float32 Bus_Voltage=0;
float32 Bus_Current=0;
float32 Bus_Current_Offset=0;
float32 Supercapacitor_Voltage=0;

//1ms滑窗
#define FILTER_SIZE 100

float32 bus_current_buf[FILTER_SIZE] = {0}; // 用于存储最近20个值
float32 bus_current_sum = 0;
Uint16 buf_index = 0;
float32 bus_current_avg = 0; // 滑动平均输出

//系统上电、下电控制
int Turn_on_off=0;//上电，下电标志
int state_flag=0;//控制器状态
int zhuanziDw_flag=0;//转子定位标志 =1需要进行定位。=0不需要
int First_start_flag=0;
int First_start_flag_count=0;

int RELAY2_flag=0;//继电器使能
int RELAY1_flag=0;
int RELAY2_flag_last=0;//继电器使能
int RELAY1_flag_last=0;

int PWM_ENABLE=1;//pwm输出使能。0输出，1关断。

float32 Vac[1000]={0};
float32 iSin[400]={0};

float M=0.95;//调制比

float32 Sin_the=0;//A相相位
float32 Cos_the=0;
//PID控制
//内环
PID_CTRL current_pid_d;/* 电流控制 */
PID_CTRL current_pid_q;
float Id_pid_kp=ld*5543.22;//
float Iq_pid_kp=lq*5543.22;//
float Id_pid_ki=0.0002342;//0.0001
float Iq_pid_ki=0.0002342;//0.0001
float current_out_limit=20;//内环电流PID输出限幅
//外环,转速环
PID_CTRL speed_pid;/* 转速控制 */
float speed_pid_kp=0.035;//0.035
float speed_pid_ki=0.0000875;//0.001
float speed_out_limit=2;//转速环输出限幅

//直流电流环
PID_CTRL currentDC_pid;
float currentDC_pid_kp=0;
float currentDC_pid_ki=0;
float currentDC_out_limit=0;
//直流电流给定
float currentDC_ref=2.5;
float currentDC_ref_ctr=2.5;
//给定
int speed_ref=0;   //转速参考
int speed_ref_ctr=0;
int speed_generation=1500;//发电转速
int speed_ref_ctr_abs=0;
int speed_abs=0;
float32 speed_normK=300;//调制前归一化系数  150
//电机参数
float32 Ld=103.85e-6;
float32 Lq=95.78e-6;
float32 Rs=0.0845;//定子电阻
float Pn=5;   //极对数
float32 flux=0.0057135;//磁链 flux=0.0057135
float32 Kt=0.042851;//转矩常数，Te=Kt*iq。
float32 Te=0;//转矩
//耦合项
float32 w1_d=0;
float32 w1_q=0;

float32 Id_out=0;
float32 Iq_out=0;
float32 Id_out_norm=0;
float32 Iq_out_norm=0;
float32 Varef=0;
float32 Vbref=0;
float32 Vcref=0;
__interrupt void adca1_isr(void);
__interrupt void adcd1_isr(void);

__interrupt void cpu_timer1_isr(void);

__interrupt void INTERRUPT_ISR_TZProtect(void);

extern FILTE VBus_filte;
extern FILTE Vab_filte;
extern FILTE IBus_filte;
extern FILTE temp_filte;


//dq变换
ABC_DQ0_POS_F abc_dq0_pos1_speed;//速度控制

ABC_DQ0_POS_F abc_dq0_pos1_cur;//电流控制

DQ0_ABC_F dq0_abc1_cur;

void PID_Parameter_Init(void);

//触发
int TZ_flag=0;
int TZ=0;
//故障检测
Uint16 Failure_count=0;//计数
Uint16 Failure_count2=0;//计数
Uint16 Failure_flag=0;//故障标志
Uint16 Failure_speed=0;
Uint16 Failure_speedcount=0;
Uint16 Failure_speedflag=0;
//转矩检测变量
Uint16 En_Torque_detec=0;
int zl_flag=0;
int gd_f=0;
int gd_count=1000;//转矩检测用，
float iq_threshold=-2.0f;

///iq滑动窗口参数与变量
#define IQ_FILTER_SIZE 100  // 滑窗大小
float iq_buffer[IQ_FILTER_SIZE] = {0};  // 环形缓冲区
float iq_sum = 0;                       // 累加和
float iq_avg = 0;                       // 平均值
Uint16 iq_index = 0;                   // 当前索引

//////转矩检测状态机
typedef enum {
    TORQUE_WAIT = 0,         // 等待输入转矩
    GENERATING=1,              // 发电运行状态
    SHUTDOWN_CONFIRM=2,        // 等待转速归零
    SHUTDOWN_END=3,
} TorqueState;

TorqueState torque_state = SHUTDOWN_END;//初始状态为发电停止，等待开始指令

Uint16 torque_counter = 0;      // 转矩检测计数器（启动用）
Uint16 shutdown_counter = 0;    // 关断计数器
Uint16 generation_again_en=0;   //允许再次发电标志


int main(void)
{

	InitSysCtrl();
	InitGpio();

	DINT;
	//关闭ＰＩＥ功能,清空PIEIER 和PIEIFR寄存器；用以清除所有的cpu中断响应组
	InitPieCtrl();
	//关闭CPU中断响应；CPU寄存器中也有两个寄存器用于设置中断
    IER = 0x0000;
    IFR = 0x0000;
    //清空中断向量表，即清空所有的中断地址表
    InitPieVectTable();
    //电池充电控制脚
    Battery_Charge_GpioInit();
    //VCC5电源控制
    VCC5_Enable_GpioInit();
    //I2C初始化
    I2CB_GpioInit();//I2C io初始
    I2CB_Init();
    //485
    Modobus_485_GpioInit();
    scia_fifo_init();  // 初始化SCI-A
    //编码器
     Init_Variables();
     Init_EQEP1_Gpio();
     Init_EQEP1();

//    //编码器中断
//    EALLOW;  // This is needed to write to EALLOW protected registers
//    PieVectTable.EQEP1_INT = &myEQEP1_ISR;
//    EDIS;    // This is needed to disable write to EALLOW protected registers
//    PieCtrlRegs.PIEIER5.bit.INTx1 = 1;//EQEP1中断
//    IER |= M_INT5;

//     //VCC5使能引脚
//     // 1. 配置GPIO为GPIO功能（不是复用外设功能）
//        GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);  // Mux位置为0表示GPIO功能
//
//        // 2. 配置GPIO59为输出模式，推挽模式（默认无特殊要求时选择推挽）
//        GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_PUSHPULL);
//
//        // 3. 设置GPIO59的初始状态（默认禁用外部芯片）
//        GPIO_WritePin(8, 1);  // 输出低电平，禁用芯片（根据需求调整为1或0）
////两个继电器驱动引脚
//        // 1. 配置GPIO为GPIO功能（不是复用外设功能）
//           GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 0);  // Mux位置为0表示GPIO功能
//
//           // 2. 配置GPIO59为输出模式，推挽模式（默认无特殊要求时选择推挽）
//           GPIO_SetupPinOptions(20, GPIO_OUTPUT, GPIO_PUSHPULL);
//
//           // 3. 设置GPIO59的初始状态（默认禁用外部芯片）
//           GPIO_WritePin(20, 0);  // 输出低电平，禁用芯片（根据需求调整为1或0）
//
//           // 1. 配置GPIO为GPIO功能（不是复用外设功能）
//              GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 0);  // Mux位置为0表示GPIO功能
//
//              // 2. 配置GPIO59为输出模式，推挽模式（默认无特殊要求时选择推挽）
//              GPIO_SetupPinOptions(21, GPIO_OUTPUT, GPIO_PUSHPULL);
//
//              // 3. 设置GPIO59的初始状态（默认禁用外部芯片）
//              GPIO_WritePin(21, 0);  // 输出低电平，禁用芯片（根据需求调整为1或0）

    //LED
	bsp_led_init();
	//继电器
	bsp_relay_init();
	//bsp_emif_init();
	//按键
	key_Init();
	//pid参数初始化
	PID_Parameter_Init();
///////ADC
	bsp_adc_init();//adc中断配置在其中
///////////PWM
    GPIO_WritePin(12, PWM_ENABLE);//PWM输出使能，低电平有效
	bsp_epwm_init();
	//SPLL_1ph_init(50,(0.00005),&spll1); //锁相环初始化,20kHz

	//I2C中断函数
    EALLOW;    // This is needed to write to EALLOW protected registers
    PieVectTable.I2CB_INT = &i2c_int1a_isr;
    EDIS;      // This is needed to disable write to EALLOW protected registers
    //485中断函数
       EALLOW;  // 允许写入EALLOW保护寄存器
       PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;  // SCI-A接收中断映射到接收ISR
       EDIS;    // 禁用写入EALLOW保护寄存器
    //485 启用中断
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // 启用PIE模块
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE组9，INT1
    IER |= M_INT9;                         // 启用CPU INT中断
    EALLOW;
//    PieVectTable.TIMER0_INT = &cpu_timer0_isr;//地址赋值给 TIMER0_INT 中断向量
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;//地址赋值给 TIMER1_INT 中断向量
    PieVectTable.EPWM1_TZ_INT = &INTERRUPT_ISR_TZProtect;//TZ中断函数地址
    EDIS;
//////////   定时器配置    /////////
    //   初始化CPU定时器0/1/2
    InitCpuTimers();
    //  配置CPU定时器1中断发生时间
    // 200MHz CPU Freq, 1 second Period (in uSeconds)选择定时器，CPU频率，定时器周期（单位us）
    //ConfigCpuTimer(&CpuTimer1, 200, 1000000);/*不分频：200*1000000/200*/     //1s
    //ConfigCpuTimer(&CpuTimer1, 200, 500000);/*不分频：200*1000000/200*/     //0.5s
    ConfigCpuTimer(&CpuTimer1, 200, 100000);/*不分频：200*1000000/200*/     //0.1s
    CpuTimer1Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0表示CPU计时器正在运行。
    //   Enable CPU int13 which is connected to CPU-Timer 1
    IER |= M_INT13;
    // Enable CPU INT8 which is connected to PIE group 8
    IER |= M_INT8;



	//dq变换初始化
	ABC_DQ0_POS_F_init(&abc_dq0_pos1_speed);
    ABC_DQ0_POS_F_init(&abc_dq0_pos1_cur);
    DQ0_ABC_F_init(&dq0_abc1_cur);

    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

	for(;;) {

//	    故障时，进行PWM封锁控制  Failure_flag 1故障；Failure_flag 0无故障
//	    PWM

//	    //软件强制触发
//        //TZ_flag  PWM封锁控制信号   TZ_flag=1:触发封锁  TZ_flag=0:关闭封锁
	    EALLOW;
//        if(Failure_flag==0&&EPwm1Regs.TZFLG.bit.OST==1&&VCC5_EN_FLAG==1&&state_flag==1)//无故障，5V上电，且处于开启状态时，关闭封锁，打开PWM
//        {
//        //开启PWM
//        EPwm1Regs.TZCLR.bit.OST=1;//1:清除触发事件标志
//        EPwm2Regs.TZCLR.bit.OST = 1;
//        EPwm3Regs.TZCLR.bit.OST = 1;
//        EPwm1Regs.TZCLR.bit.INT = 1;    //clear INT flag
//        TZ_flag=3;
//        }
        if(Failure_flag==1&&EPwm1Regs.TZFLG.bit.OST==0)//有故障时，封锁PWM
        {//软件强制封锁
            EPwm1Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
            EPwm2Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
            EPwm3Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
            TZ_flag=3;
        }
	    EDIS;


        //关闭5V电源之后将PWM使能引脚置为低电平
        if(VCC5_EN_FLAG==0)//关闭5V电源之后将PWM使能引脚置为低电平
        {
            DELAY_US(2*1000);//2ms

            //软件强制封锁PWM
            EALLOW;
            EPwm1Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
            EPwm2Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
            EPwm3Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
            EDIS;

            //将PWM使能引脚置为低电平
            PWM_ENABLE=0;//输出低电平
            GPIO_WritePin(12, PWM_ENABLE);//PWM输出使能，0开1关

            //关闭编码器引脚的上拉并置为低电平
            EALLOW;
             GpioCtrlRegs.GPBPUD.bit.GPIO50 = 1; // Enable pull-up on GPIO50 (EQEP1A)
             GpioCtrlRegs.GPBPUD.bit.GPIO51 = 1; // Enable pull-up on GPIO51 (EQEP1B)
             GpioCtrlRegs.GPBPUD.bit.GPIO53 = 1; // Enable pull-up on GPIO53 (EQEP1I)

             GpioCtrlRegs.GPBMUX2.bit.GPIO50=0; //0，普通IO
             GpioCtrlRegs.GPBMUX2.bit.GPIO51=0; //0，普通IO
             GpioCtrlRegs.GPBMUX2.bit.GPIO53=0; //0，普通IO

             GPIO_WritePin(50, 0);//置为低电平
             GPIO_WritePin(51, 0);
             GPIO_WritePin(53, 0);
             EDIS;
        }


	   //电容接入控制
	    if(Turn_on_off==0&&state_flag==1)//下电
	    {
	        speed_ref_ctr=0;//关断时速度置零
	        En_Torque_detec=0;//输入转矩检测置零
	        DELAY_US(100);//10us

	        RELAY_1_OFF();//关继电器1
	        DELAY_US(10*1000);//10ms
	        RELAY1_flag=0;
	        RELAY1_flag_last=RELAY1_flag;//更新继电器状态

	        RELAY_2_OFF();//关继电器2
	        RELAY2_flag=0;
	        RELAY2_flag_last=RELAY2_flag;//更新继电器状态


	        DELAY_US(5000*1000);//5s，再关PWM，放掉电容里的电。
            //软件强制封锁PWM
            EALLOW;
            EPwm1Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
            EPwm2Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
            EPwm3Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
            EDIS;

	        GPIO_WritePin(12, 1);//PWM输出使能，0开1关
	        PWM_ENABLE=1;//PWM使能标志位

	        state_flag=0;//置状态为关。
	        LED3_OFF();//指示灯灭

            //VCC5控制
            VCC5_EN_FLAG=0;//5V 下电
            VCC5_Enable(VCC5_EN_FLAG);
            DELAY_US(1000*1000);//1s
	    }

        if(Turn_on_off==1&&state_flag==0)//上电
        {

            static int on_flag=0;
            if(on_flag==0)
            {
                //VCC5控制
                VCC5_EN_FLAG=1;//5V 上电
                VCC5_Enable(VCC5_EN_FLAG);
                DELAY_US(1000*1000);//1s

            speed_ref_ctr=0;//启动时速度置零
            GPIO_WritePin(12, 1);//PWM输出使能，0开1关  关PWM
            //初始化编码器
            //编码器
             Init_Variables();
             Init_EQEP1_Gpio();
             Init_EQEP1();

            DELAY_US(10*1000);//10ms

            RELAY_2_ON();//开继电器2，预充电
            RELAY2_flag=1;
            RELAY2_flag_last=RELAY2_flag;//更新继电器状态

            //第一次启动给定减小
            speed_out_limit=5;
            current_out_limit=50;

            PID_Parameter_Init();//pid参数置零，初始化

            DELAY_US(1000*1000);//1s
            zhuanziDw_flag=1;//使能转子定位

            //解除PWM封锁
            EALLOW;
            EPwm1Regs.TZCLR.bit.OST=1;//1:清除触发事件标志
            EPwm2Regs.TZCLR.bit.OST = 1;
            EPwm3Regs.TZCLR.bit.OST = 1;
            EPwm1Regs.TZCLR.bit.INT = 1;    //clear INT flag
            EDIS;
            //进行转子定位
            GPIO_WritePin(12, 0);//PWM输出使能，0开1关  开PWM
            PWM_ENABLE=0;//PWM使能标志位
            DELAY_US(500*1000);//500ms

            zhuanziDw_flag=0;//关闭转子定位

            DELAY_US(2000*1000);//2s,等待直流电容充电。
            on_flag++;
            }
            else
            {
                if(fabsf(Supercapacitor_Voltage-Bus_Voltage)<0.2) on_flag++;
                    if(on_flag==25)//判断直流电容是否充电完成。如果充电完成，开继电器1
                    {
                    DELAY_US(500*1000);//500ms
                    RELAY_1_ON();//开继电器1
                    RELAY1_flag=1;
                    RELAY1_flag_last=RELAY1_flag;//更新继电器状态

                    DELAY_US(1000*1000);//1000ms,继电器1与继电器2切换时间过短会造成冲击
                    RELAY_2_OFF();//关继电器2
                    RELAY2_flag=0;
                    RELAY2_flag_last=RELAY2_flag;//更新继电器状态

                    DELAY_US(1000*1000);//1000ms
                    on_flag=0;//开启成功
                    while(motor.Speed_N!=0);
//
                    First_start_flag=0;//第一次启动标志


                    LED3_ON();//指示灯亮
                    torque_state = TORQUE_WAIT;//初始转矩检测状态
                    state_flag=1;//置状态为开启。
                    }
            }
        }


	    //继电器控制
	    if(RELAY2_flag!=RELAY2_flag_last||RELAY1_flag!=RELAY1_flag_last)
	    {
	    if(RELAY2_flag==1)//25欧电阻支路
	    RELAY_2_ON();
	    else
	    RELAY_2_OFF();
	    if(RELAY1_flag==1)//电感支路
	    RELAY_1_ON();
	    else
	    RELAY_1_OFF();
	    RELAY2_flag_last=RELAY2_flag;//更新继电器状态
	    RELAY1_flag_last=RELAY1_flag;
	    }
	//if(RELAY2_flag==1&&Bus_Voltage>(Supercapacitor_Voltage-0.3))
	//{
	//    RELAY_1_ON();
	//    RELAY1_flag=1;
	//}
	    //读电池信息
	    if(COM_Allow==1)
	    {
	        Read_Cap_Current();//读电容电流
	        COM_Allow=0;
	    }
	    if(COM_Allow==2)
	    {//读BMS信息
	        if(I2C_ERROR_FLAG!=0)//I2C故障
	        {
	        I2cbRegs.I2CMDR.bit.IRS=0;//I2C复位
	        I2CB_Init();
	        DELAY_US(100000);//500us
            I2cbRegs.I2CMDR.bit.IRS=1;//I2C使能
	        }
	    Read_BMS_Information(COM_flag);
	    COM_Allow=0;//读完置0
	    }

	    if(slave_addr_Rerr!=0||receive_Rerr!=0)
	    {
        //485通信接收错误处理，恢复
        Modobus_485_ReceiveErr_handle();
	    }

	    Battery_Charge_EN(CHARGE_FLAG);


	}
}

void PID_Parameter_Init(void)
{
    //转速外环参数
      bsp_pid_init(&speed_pid);
      speed_pid.Kp=speed_pid_kp;
      speed_pid.Ki=speed_pid_ki;
      speed_pid.PIDmax=speed_out_limit;
      speed_pid.PIDmin=-speed_out_limit;
      speed_pid.Imax=speed_out_limit;
      speed_pid.Imin=-speed_out_limit;
      speed_pid.Iout=0;
      speed_pid.PIDout=0;
      //电流内环参数
      bsp_pid_init(&current_pid_d);
      current_pid_d.Kp=Id_pid_kp;
      current_pid_d.Ki=Id_pid_ki;
      current_pid_d.PIDmax=current_out_limit;
      current_pid_d.PIDmin=-current_out_limit;
      current_pid_d.Imax=current_out_limit;
      current_pid_d.Imin=-current_out_limit;
      current_pid_d.Iout=0;
      current_pid_d.PIDout=0;

      bsp_pid_init(&current_pid_q);
      current_pid_q.Kp=Iq_pid_kp;
      current_pid_q.Ki=Iq_pid_ki;
      current_pid_q.PIDmax=current_out_limit;
      current_pid_q.PIDmin=-current_out_limit;
      current_pid_q.Imax=current_out_limit;
      current_pid_q.Imin=-current_out_limit;
      current_pid_q.Iout=0;
      current_pid_q.PIDout=0;
}


__interrupt void adca1_isr(void)
{
    //编码器解码
    POSSPEED_Calc();
    //进行故障检测与保护

    speed_abs=abs(motor.Speed_N);
    speed_ref_ctr_abs=abs(speed_ref_ctr);
    if(speed_abs>3400)
    {
        Failure_speedcount++;
        if(Failure_speedcount==200)//连续200次，即连续10ms  转速测量了10次
        {
            Failure_flag=1;
            Failure_speedcount=1;
            Failure_count2=4;
        }

    }
    else Failure_speedcount=0;

    if(Bus_Voltage>43.0||Supercapacitor_Voltage>43.0||fabsf(Iq_Current)>40.0 )// abs(motor.Speed_N)>3300||
    {
        Failure_count++;
        if(Failure_count==10)//连续10次检测到  500us
        {
        Failure_flag=1;
        Failure_count=1;
        }
    }
    else  Failure_count=0;

    if(fabsf(Iq_Current)>32.0)// abs(motor.Speed_N)>3300||
    {
        Failure_count2++;
        if(Failure_count2==400)//连续400次检测到  20ms
        {
        Failure_flag=1;
        Failure_count2=2;
        }
    }
    else  Failure_count2=0;

    if (Cap_ERR_STATUS!=0||Bat_ERR_STATUS!=0)//电容或电池组故障
    {
        Failure_flag=1;
        Failure_count2=3;

    }

///保护
    if(Failure_flag==1)//
    {

              //PWM封锁控制
                    EALLOW;
                    //软件强制封锁
                        EPwm1Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
                        EPwm2Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
                        EPwm3Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
                        TZ_flag=3;
                    EDIS;
//                    //关PWM缓冲芯片
//                    PWM_ENABLE=1;//输出低电平
//                    GPIO_WritePin(12, PWM_ENABLE);//PWM输出使能，0开1关

                    //关继电器
                    RELAY_1_OFF();//关继电器1
                    //DELAY_US(1*1000);//1ms
                    RELAY1_flag=0;
                    RELAY1_flag_last=RELAY1_flag;//更新继电器状态
                    RELAY_2_OFF();//关继电器2
                    RELAY2_flag=0;
                    RELAY2_flag_last=RELAY2_flag;//更新继电器状态

                    //下电标志
                    Turn_on_off=0;
                    state_flag=0;
                    LED3_OFF();//上电指示灯灭
                //故障标志
    }
    /////////////////////////////////////////开启状态//////////////////////////////////////////////////
    if(state_flag==1)//开启状态
    {
    /////////////////////////////输入转矩检测///////////////////////////////////////////
//        //////////反转//////////////////
//        if (En_Torque_detec == 1)
//        {
//            switch (torque_state)
//            {
//                case TORQUE_WAIT:
//                    if (speed_ref_ctr_abs == 0)  // 未启动
//                    {
//                        if (iq_avg > 0.8f)
//                            torque_counter++;
//                        else
//                            torque_counter = 0;
//
//                        if (torque_counter >= 100)
//                        {
//                            speed_ref_ctr = -speed_generation;
//                            speed_ref_ctr_abs = abs(speed_ref_ctr);
//                            torque_state = GENERATING;
//                            torque_counter = 0;
//                        }
//                    }
//                    break;
//
//                case GENERATING:
//                    if ((speed_abs > speed_ref_ctr_abs - 100) && (speed_ref_ctr_abs > 0))
//                    {
//                        if (iq_avg < 0.0f)
//                            shutdown_counter++;
//                        else
//                            shutdown_counter = 0;
//
//                        if (shutdown_counter >= gd_count)
//                        {
//                            if (abs(speed_ref_ctr_abs - speed_abs) < 50)
//                            {
//                                speed_ref_ctr = 0;
//                                torque_state = SHUTDOWN_CONFIRM;
//                                gd_f++;
//                            }
//                        }
//                    }
//                    break;
//
//                case SHUTDOWN_CONFIRM:
//                    if (speed_abs == 0)
//                    {
//                        speed_ref_ctr = 0;
//                        speed_ref_ctr_abs = 0;
//                        torque_state = SHUTDOWN_END;
//                        shutdown_counter = 0;
//                    }
//                    break;
//
//                case SHUTDOWN_END:
//                    break;
//
//                default:
//                    torque_state = SHUTDOWN_END;
//                    break;
//            }
//        }

        //////////正转//////////////////
        if (En_Torque_detec == 1)
        {
            static int SHUTDOWN_count = 0;
            switch (torque_state)
            {
                case TORQUE_WAIT:
                    if (speed_ref_ctr_abs == 0)  // 给定为0，未启动时
                    {
                        //if (iq_avg < iq_threshold && motor.Speed_N>20)
                        if (motor.Speed_N>400)//先转动起来再进行控制，防止启动堵转过电流
                            torque_counter++;
                        else
                            torque_counter = 0;

                        if (torque_counter >= 500)//持续25ms
                        {
                            speed_ref_ctr = speed_generation;
                            speed_ref_ctr_abs = abs(speed_ref_ctr);
                            torque_state = GENERATING;
                            torque_counter = 0;
                        }
                    }
                    break;

                case GENERATING:
                    if ((speed_abs > speed_ref_ctr_abs - 100) && (speed_ref_ctr_abs > 0))
                    {
                        if (iq_avg > -1.7f)//当q轴电流大于0或者直流电流小于0，表明输入扭矩过小，停止发电。
                            shutdown_counter++;
                        else
                            shutdown_counter = 0;

                        if (shutdown_counter >= gd_count)
                        {
                            if (abs(speed_ref_ctr_abs - speed_abs) < 50)
                            {
                                //speed_ref_ctr = 0;
                                torque_state = SHUTDOWN_CONFIRM;
                                gd_f++;
                            }
                        }
                    }
                    break;

                case SHUTDOWN_CONFIRM:

                    //PWM封锁控制
//                          EALLOW;
//                          //软件强制封锁
//                              EPwm1Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
//                              EPwm2Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
//                              EPwm3Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
//                              TZ_flag=3;
//                          EDIS;

                    speed_ref_ctr = 0;
                    speed_ref_ctr_abs = 0;
                    shutdown_counter = 0;
                    if(speed_abs<200)//先控制转速降低，小于阈值之后再结束发电，不再控制
                    {
                    torque_state = SHUTDOWN_END;
                    generation_again_en=0;//失能发电
                    }


//                    if (speed_abs == 0)
//                    {
//                        speed_ref_ctr = 0;
//                        speed_ref_ctr_abs = 0;
//                        torque_state = SHUTDOWN_END;
//                        shutdown_counter = 0;
//                        generation_again_en=0;
//
//                        //软件强制封锁PWM
//                        EALLOW;
//                        EPwm1Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
//                        EPwm2Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
//                        EPwm3Regs.TZFRC.bit.OST=1;//0:无TZ事件；1：有TZ事件。
//                        EDIS;
//                    }
                    break;

                case SHUTDOWN_END:

                    if(speed_abs<10)//转速低于一定值，认为已经结束发电，重置，允许下一次发电
                    {
                        generation_again_en=1;
                    }
                    else if(speed_abs>800)
                    {
                        SHUTDOWN_count++;
                        if(SHUTDOWN_count>500)//25ms
                        {
                            SHUTDOWN_count=0;
                            generation_again_en=1;
                        }
                    }
                    else
                    {
                        generation_again_en=0;
                    }


                    if (generation_again_en==1)
                    {
                        torque_state = TORQUE_WAIT;
//                        //解除PWM封锁
//                        EALLOW;
//                        EPwm1Regs.TZCLR.bit.OST=1;//1:清除触发事件标志
//                        EPwm2Regs.TZCLR.bit.OST = 1;
//                        EPwm3Regs.TZCLR.bit.OST = 1;
//                        EPwm1Regs.TZCLR.bit.INT = 1;    //clear INT flag
//                        EDIS;
                    }
                    break;

                default:
                    torque_state = SHUTDOWN_END;
                    break;
            }
        }


//        //////////反转//////////////////
//        if(En_Torque_detec==1)//开启输入转矩检测。检测到有输入转矩则开始启动
//        {
//            if(speed_ref_ctr_abs==0&&zl_flag<100)//检测到转矩电流，开机
//              {
//                //zl_flag++;
//                if(Iq_Current>0.8) zl_flag++;
//                else zl_flag=0;
//
//                if(zl_flag==100)
//                {
//                speed_ref_ctr=-500;//给定转速
//                speed_ref_ctr_abs=abs(speed_ref_ctr);//更新转速给定的绝对值
//                }
//
//              }
//            //zl_flag>=20&&
//            if((speed_abs>400) && (speed_ref_ctr_abs>400))//处于稳态时，检测到电流连续小于0.2达gd_count次，则关断
//            {
//                //if(Iq_Current<0.2||CC2_Current_mA_Real<0) zl_flag++;
//                if(Iq_Current<0.2) zl_flag++;
//                else zl_flag=100;
//
//                if(zl_flag>=gd_count)
//                {
//                  if(abs(speed_ref_ctr_abs-speed_abs)<50)//处于稳态时,给定与实际差值小
//                    speed_ref_ctr=0;//给定转速
//                  zl_flag=101;
//                    gd_f++;//记录关断次数
//                }
//            }
//
//            if(speed_abs==0&&zl_flag==101) //如果转速过低，电流为转动，则置为初始状态
//            {
//               // GPIO_WritePin(12, 1);//关
//                speed_ref_ctr=0;//给定转速
//               // zl_flag=0;
//            }
//        }
//

//        ////////////正转//////////////////
//        if(En_Torque_detec==1)//开启输入转矩检测。检测到有输入转矩则开始启动
//        {
//            if(speed_ref_ctr_abs==0&&zl_flag<100)//检测到转矩电流，开机
//              {
//                //zl_flag++;
//                if(Iq_Current<-1.5) zl_flag++;
//                else zl_flag=0;
//
//                if(zl_flag==100)
//                {
//                speed_ref_ctr=1000;//给定转速
//                }
//              }
//            //zl_flag>=20&&
//            if((speed_abs>900) && (speed_ref_ctr_abs>900))//检测到电流连续大于0达gd_count次，则关断
//            {
//                if(Iq_Current>0) zl_flag++;
//                else zl_flag=100;
//
//                if(zl_flag==gd_count)
//                {
//                  if(abs(speed_ref_ctr_abs-speed_abs)<50)//处于稳态时,给定与实际差值小
//                    speed_ref_ctr=0;//给定转速为0；
//                    zl_flag=101;
//                    gd_f++;//记录关断次数
//                }
//            }
//
//            if(speed_abs==0&&zl_flag==101) //如果转速过低，电流为转动，则置为初始状态
//            {
//               // GPIO_WritePin(12, 1);//关
//                speed_ref_ctr=0;//给定转速
//              //  zl_flag=0;
//            }
//        }

        speed_ref_ctr_abs=abs(speed_ref_ctr);//更新转速给定的绝对值

    /////////////////////////////采样///////////////////////////////////////////
        //交流电流采样
        APhase_Current=AdcaResultRegs.ADCRESULT0;//
        BPhase_Current=AdcbResultRegs.ADCRESULT0;//
        CPhase_Current=AdccResultRegs.ADCRESULT0;//
    //计算实际值
        Ia_Current=0.0198*APhase_Current - 40.52;//A相电流
        Ib_Current=0.0190*BPhase_Current - 39.115;//B相电流
        Ic_Current=0.0196*CPhase_Current - 40.226;//C相电流

    //归一化
        Ia_Current_norm=Ia_Current*K_norm;
        if(Ia_Current_norm>1.0)Ia_Current_norm=1.0;
        if(Ia_Current_norm<-1.0)Ia_Current_norm=-1.0;

        Ib_Current_norm=Ib_Current*K_norm;
        if(Ib_Current_norm>1.0)Ib_Current_norm=1.0;
        if(Ib_Current_norm<-1.0)Ib_Current_norm=-1.0;

        Ic_Current_norm=Ic_Current*K_norm;
        if(Ic_Current_norm>1.0)Ic_Current_norm=1.0;
        if(Ic_Current_norm<-1.0)Ic_Current_norm=-1.0;


//        //编码器解码
//        POSSPEED_Calc();
        //////电角度
        Sin_the=sinf(motor.theta_elec);
        Cos_the=cosf(motor.theta_elec);
        //dq变换，输入为cos形式，需要归一化（-1,1）//基于余弦的变换
        //交流电流DQ
        abc_dq0_pos1_cur.a = -Ia_Current_norm;//注意电流方向！！！传感器方向问题
        abc_dq0_pos1_cur.b = -Ib_Current_norm;
        abc_dq0_pos1_cur.c = -Ic_Current_norm;
        abc_dq0_pos1_cur.sin = Sin_the;
        abc_dq0_pos1_cur.cos = Cos_the;
        ABC_DQ0_POS_F_FUNC(&abc_dq0_pos1_cur);

        Id_Current=abc_dq0_pos1_cur.d/K_norm;//与A轴重合。反归一化
        Iq_Current=abc_dq0_pos1_cur.q/K_norm;//幅值

        /////////计算iq平均值////滑动滤波
        iq_sum -= iq_buffer[iq_index];        // 减去即将被覆盖的旧值
        iq_buffer[iq_index] = Iq_Current;        // 存入新值
        iq_sum += Iq_Current;        // 加入新值
        iq_avg = iq_sum / IQ_FILTER_SIZE;        // 计算平均值
        // 更新索引，形成循环队列
        iq_index++;
        if (iq_index >= IQ_FILTER_SIZE)
            iq_index = 0;


        Te=Kt*Iq_Current;//转矩计算

        ///////////////首次启动判断,给定逐渐增加，避免冲击///////////////////////
        if(speed_ref_ctr_abs>0&&First_start_flag==0)
        {

            if(speed_out_limit<18.0)
            {
                speed_out_limit=speed_out_limit+0.05;

            }

            if(speed_abs<=100)
            {
                current_out_limit=current_out_limit;

            }
            if(speed_abs>100)
            {
                First_start_flag_count++;//5ms
                if(First_start_flag_count==100) current_out_limit=150;
            }

            if(speed_out_limit>=18.0&&current_out_limit>=150.0)
            {
                First_start_flag=1;
                First_start_flag_count=0;//计数置零，方便下次启动使用
            }
//                        if(speed_out_limit>=15.0)
//                        {
//                            First_start_flag=1;
//                        }

            //转速环输出限幅
            speed_pid.PIDmax=speed_out_limit;
            speed_pid.PIDmin=-speed_out_limit;
            speed_pid.Imax=speed_out_limit;
            speed_pid.Imin=-speed_out_limit;

            current_pid_d.PIDmax=current_out_limit;
            current_pid_d.PIDmin=-current_out_limit;
            current_pid_d.Imax=current_out_limit;
            current_pid_d.Imin=-current_out_limit;

            current_pid_q.PIDmax=current_out_limit;
            current_pid_q.PIDmin=-current_out_limit;
            current_pid_q.Imax=current_out_limit;
            current_pid_q.Imin=-current_out_limit;

        }

        //转速给定赋值
       if(speed_ref!=speed_ref_ctr)//速度参考有变化
       {
            //判断给定指令是否正确
            if(speed_ref_ctr_abs>=0&&speed_ref_ctr_abs<=3000)
            {
                //逐渐增加速度给定
                if(speed_ref==0&&speed_ref_ctr>600)
                {
                    speed_ref=600;
                }
                else if(speed_ref==0&&speed_ref_ctr<-600)
                {
                    speed_ref=-600;
                }
                else if(speed_ref_ctr_abs-abs(speed_ref)>10) //给定大于当前
                    {
                    if(speed_ref_ctr>0)  speed_ref=speed_ref+10;
                    else speed_ref=speed_ref-10;
                    }
                else  speed_ref= speed_ref_ctr;
            }
            else
            {speed_ref_ctr = speed_ref;
            }
       }

        //PID控制
        //转速外环
        speed_pid.ref=(float)speed_ref;//转速参考，注意符号！！！！！此处以正对电机轴顺时针为正
        speed_pid.fb=(float)motor.Speed_N;//转速反馈
        bsp_pid_ctrl(&speed_pid);

        //电流内环
        //d轴
        current_pid_d.ref=0;//电流给定，id=0控制
        current_pid_d.fb=(float)Id_Current;
        bsp_pid_ctrl(&current_pid_d);
        //q轴
        if(generation_again_en==0||torque_state==TORQUE_WAIT)
        {
            speed_pid.Iout=0;
            current_pid_q.ref=0;//电流给定，即转速环输出
        }
        else
        {
        current_pid_q.ref=speed_pid.PIDout;//电流给定，即转速环输出
        }
        current_pid_q.fb=(float)Iq_Current;
        bsp_pid_ctrl(&current_pid_q);
        //耦合项
        w1_d=motor.w_elec*(flux+Ld*Id_Current);
        w1_q=-motor.w_elec*Lq*Iq_Current;
        //解耦
        Id_out=(float32)current_pid_d.PIDout+w1_q;
        Iq_out=(float32)current_pid_q.PIDout+w1_d;
        //归一化
        Id_out_norm=Id_out*2/speed_normK;
        Iq_out_norm=Iq_out*2/speed_normK;
        //限幅
        if(Id_out_norm>1)Id_out_norm=1;
        if(Id_out_norm<-1)Id_out_norm=-1;
        if(Iq_out_norm>1)Iq_out_norm=1;
        if(Iq_out_norm<-1)Iq_out_norm=-1;

    }

    ////////////////////进行转子定位////////////////////////
    if(zhuanziDw_flag==1&&state_flag==0)
    {
      Id_out_norm=0.08;//d轴给定一个电压吸附转子的d轴
      Iq_out_norm=0;
      Sin_the=0;
      Cos_the=1.0;
      motor.theta_elec=0;//电角度置零
      //motor.mech_position=0;//机械角度归零
      EQep1Regs.QPOSCNT=0;//编码器归零

    }
    //结束转子定位
    if(zhuanziDw_flag==0&&state_flag==0)
    {
      Id_out_norm=0;//给定
      Iq_out_norm=0.0;
      Sin_the=0;
      Cos_the=1.0;
      motor.theta_elec=0;//电角度置零
      //motor.mech_position=0;//机械角度归零
      EQep1Regs.QPOSCNT=0;//编码器归零

    }

    //dq--abc
    dq0_abc1_cur.d =Id_out_norm;
    dq0_abc1_cur.q =Iq_out_norm;
    dq0_abc1_cur.z =0;
    dq0_abc1_cur.sin =Sin_the;
    dq0_abc1_cur.cos =Cos_the;
    DQ0_ABC_F_FUNC(&dq0_abc1_cur);

    Varef=dq0_abc1_cur.a;
    if(Varef>1)Varef=1;
    if(Varef<-1)Varef=-1;

    Vbref=dq0_abc1_cur.b;
    if(Vbref>1)Vbref=1;
    if(Vbref<-1)Vbref=-1;

    Vcref=dq0_abc1_cur.c;
    if(Vcref>1)Vcref=1;
    if(Vcref<-1)Vcref=-1;
//    ////////////////////加载比较值////////////////////////
    EPwm1Regs.CMPA.bit.CMPA =  EPwm1Regs.TBPRD*((1.0+M*Varef)/2.0);
    EPwm1Regs.CMPB.bit.CMPB =  EPwm1Regs.TBPRD*((1.0+M*Varef)/2.0);
    EPwm2Regs.CMPA.bit.CMPA =  EPwm2Regs.TBPRD*((1.0+M*Vbref)/2.0);
    EPwm2Regs.CMPB.bit.CMPB =  EPwm2Regs.TBPRD*((1.0+M*Vbref)/2.0);
    EPwm3Regs.CMPA.bit.CMPA =  EPwm3Regs.TBPRD*((1.0+M*Vcref)/2.0);
    EPwm3Regs.CMPB.bit.CMPB =  EPwm3Regs.TBPRD*((1.0+M*Vcref)/2.0);

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//直流量采样
__interrupt void adcd1_isr(void)
{
    static Uint16 VCC5_ON_count=0;
    if(VCC5_EN_FLAG==1)//VCC5上电稳定时才计算采样值
    {
        if(VCC5_ON_count<=10000)//
        {
            VCC5_ON_count++;
        }
        else if(VCC5_ON_count >= 10001 && VCC5_ON_count <= 10201)//进行温漂校正
        {
            Bus_Current_AD=AdcdResultRegs.ADCRESULT2;//
            Bus_Current=IBus_filte.alpha * Bus_Current + (1 - IBus_filte.alpha) *(0.0196*Bus_Current_AD - 39.879);//输出直流电压，滤波后
            ////////////////////计算100次采样的电流平均值，滑窗平均//////////////////////////
                // 减去当前索引对应的旧值
                bus_current_sum -= bus_current_buf[buf_index];
                // 加上新值
                bus_current_buf[buf_index] = Bus_Current;
                bus_current_sum += Bus_Current;
                // 更新索引（循环）
                buf_index++;
                if(buf_index >= FILTER_SIZE)
                    buf_index = 0;
                // 计算平均值
                bus_current_avg = bus_current_sum / FILTER_SIZE;

                Bus_Current_Offset=bus_current_avg-0.0;
                VCC5_ON_count++;
        }
        else if(VCC5_ON_count>10201)
        {
    Bus_Voltage_AD=AdcdResultRegs.ADCRESULT1;//2k欧：y = 0.0083x - 0.0463   1k欧y = 0.0159x - 0.1435
    Bus_Current_AD=AdcdResultRegs.ADCRESULT2;//y = 0.0062x + 0.0212
    Supercapacitor_Voltage_AD=AdcdResultRegs.ADCRESULT3;//y = 0.0083x - 0.065

//    Bus_Voltage=0.0083*Bus_Voltage_AD - 0.0463;
//    //直流母线电压
//    Bus_Voltage=VBus_filte.alpha * Bus_Voltage + (1 - VBus_filte.alpha) *(0.0159*Bus_Voltage_AD - 0.1435);//输出直流电压，滤波后
//    //直流电流
//    Bus_Current=VBus_filte.alpha *Bus_Current+(1 - VBus_filte.alpha) *(0.0062*Bus_Current_AD + 0.0212);
//    //直流母线电压
//    Bus_Voltage=0.0159*Bus_Voltage_AD - 0.1435;//输出直流电压

//    //直流母线电压  理论计算
//    Bus_Voltage=Bus_Voltage_AD*0.0153198;
    //直流母线电压  拟合
    //Bus_Voltage=Bus_Voltage_AD*0.0163-0.0148;
    Bus_Voltage=VBus_filte.alpha * Bus_Voltage + (1 - VBus_filte.alpha) *(Bus_Voltage_AD*0.0163-0.0148);//输出直流电压，滤波后
    //直流电流
//    Bus_Current=0.0196*Bus_Current_AD - 39.879;//直流电流
    Bus_Current=IBus_filte.alpha * Bus_Current + (1 - IBus_filte.alpha) *(0.0196*Bus_Current_AD - 39.879-Bus_Current_Offset);//输出直流电压，滤波后
    //超级电容电压
//    Supercapacitor_Voltage=0.0159*Supercapacitor_Voltage_AD - 0.1435;
    //超级电容电压  理论计算
//    Supercapacitor_Voltage=Supercapacitor_Voltage_AD/4096.0*3*251/12;
   //Supercapacitor_Voltage=Supercapacitor_Voltage_AD*0.0153198;
    //超级电容电压  拟合
   // Supercapacitor_Voltage=Supercapacitor_Voltage_AD*0.0159-0.0167;
    Supercapacitor_Voltage=VBus_filte.alpha * Supercapacitor_Voltage + (1 - VBus_filte.alpha) *(Supercapacitor_Voltage_AD*0.0159-0.0167);
////////////////////计算20次采样的电流平均值，滑窗平均//////////////////////////
    // 减去当前索引对应的旧值
    bus_current_sum -= bus_current_buf[buf_index];
    // 加上新值
    bus_current_buf[buf_index] = Bus_Current;
    bus_current_sum += Bus_Current;
    // 更新索引（循环）
    buf_index++;
    if(buf_index >= FILTER_SIZE)
        buf_index = 0;
    // 计算平均值
    bus_current_avg = bus_current_sum / FILTER_SIZE;
        }
    }
    else
    {
        VCC5_ON_count=0;
    }


    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
////定时器1中断
//__interrupt void cpu_timer1_isr(void)
//{
//
//    if(Failure_flag==1)  LED3C_TOGGLE();//故障时。每0.5秒一次
//    else LED3C_OFF();// 故障指示灯闪烁
//
//    if(I2C_ERROR_FLAG!=0) LED5C_TOGGLE();//I2C通讯故障
//    else LED5C_OFF();
//
//    if(COM_Allow%2==0)//每秒一次
//    {
//        LED2_TOGGLE();//系统运行指示
//        if(VCC5_EN_FLAG==1)  LED4C_TOGGLE();//VCC5上电指示
//        else LED4C_OFF();
//
//       // LED5C_TOGGLE();//其他情况
//    }
//    // 每2秒执行一次 I2C 读取（每4次中断）
//    COM_Allow++;
//    if (COM_Allow > 4)
//        COM_Allow = 1;
//
//
//   // Acknowledge承认 this interrupt to receive more interrupts from group 1
//   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;//中断响应标志位置 1，防止其他PIE组应答
//
//}

// 定时器1中断周期为100ms
__interrupt void cpu_timer1_isr(void)
{
    static Uint16 tick_cnt = 0; // 0.1s为单位，最大值为20即可覆盖2s任务

    tick_cnt++;

    COM_Allow=1;//0.1s每次

    // 每0.5秒一次（5次中断）
    if (tick_cnt % 5 == 0)
    {
        if (Failure_flag == 1||Cap_ERR_STATUS!=00||Bat_ERR_STATUS!=0)
            LED3C_TOGGLE(); // 故障灯闪烁
        else
            LED3C_OFF();
    }

//    // 每0.1秒检测 I2C 通讯状态
//    if (I2C_ERROR_FLAG != 0)
//        LED5C_TOGGLE(); // I2C通讯故障
//    else
//        LED5C_OFF();
  //检测发电状态
    if ( torque_state == TORQUE_WAIT&&state_flag==1)
        LED5C_TOGGLE(); // 准备好发电
    else
        LED5C_OFF();


    // 每1秒执行一次（10次中断）
    if (tick_cnt % 10 == 0)
    {
        LED2_TOGGLE(); // 系统运行指示

        if (VCC5_EN_FLAG == 1)
            LED4C_TOGGLE(); // VCC5上电指示
        else
            LED4C_OFF();
    }

    // 每2秒执行一次（20次中断）
    if (tick_cnt >= 20)
    {
        tick_cnt = 0; // 归零
        // 执行 I2C 读取操作
        COM_Allow=2; // 置2读
    }

    // 中断响应标志位置 1，防止其他PIE组应答
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


__interrupt void INTERRUPT_ISR_TZProtect(void)
{
   // LED3_TOGGLE();
    TZ++;
 //   LED3_ON();
//    EALLOW;
////    EPwm1Regs.TZCLR.bit.OST=1;      //for tz trip One-shot Flag clear;
//    EPwm1Regs.TZCLR.bit.INT = 1;    //clear INT flag
//    EDIS;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2; // Acknowledge interrupt to PIE
}



