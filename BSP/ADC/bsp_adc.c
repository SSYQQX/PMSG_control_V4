
#include "bsp_adc.h"
#include "math.h"

extern __interrupt void adca1_isr(void);
extern __interrupt void adcd1_isr(void);

/* 一阶滤波 */
#define PI              3.14159265f
#define TS              0.00005f/* 采样周期50us */


// float alpha;/* 滤波系数 */
// float f;/* 截止频率 */
// float t;/* 时间常数 */
FILTE Vab_filte;
FILTE VBus_filte;
FILTE IBus_filte;
FILTE temp_filte;


//这里的 alpha 越大，响应越慢、滤波越强；
//alpha 越小，响应越快、保留动态更多。
//void filte_init(FILTE *filte);
static void filte_init(FILTE *filte)
{
    filte->t = 1 / (2.0f * PI * filte->f);
    filte->alpha = filte->t / (filte->t + TS);
}



void bsp_adc_init(void)
{

  //  SetupInputXBAR5();

    EALLOW;
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    /* 配置ADC为单端模式 12bit */
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    /* 转换结束发送脉冲 *///将脉冲设为延迟，中断触发在soc后期
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    /* 开启ADC电源 */
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    EDIS;

    DELAY_US(1000);
//一阶低通滤波器初始化
    Vab_filte.f=1000;
    filte_init(&Vab_filte);

    VBus_filte.f = 150;
    filte_init(&VBus_filte);

    IBus_filte.f=50;
    filte_init(&IBus_filte);

    temp_filte.f = 5;
    filte_init(&temp_filte);

    // f = 200;
    // t = 1 / (2.0f * PI * f);
    // alpha = t / (t + TS);

    EALLOW;
/* ---------------------ADC_A配置------------------------- */
    /* 输入A相电流 A0 */
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;/* 采样时间75ns ，采样窗口=（acqps+1）*1/SYSCLKS；SYSCLKS=200Mhz；*/
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;//触发源
//    /* 碳化硅5温度 A1 */
//    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;
//    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14;
//    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 4;
//
//    /* 飞跨电容电压 A2 */
//    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;
//    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 14;
//    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 4;
//
//    /* 碳化硅4温度 A3 */
//    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;
//    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 14;
//    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 4;
//
//    /* 输入电流 A4 */
//    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 4;
//    AdcaRegs.ADCSOC4CTL.bit.ACQPS = 14;
//    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 4;
//
//    /* 碳化硅3温度 A5 */
//    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 5;
//    AdcaRegs.ADCSOC5CTL.bit.ACQPS = 14;
//    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 4;
//
//    /* 碳化硅6温度 IN14 */
//    AdcaRegs.ADCSOC14CTL.bit.CHSEL = 14;
//    AdcaRegs.ADCSOC14CTL.bit.ACQPS = 14;
//    AdcaRegs.ADCSOC14CTL.bit.TRIGSEL = 4;
//
//    /* DAB副边母线电压 IN15 */
//    AdcaRegs.ADCSOC15CTL.bit.CHSEL = 15;
//    AdcaRegs.ADCSOC15CTL.bit.ACQPS = 14;
//    AdcaRegs.ADCSOC15CTL.bit.TRIGSEL = 4;
//
//
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

/* ---------------------ADC_B配置------------------------- */
     /* 输入B相电流 B0 */
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;/* 采样时间75ns */
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;
//    /* 输出电压 B1 */
//    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1;
//    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 14;/* 采样时间75ns */
//    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 4;
//     /* PCB温度 B2 */
//    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;
//    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 14;/* 采样时间75ns */
//    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 4;
//     /* 输出电流 B3 */
//    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 3;
//    AdcbRegs.ADCSOC3CTL.bit.ACQPS = 14;/* 采样时间75ns */
//    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 4;
//
//    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 1;
//    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;
//    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
/* ---------------------ADC_C配置------------------------- */
     /* 输入C相电流 C2 */
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 14;/* 采样时间75ns */
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

//     /* 碳化硅1温度 C4 */
//    AdccRegs.ADCSOC4CTL.bit.CHSEL = 4;
//    AdccRegs.ADCSOC4CTL.bit.ACQPS = 14;/* 采样时间75ns */
//    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 4;
//
//
//    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 1;
//    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;
//    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
/* ---------------------ADC_D配置------------------------- */

         /* 线电压Uab D3 */
        AdcdRegs.ADCSOC0CTL.bit.CHSEL = 3;
        AdcdRegs.ADCSOC0CTL.bit.ACQPS = 14;/* 采样时间75ns */
        AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

     /* 输出母线电压 D0 */
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 0;
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 14;/* 采样时间75ns */
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 5;

     /* 输出母线电流 D1 */
    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 1;
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = 14;/* 采样时间75ns */
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 5;

     /* 输出超级电容电压 D2 */
    AdcdRegs.ADCSOC3CTL.bit.CHSEL = 2;
    AdcdRegs.ADCSOC3CTL.bit.ACQPS = 14;/* 采样时间75ns */
    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = 5;

//
    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 3;
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

//ADC中断控制
    EDIS;

       EALLOW;  // This is needed to write to EALLOW protected registers
       PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
       PieVectTable.ADCD1_INT = &adcd1_isr; //function for ADCD interrupt 1
       EDIS;    // This is needed to disable write to EALLOW protected registers

    // Enable ADC in the PIE: Group 1 interrupt 1
    //
       PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
       PieCtrlRegs.PIEIER1.bit.INTx1 = 1;          // Enable ADCA1 interrupt
       PieCtrlRegs.PIEIER1.bit.INTx6 = 1;          // Enable ADCD1 interrupt
       IER |= M_INT1;                              // Enable CPU INT1，开中断1

   EINT;
   ERTM;
}


