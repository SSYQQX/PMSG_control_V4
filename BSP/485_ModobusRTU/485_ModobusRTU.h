/*
 * 485.h
 *
 *  Created on: 2024年11月30日
 *      Author: XUQQ
 */
#ifndef MODOBUSRTU_485_H_
#define MODOBUSRTU_485_H_
//
// 包含的头文件
//
#include "F28x_Project.h"
#include "bsp_gpio.h"
//
// 宏定义
//
#define CPU_FREQ        200E6      // CPU频率为60 MHz
#define LSPCLK_FREQ     CPU_FREQ/4 // LSP时钟频率为CPU频率的四分之一
#define SCI_FREQ        115200      // SCI波特率115200 bps
#define SCI_PRD         (LSPCLK_FREQ/(SCI_FREQ*8))-1 // SCI时钟周期

#define MODBUS_SLAVE_ADDR 0x01   // 从机地址
//
// 函数原型声明
//
interrupt void sciaRxFifoIsr(void);      // SCI-A 接收FIFO中断服务程序
void modbus_process_request(void);       // 处理MODBUS请求
void modbus_send_response(Uint16 data_len);         // 发送MODBUS响应
void scia_fifo_init(void);               // SCI-A FIFO初始化
Uint16 modbus_calculate_crc(Uint16 *data, Uint16 length); // 计算MODBUS CRC
void Modobus_485_ReceiveErr_handle(void);// //485通信接受错误处理，恢复
void Modobus_485_GpioInit(void);
Uint16 Read_HoldRegister(Uint16 reg_addr);
Uint16 Write_HoldRegister(Uint16 reg_addr,Uint16 re_value);
Uint16 Read_InputRegister(Uint16 reg_addr);
//错误标志
extern int crc_err;
extern int func_code_err;

extern Uint16 Failure_flag;//故障标志

extern int receive_Rerr;
extern int slave_addr_Rerr;
extern Uint16 MODBUS_REG_VALUE[15];  // 寄存器值
extern int speed_generation;//初始发电转速
extern int speed_ref_ctr;//给定转速
extern int motorspeed_rpm;//转速反馈
extern int Turn_on_off;//上电，下电标志，直流接入控制
extern int state_flag;//控制器状态，电机控制器状态
extern int RELAY2_flag;//继电器使能，状态
extern int RELAY1_flag;
extern Uint16 generation_again_en;//允许再次发电使能
extern Uint16 En_Torque_detec;//输入转矩检测使能位
extern  float32 Bus_Voltage;
extern  float32 Bus_Current;
extern float32 Supercapacitor_Voltage;
extern float32 Id_Current;
extern float32 Iq_Current;
//电池组
extern Uint16 Bat_ERR_STATUS;
extern Uint16 Bat_PACK_Voltage;
extern Uint16 Bat_PACK_Current;
extern Uint16 Bat_CELL_MaxVoltage;
extern Uint16 Bat_CELL_MinVoltage;
//电容组
extern Uint16 Cap_ERR_STATUS;
extern Uint16 Cap_PACK_Voltage;
extern Uint16 Cap_PACK_Current;
extern Uint16 Cap_CELL_MaxVoltage;
extern Uint16 Cap_CELL_MinVoltage;
extern float32 Te;
extern float bus_current_avg;
#endif /* MODOBUSRTU_485_H_ */
