/*
 * 485.c
 *
 *  Created on: 2024年11月30日
 *      Author: XUQQ
 */
#include "485_ModobusRTU.h"
//
// 全局变量
//
Uint16 modbus_slave_addr = MODBUS_SLAVE_ADDR;   // 从机地址
Uint16 rx_buffer[8];    // 接收缓冲区
Uint16 tx_buffer[8];    // 发送缓冲区
Uint16 MODBUS_REG_VALUE[15]={0};  // 寄存器值

//错误标志
int crc_err=0;
int func_code_err=0;

int receive_Rerr=0;
int slave_addr_Rerr=0;


interrupt void sciaRxFifoIsr(void)
{
    Uint16 i;
    Uint16 rx_count;
    int cout=0;
    if(SciaRegs.SCIRXST.bit.RXERROR==1)//接收错误中断
    {
        receive_Rerr++;
        SciaRegs.SCIFFTX.bit.SCIRST=0;//sci复位
        for(cout=0;cout<1000;cout++);//延迟一段时间
        SciaRegs.SCIFFTX.bit.SCIRST=1;//sci重新启动
    }
    else//接收正常
    {
        // 获取接收FIFO中的字节数
        rx_count = SciaRegs.SCIFFRX.bit.RXFFST;

        // 确保我们不会读取超过FIFO中实际数据字节数
        for (i = 0; i < rx_count; i++)
        {
            rx_buffer[i] = SciaRegs.SCIRXBUF.all;  // 从接收FIFO读取数据
        }

        // 处理接收到的MODBUS请求
        if (rx_buffer[0] == modbus_slave_addr)  // 地址匹配
        {
            modbus_process_request();  // 处理MODBUS请求
        }
        else //地址不匹配,则不响应
        {
         slave_addr_Rerr++;
        }
    }

    // 清除接收FIFO溢出标志
    SciaRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;   // 清除接收FIFO中断标志
    PieCtrlRegs.PIEACK.all |= 0x100;       // 发送PIE中断确认

}



//
// modbus_process_request - 处理MODBUS请求
//
void modbus_process_request(void)
{
    Uint16 function_code= rx_buffer[1];  // 获取功能码
    Uint16 crc_received = (rx_buffer[7] << 8) + rx_buffer[6]; // 获取接收到的CRC
    Uint16 modbus_reg_addr=(rx_buffer[2] << 8) + rx_buffer[3]; // 获取接收到的寄存器地址
    Uint16 crc_cal=0;
    Uint16 data_len=0;
    Uint16 reg_value_requst=0;
    Uint16 Write_Value=0;
    // 计算MODBUS CRC
    Uint16 crc_calculated = modbus_calculate_crc(rx_buffer, 6);

    if (crc_received != crc_calculated)  // 如果CRC不匹配，则返回错误
    {
        crc_err=1;
        function_code=0;//CRC错误使不回应
    }
    else
        {
        crc_err=0;
        }

    switch(function_code)
    {

        case 0x03:  // 读取保持寄存器
            tx_buffer[0] = modbus_slave_addr; // 从机地址
            tx_buffer[1] = function_code;    // 功能码
            tx_buffer[2] = 0x02;             // 数据长度（2字节）
            //寄存器值，高位在前，低位在后

            reg_value_requst=Read_HoldRegister(modbus_reg_addr);
            tx_buffer[3] = (reg_value_requst >> 8) & 0xFF;  // 高字节
            tx_buffer[4] = reg_value_requst & 0xFF;         // 低字节
//            tx_buffer[3] = (MODBUS_REG_VALUE[modbus_reg_addr] >> 8) & 0xFF;  // 高字节
//            tx_buffer[4] = MODBUS_REG_VALUE[modbus_reg_addr] & 0xFF;         // 低字节
            crc_cal=modbus_calculate_crc(tx_buffer, 5); // 计算CRC
            tx_buffer[5] = crc_cal&0x00FF;//低位
            tx_buffer[6] = crc_cal>>8&0x00FF;//高位
            data_len=7;
            modbus_send_response(data_len);
            func_code_err=0;
            break;

        case 0x06:  // 写单个保持寄存器
            Write_Value = (rx_buffer[4] << 8) | (rx_buffer[5] & 0x00FF);  // 写入的数据
            tx_buffer[0] = modbus_slave_addr; // 从机地址
            tx_buffer[1] = function_code;    // 功能码
            //寄存器地址占两字节，高位在前，低位在后
            tx_buffer[2] = rx_buffer[2];  // 写入的寄存器地址高字节
            tx_buffer[3] = rx_buffer[3];         // 写入的寄存器地址低字节
            //寄存器值，大端在前，小端在后
            //处理写入
            reg_value_requst=Write_HoldRegister(modbus_reg_addr,Write_Value);
            //回读写入的数据
            tx_buffer[4] = rx_buffer[4];   // 写入的寄存器值高字节
            tx_buffer[5] = rx_buffer[5];          // 写入的寄存器值低字节
            crc_cal=modbus_calculate_crc(tx_buffer, 6); // 计算CRC
            //发送时，CRC小端在前，大端在后
            tx_buffer[6] = crc_cal&0x00FF;
            tx_buffer[7] = crc_cal>>8&0x00FF;
            data_len=8;
            modbus_send_response(data_len);
            func_code_err=0;
            break;
        case 0x04://读单个输入寄存器（只读）
            tx_buffer[0] = modbus_slave_addr; // 从机地址
            tx_buffer[1] = function_code;    // 功能码
            tx_buffer[2] = 0x02;             // 数据长度（2字节）
            //寄存器值，高位在前，低位在后
            reg_value_requst=Read_InputRegister(modbus_reg_addr);
            tx_buffer[3] = (reg_value_requst >> 8) & 0xFF;  // 高字节
            tx_buffer[4] = reg_value_requst & 0xFF;         // 低字节
            crc_cal=modbus_calculate_crc(tx_buffer, 5); // 计算CRC
            tx_buffer[5] = crc_cal&0x00FF;//低位
            tx_buffer[6] = crc_cal>>8&0x00FF;//高位
            data_len=7;
            modbus_send_response(data_len);
            func_code_err=0;
            break;
        default:
            func_code_err=1; // 不支持的功能码，则不响应

    }
}

//
// modbus_send_response - 发送MODBUS响应
//
void modbus_send_response(Uint16 data_len)
{
    Uint16 i;
    //发送之前，FIFIO指针置零，避免FIFO中残留的内容影响发送
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 0; // 置零发送FIFO，指针归零
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 1; // 重置发送FIFO，
    // 发送响应数据
    for (i = 0; i < data_len; i++)
    {
        SciaRegs.SCITXBUF.all = tx_buffer[i];
    }
}

//
// modbus_calculate_crc - 计算MODBUS CRC
Uint16 modbus_calculate_crc(Uint16 *data, Uint16 length) {
    Uint16 crc = 0xFFFF;  // 初始CRC值
    Uint16 i, j;
    Uint16 byte;  // 用来提取字节

    // 遍历所有数据字节
    for (i = 0; i < length; i++) {
        byte = data[i] & 0x00FF;  // 提取低字节
        crc ^= byte;  // 将低字节与CRC进行异或

        // 对低字节进行8次移位
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {  // 如果CRC的最低位为1
                crc = (crc >> 1) ^ 0xA001;  // 右移并与多项式0xA001异或
            } else {
                crc >>= 1;  // 否则，仅右移
            }
        }
    }

    return crc;  // 返回最终的CRC值
}



//
// scia_fifo_init - 配置SCI-A FIFO
//
void scia_fifo_init()
{
   SciaRegs.SCICCR.all = 0x0007;      // 1位停止位，无环回
                                      // 无奇偶校验，8位数据位
                                      // 异步模式，空闲行协议
   SciaRegs.SCICTL1.all = 0x0003;     // 启用TX、RX、内部SCICLK，
                                      // 禁用RX错误、休眠、TX唤醒
   SciaRegs.SCICTL1.bit.RXERRINTENA=1;//启用接受错误中断
   SciaRegs.SCICTL2.bit.TXINTENA = 0; // 关闭发送中断
   SciaRegs.SCICTL2.bit.RXBKINTENA = 1; // 启用接收中断

   SciaRegs.SCIHBAUD.all = 0x0000;
   SciaRegs.SCILBAUD.all = SCI_PRD;   // 设置波特率

   SciaRegs.SCICCR.bit.LOOPBKENA = 0; // 启用环回模式
   SciaRegs.SCIFFTX.all = 0xC022;
   SciaRegs.SCIFFTX.bit.TXFFIL=8;//发送深度为8，即8个字节
   SciaRegs.SCIFFRX.all = 0x0022;
   SciaRegs.SCIFFRX.bit.RXFFIL=8;//接受深度为8，即8个字节
   SciaRegs.SCIFFCT.all = 0x00;

   SciaRegs.SCICTL1.all = 0x0023;     // 释放SCI复位
   SciaRegs.SCICTL1.bit.RXERRINTENA=1;//启用接受错误中断
   SciaRegs.SCIFFTX.bit.TXFIFORESET = 1; // 重置发送FIFO
   SciaRegs.SCIFFRX.bit.RXFIFORESET = 1; // 重置接收FIFO

}
void Modobus_485_ReceiveErr_handle(void)
{
    int count=0;
    if((slave_addr_Rerr!=0||receive_Rerr!=0)&&(SciaRegs.SCIFFRX.bit.RXFFST!=0))//有错误
    {
        SciaRegs.SCIFFRX.bit.RXFIFORESET = 0; // 置零接收FIFO，指针归零
        for(count=0;count<50;count++);//延迟一段时间
        SciaRegs.SCIFFRX.bit.RXFIFORESET = 1; // 重置接收FIFO，启动接收
        if(SciaRegs.SCIFFRX.bit.RXFFST==0)
        {
        slave_addr_Rerr=0;
        receive_Rerr=0;
        }
    }
}

void Modobus_485_GpioInit(void)
{
    //
    // 对于本示例，仅初始化SCI-A端口的引脚。
    // GPIO_SetupPinMux() - 设置GPxMUX1/2和GPyMUX1/2寄存器位
    // GPIO_SetupPinOptions() - 设置GPIO的方向和配置
    // 这些函数在F2837xD_Gpio.c文件中定义
    //
       GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);  // 配置GPIO28为SCI-A RX
       GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);  // 配置为输入模式
       GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);  // 配置GPIO29为SCI-A TX
       GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);  // 配置为输出模式
}

Uint16 Read_HoldRegister(Uint16 reg_addr)//读保持寄存器
{
    Uint16 re_value=0;
    switch(reg_addr)
    {
    case 0://转速给定
        re_value=(Uint16)speed_ref_ctr;
        break;
    case 2:
        re_value=(Uint16)Turn_on_off;//上电，下电标志，直流接入控制
        break;
    case 4:
        re_value=(Uint16)RELAY1_flag;
        break;
    case 5:
        re_value=(Uint16)RELAY2_flag;
        break;
    case 21:
        re_value=En_Torque_detec;//输入转矩检测使能位
        break;
    case 22:
        re_value=CHARGE_FLAG;//电池充电使能
        break;
    case 23:
        re_value=Failure_flag;////故障标志
        break;
    case 24:
        re_value=VCC5_EN_FLAG;////VCC5使能标志
        break;
    case 28:
        re_value=generation_again_en;////VCC5使能标志
        break;
    case 29:
        re_value=(Uint16)speed_generation;////初始发电转速
        break;

    default:
        re_value=0xFF;

    }

    return re_value;

}
Uint16 Write_HoldRegister(Uint16 reg_addr,Uint16 Wite_Value)//写 保持寄存器
{
    Uint16 re_value=0;
    switch(reg_addr)
    {
    case 0://转速给定
        speed_ref_ctr=(int)Wite_Value;
        break;
    case 2:
        Turn_on_off=(int)Wite_Value;//上电，下电标志，直流接入控制
        break;
    case 4:
        RELAY1_flag=(int)Wite_Value;
        break;
    case 5:
        RELAY2_flag=(int)Wite_Value;
        break;
    case 21:
        En_Torque_detec=Wite_Value;//输入转矩检测使能位
        break;
    case 22:
        CHARGE_FLAG=Wite_Value;//电池充电使能
        break;
    case 23:
        Failure_flag=Wite_Value;////故障标志
        break;
    case 24:
        VCC5_EN_FLAG=Wite_Value;//VCC5V使能
        break;
    case 28:
        generation_again_en=Wite_Value;//再次发电使能
        break;
    case 29:
        speed_generation=(int)Wite_Value;//初始发电转速
        break;

    default:
        re_value=0xFF;
    }
    return re_value;
}
Uint16 Read_InputRegister(Uint16 reg_addr)//读输入寄存器
{
    Uint16 re_value=0;
    switch(reg_addr)
    {
    case 1:
        re_value=(Uint16)motorspeed_rpm;//转速反馈
        //re_value=1000;
        break;
    case 3:
        re_value=(Uint16)state_flag;//控制器状态，电机控制器状态
        break;
    case 6:
        re_value=(Uint16)(Bus_Voltage*100);//母线电压
        break;
    case 7:
        re_value=(Uint16)(int)(Bus_Current*1000);//母线电流
        break;
    case 8:
        re_value=(Uint16)(Supercapacitor_Voltage*100);//电容电压
        break;
    case 9:
        re_value=(Uint16)(int)(Id_Current*100);//D轴电流///先转int 再转 Uint16，否则符号会丢失。
        break;
    case 10:
        re_value=(Uint16)(int)(Iq_Current*100);//Q轴电流
        break;
    case 11:
        re_value=Bat_ERR_STATUS;//电池组状态
        break;
    case 12:
        re_value=Bat_PACK_Voltage;//电池组电压，单位mV
        break;
    case 13:
        re_value=Bat_PACK_Current;//电池组电流，单位mA
        break;
    case 14:
        re_value=Bat_CELL_MaxVoltage;//电池最大电压，单位mV
        break;
    case 15:
        re_value=Bat_CELL_MinVoltage;//电池最小电压，单位mV
        break;
    case 16:
        re_value=Cap_ERR_STATUS;//电容组状态
        break;
    case 17:
        re_value=Cap_PACK_Voltage;//电池组电压，单位mV
        break;
    case 18:
        re_value=Cap_PACK_Current;//电池组电流，单位mA
        break;
    case 19:
        re_value=Cap_CELL_MaxVoltage;//电池最大电压，单位mV
        break;
    case 20:
        re_value=Cap_CELL_MinVoltage;//电池最小电压，单位mV
        break;
    case 25:
        re_value=(Uint16)(int)(Te*1000);//转矩
        break;
    case 26:
        re_value=(Uint16)(int)(bus_current_avg*1000);//直流电流
        break;
    case 27:
        re_value=(Uint16)(int)((int)(Cap_PACK_Current)*0.001*Supercapacitor_Voltage*100);//直流功率
        break;
    default:
        re_value=0xFF;
    }
    return re_value;
}
