#include "bsp_I2C.h"

int COM_flag=100;//读写设备号
int I2C_ERROR_FLAG=0;//I2C故障标志
int COM_Allow=0;//I2C访问控制位，=2时通信
///////////////////////////////BMS变量//////////////////////////////////////////////////
int invalid_interrupt_flag=0;
//寄存器配置值
int Ram_SetFlag=0;
Uint16 Alarm_Set=0;//警报启用

Uint16  FET_Status=0;//FET状态  ALRT_PIN DDSG_PIN DCHG_PIN PDSG_FET DSG_FET PCHG_FET CHG_FET

Uint16  Pump_Control=0;//

Uint16 Safety_Alert_A=0xFF;//A类警告
Uint16 Safety_Status_A=0xFF;//A类故障
Uint16 Safety_Alert_B=0xFF;//B类警告
Uint16 Safety_Status_B=0xFF;//B类故障
Uint16 Safety_Alert_C=0xFF;//C类警告
Uint16 Safety_Status_C=0xFF;//C类故障

Uint16  PF_Alert_A=0xFF;//
Uint16  PF_Status_A=0xFF;//
Uint16  PF_Alert_B=0xFF;//
Uint16  PF_Status_B=0xFF;//
Uint16  PF_Alert_C=0xFF;//
Uint16  PF_Status_C=0xFF;//
Uint16  PF_Alert_D=0xFF;//
Uint16  PF_Status_D=0xFF;//
//电压电流
Uint16  Cell1_Voltage_mv=0;//电芯1电压
Uint16  Cell2_Voltage_mv=0;//电芯2电压
Uint16  Cell3_Voltage_mv=0;//电芯3电压
Uint16  Cell4_Voltage_mv=0;//电芯4电压
Uint16  Cell5_Voltage_mv=0;//电芯5电压
Uint16  Cell6_Voltage_mv=0;//电芯6电压
Uint16  Cell7_Voltage_mv=0;//电芯7电压
Uint16  Cell8_Voltage_mv=0;//电芯8电压
Uint16  Cell9_Voltage_mv=0;//电芯9电压
Uint16  Cell10_Voltage_mv=0;//电芯10电压
Uint16  Stack_Voltage_mv=0;//电池组顶部电压
Uint16  PACK_Pin_Voltage_mv=0;//PACK引脚上电压
Uint16  CC2_Current_mA=0;//CC2 滤波器生成的电流测量值
int16  CC2_Current_mA_Real=0;//CC2 滤波器生成的电流测量值
Uint16 Cells_MinVoltage_mv=0;
Uint16 Cells_MaxVoltage_mv=0;
//温度测量值
int16 Int_Temperature_Value=0;//内部温度  单位0.1k
int16 TS1_Temperature_Value=0;//Ts1温度
int16 TS2_Temperature_Value=0;//Ts2温度
int16 TS3_Temperature_Value=0;//Ts3温度
//电池均衡
int16 CB_ACTIVE_CELLS=0;//
//电池状态
Uint16 Battery_Status=0;

Uint16 I2C_MODE_Val=0;//I2C 模式
Uint16 Vcell_Num=0;//电池节数
Uint16 COV_Threshold_mv=0;//过压保护阈值
Uint16 CUV_Threshold_mv=0;//过压保护阈值
Uint16 SCD_Threshold_A=0;//放电短路保护阈值
int OCC_Threshold_A=0;//充电过流保护阈值
Uint16 OCD1_Threshold_A=0;//放电过流保护阈值1
int OCD2_Threshold_A=0;//放电过流保护阈值2
Uint16 FET_ControlMODE=0;//FET控制模式
Uint16 FET_Options=0;//地址 0x9308，默认值 0x0D。。FET控制模式。串并联设置等
Uint16 Protections_A=0;//启用各种保护
Uint16 Manufacturing_Status_Value=0;

//写，子命令
Uint16 SWAP_I2C_MODE8[4]={0xE7,0x36,0x29,0xDF};//i2c快速模式，无crc。写入0x3E，此句带CRC
Uint16 I2C_MODE_Addr[2]={0x39,0x92};//i2c模式存储器地址
//更改I2C地址
Uint16 I2C_Address_Addr[2]={0x3A,0x92};//0x923A
Uint16 I2C_Address_Value[1]={0x40};//地址由0x10改为0x40
//进入CONFIG_UPDATE 模式  子命令
Uint16 ENTER_CFG_UPDATE_Addr[2]={0x90,0x00};//0x0090
//退出CONFIG_UPDATE 模式  子命令
Uint16 EXIT_CFG_UPDATE_Addr[2]={0x92,0x00};//0x0092
//复位命令
Uint16 RESET_Addr[2]={0x12,0x00};//0x0012
//Uint16 Settings_Protection_A[16]={0x3A,0x92};
Uint16 Settings_Protection_A[16]={0x80,0x91};
//数据存储器访问。将地址的低位字节写入 0x3E，将地址的高位字节写入 0x3F。从从0x40 开始读取数据长度的缓冲区。从 0x61 读取响应的长度
//电池节数配置
Uint16 Vcell_Mode_Addr[2]={0x04,0x93};//电池节数配置。地址
Uint16 Vcell_Mode_value[2]={0xFF,0x03};//写入值 0x03FF.10节
//过压保护阈值
Uint16 COV_Threshold_Addr[2]={0x78,0x92};//过压保护阈值配置。地址0x9278
Uint16 COV_Threshold_Value[1]={76};//写入值 76;76*50.6=3800mv  3845
//低压保护阈值
Uint16 CUV_Threshold_Addr[2]={0x75,0x92};//低压保护阈值配置。地址0x9275
Uint16 CUV_Threshold_Value[1]={49};//写入值 49; 49*50.6=2500mv  2479
//放电短路保护阈值
Uint16 SCD_Threshold_Addr[2]={0x86,0x92};//放电短路保护阈值配置。地址0x9286  20mR电阻
Uint16 SCD_Threshold_Value[1]={13};//写入值 13;  20A 400mv 第14档
//充电过流保护阈值
Uint16 OCC_Threshold_Addr[2]={0x80,0x92};//充电过流保护阈值。地址0x9280
Uint16 OCC_Threshold_Value[1]={35};//写入值 35;  3.5A 70mv=35*2mv
//充电过流保护延迟
Uint16 OCC_Delay_Addr[2]={0x81,0x92};//0x9281
Uint16 OCC_Delay_Value[1]={0x0A};//写入值10     10*3.3ms=33ms
//放电过流保护阈值1  OCD1电流大 延迟时间短 300ms
Uint16 OCD1_Threshold_Addr[2]={0x82,0x92};//放电过流保护阈值1。地址0x9282
Uint16 OCD1_Threshold_Value[1]={100};//写入值 100;  10A  200mv=100*2mv
//放电过流保护1延迟
Uint16 OCD1_Delay_Addr[2]={0x83,0x92};//0x9283
Uint16 OCD1_Delay_Value[1]={0x5B};//写入值91     91*3.3ms=300ms
//放电过流保护阈值2  OCD2电流小 延迟时间长 24ms
Uint16 OCD2_Threshold_Addr[2]={0x84,0x92};//放电过流保护阈值2。地址0x9284
Uint16 OCD2_Threshold_Value[1]={35};//写入值 35;  3.5A  70mv=35*2mv
//放电过流保护2延迟
Uint16 OCD2_Delay_Addr[2]={0x85,0x92};//0x9285
Uint16 OCD2_Delay_Value[1]={0x7F};//写入值127     127*3.3ms=419ms
///////////////放电过流恢复条件
//电流阈值。电流大于一定值恢复。放电电流为负，充电电流为正 单位mA
Uint16 OCD_Recovery_CurrentThreshold_Addr[2]={0x8D,0x92};//0x928D
Uint16 OCD_Recovery_CurrentThreshold_Value[2]={0x9C,0XFF};///-100  补码为ff9c
//时间阈值
Uint16 OCD_Recovery_TimeThreshold_Addr[2]={0xAF,0x92};//0x92AF
Uint16 OCD_Recovery_TimeThreshold_Value[1]={0x05};///5 S
/////////////电芯平衡设置///////持续均衡20s秒后，电芯平衡算法重新计算要平衡的电芯。
Uint16 Balancing_Configuration_Addr[2]={0x35,0x93};//0x9335.均衡模式设置
Uint16 Balancing_Configuration_Value[1]={0x01};//充电模式下允许均衡。睡眠模式下禁止均衡，静止条件下禁止均衡。
//允许同时平衡的电芯数 5
Uint16 Cell_Balance_Max_Cells_Addr[2]={0x3A,0x93};//0x933A
Uint16 Cell_Balance_Max_Cells_Value[1]={5};//5节
//最低电芯电压高于此值,且最大电芯与最小电芯电压差大于40mV 时 开始均衡
Uint16 Cell_Balance_Min_Cell_V_Addr[2]={0x3B,0x93};// 0x933B
Uint16 Cell_Balance_Min_Cell_V_Value[2]={0x10,0x0E};//3600mv 0x0e10
//读取正在均衡的电池  子命令
Uint16 CB_ACTIVE_CELLSA_Addr[2]={0x83,0x00};//0x0083
//电芯平衡时的电压测量速度  存储器
Uint16 Power_Config_Addr[2]={0x34,0x92};// 0x9234
Uint16 Power_Config_Value[2]={0xA2,0x29};//0x29A2.///均衡时电压测量速度降为1/4

//CC Gain 电流采样电阻值   CC_Gain_Value=7.4768/Rs   以IEEE754 单精度 4 字节格式表示
Uint16 CC_Gain_Addr[2]={0xA8,0X91};//0x91A8
Uint16 CC_Gain_Value[4]={0xB7,0x62,0xBF,0x3E};//使用 IEEE754 单精度 4 字节格式以小端字节序的顺序存储浮点值//20mR 3E BF 62 B7
//Capacity Gain  将库仑计数器的原始计数测量值转换为通过的电荷 Capacity Gain=CC Gain*298261.6178
Uint16 Capacity_Gain_Addr[2]={0xAC,0X91};//0x91AC
Uint16 Capacity_Gain_Value[4]={0x10,0xC7,0xD9,0x47};//使用 IEEE754 单精度 4 字节格式以小端字节序的顺序存储浮点值//20mR 47 D9 C7 10
//FET 驱动器的当前状态
Uint16 Manufacturing_Status_Addr[2]={0x57,0x00};//驱动器的当前状态
//低电压 Shutdown阈值；电池包电压阈值
Uint16 Shutdown_Stack_Voltage_Addr[2]={0x41,0x92};//0x9241
Uint16 Shutdown_Stack_Voltage_Value[2]={0x00,0x00};//0//低压不进入Shutdown模式
//配置FET控制模式。自主模式。
Uint16 FET_ControlMODE_Addr[2]={0x43,0x93};//自主模式。。地址0x9343  Manufacturing:Mfg Status Init
Uint16 FET_ControlMODE_Value[2]={0x10,0x00};//写入值 0x0010; 禁用永久失效检查，启用FET正常控制模式。注意是两个长度
// FET_ENABLE
Uint16 FET_ENABLE_Addr[2]={0x22,0x00};
Uint16 PF_ENABLE_Addr[2]={0x24,0x00};
// FET Options
Uint16 FET_Options_Addr[2]={0x08,0x93};//地址0x9308
Uint16 FET_Options_Value[1]={0x1F};//写入值 0x1F; DSG 之前导通 PDSG FET;器件控制，串联模式;写入0x1F，CHG可以在SLEEP模式下启用。
//Protections A配置 SCD OCD2 OCD1 OCC COV CUV
Uint16 Protections_A_Addr[2]={0x61,0x92};//地址0x9261
Uint16 Protections_A_Value[1]={0xFC};//写入值 0xFC; 启用SCD OCD2 OCD1 OCC COV CUV保护
Uint16 DSG_FET_Protections_A[2]={0x69,0x92};//0x9269
Uint16 DSG_FET_Protections_A_Value[1]={0};//DSG FET保护动作使能。0xE4全开
Uint16 CHG_FET_Protections_A[2]={0x65,0x92};//0x9265
Uint16 CHG_FET_Protections_A_Value[1]={0};//CHG FET保护动作使能。0x98 全开
////关闭预充电
Uint16 Precharge_Start_Voltage_Addr[2]={0x0A,0x93};//0x930A
Uint16 Precharge_Start_Voltage_Value[2]={0x00,0x00};
Uint16 Precharge_Stop_Voltage_Addr[2]={0x0C,0x93};//0x930C
Uint16 Precharge_Stop_Voltage_Value[2]={0x00,0x00};
///////TS引脚配置   TS1使用默认设置
//TS2 0x0B 热敏电阻温度测量，报告但不用于电芯温度保护
Uint16 TS2_Config_Addr[2]={0xFE,0x92};//0x92FE
Uint16 TS2_Config_Value[1]={0x0B};//
//TS3 0x0B 热敏电阻温度测量，报告但不用于电芯温度保护
Uint16 TS3_Config_Addr[2]={0xFF,0x92};//0x92FF
Uint16 TS3_Config_Value[1]={0x0B};//
//FET控制,子命令，两字节！！！！
Uint16 ALL_FETS_ON_Addr[2]={0x96,0x00};//
Uint16 ALL_FETS_OFF_Addr[2]={0x95,0x00};//
Uint16 DSG_PDSG_OFF_Adde[2]={0x93,0x00};
Uint16 CHG_PCHG_OFF_Adde[2]={0x94,0x00};
//OTP_WR_CHECK 子命令，读取判断是否允许进行OTP写入
Uint16 OTP_WR_CHECK_Addr[2]={0xA0,0x00};//0x00a0
Uint16 OTP_WR_CHECK_Value=0;//检查结果
//进行OTP写入
Uint16 OTP_WRITE_Addr[2]={0xA1,0x00};//0x00A1
Uint16 OTP_WRITE_Value=0;//写入结果
//Uint16 Vcell_Mode[2]={0x80,0x91};//电池节数配置。写入值 0x0203.三节
//Uint16 Vcell_Mode_value[2]={0x7A,0x30};//
//子命令
Uint16  Cells_MaxMinVoltage[2]={0x75,0x00};//0x0075偏移量 最大电芯电压 4 最小电芯电压 6
//直接命令。读命令
Uint16  Safety_Alert_A_Addr=0x02;//A类警告
Uint16  Safety_Status_A_Addr=0x03;//A类故障
Uint16  Safety_Alert_B_Addr=0x04;//B类警告
Uint16  Safety_Status_B_Addr=0x05;//B类故障
Uint16  Safety_Alert_C_Addr=0x06;//C类警告
Uint16  Safety_Status_C_Addr=0x07;//C类故障
//直接命令。读电压和电流
Uint16  Cell1_Voltage_Addr=0x14;//电芯1电压
Uint16  Cell2_Voltage_Addr=0x16;//电芯2电压
Uint16  Cell3_Voltage_Addr=0x18;//电芯3电压
Uint16  Cell4_Voltage_Addr=0x1A;//电芯4电压
Uint16  Cell5_Voltage_Addr=0x1C;//电芯5电压
Uint16  Cell6_Voltage_Addr=0x1E;//电芯6电压
Uint16  Cell7_Voltage_Addr=0x20;//电芯7电压
Uint16  Cell8_Voltage_Addr=0x22;//电芯8电压
Uint16  Cell9_Voltage_Addr=0x24;//电芯9电压
Uint16  Cell10_Voltage_Addr=0x26;//电芯10电压


Uint16  Stack_Voltage_Addr=0x34;//电池组顶部电压
Uint16  PACK_Pin_Voltage_Addr=0x36;//PACK引脚上电压
Uint16  CC2_Current_Addr=0x3A;//CC2 滤波器生成的电流测量值
//直接命令，读温度
Uint16   Int_Temperature_Addr=0x68;//内部温度  0.1K单位
Uint16   TS1_Temperature_Addr=0x70;//TS1温度
Uint16   TS2_Temperature_Addr=0x72;//TS2温度
Uint16   TS3_Temperature_Addr=0x74;//TS3温度
//直接命令，读电池状态
Uint16  Battery_Status_Addr=0x12;//电池状态
//读FET状态，直接命令
Uint16  FET_Status_Addr=0x7F;//FET状态
//永久故障读取，直接命令
Uint16  PF_Alert_A_Addr=0x0A;//
Uint16  PF_Status_A_Addr=0x0B;//
Uint16  PF_Alert_B_Addr=0x0C;//
Uint16  PF_Status_B_Addr=0x0D;//
Uint16  PF_Alert_C_Addr=0x0E;//
Uint16  PF_Status_C_Addr=0x0F;//
Uint16  PF_Alert_D_Addr=0x10;//
Uint16  PF_Status_D_Addr=0x11;//


//写函数的变量
Uint16 Send_Data[16]={0};
Uint16 SendData_length=0;
Uint16 SendReg_address=0;
//读函数的变量
Uint16 Read_Data[16]={0};
Uint16 ReadData_length=0;
Uint16 ReadReg_address=0;
/////////////////////////////////////////////////////////////////////////////////
//电池组
Uint16 Bat_ERR_STATUS=0;
Uint16 Bat_PACK_Voltage=0;
Uint16 Bat_PACK_Current=0;
Uint16 Bat_CELL_MaxVoltage=0;
Uint16 Bat_CELL_MinVoltage=0;
//电容组
Uint16 Cap_ERR_STATUS=0;
Uint16 Cap_PACK_Voltage=0;
Uint16 Cap_PACK_Current=0;
Uint16 Cap_CELL_MaxVoltage=0;
Uint16 Cap_CELL_MinVoltage=0;

/////////////////////////////////////////////////////////////////////////////////



//读写位：0写；1：读
//
// Globals
////发送消息的结构体
struct I2CMSG I2cMsgOut1={ I2C_MSGSTAT_SEND_WITHSTOP,//消息状态，表示发送消息后发送一个停止条件
                           I2C_SLAVE_ADDR,//从设备的地址
                           I2C_NUMBYTES,//发送的数据字节数
                           REG_ADDR,
                           0x00,                   // Msg Byte 1//要发送的两个字节的数据
                           0x00,
                           0x00,
                           0x00
};                  // Msg Byte 2
//接收消息的结构体
struct I2CMSG I2cMsgIn1={ I2C_MSGSTAT_SEND_NOSTOP,//发送消息后不发送停止条件，这通常用于连续发送多个消息
                          I2C_SLAVE_ADDR,
                          I2C_NUMBYTES,
                          REG_ADDR,
                          };

struct I2CMSG *CurrentMsgPtr;//当前的消息指针。


void I2CB_GpioInit(void)
{
GPIO_SetupPinMux(40, GPIO_MUX_CPU1, 6);//SDAB
GPIO_SetupPinMux(41, GPIO_MUX_CPU1, 6);//SCLB

// 配置GPIO工作模式，设置为输入模式
GPIO_SetupPinOptions(40, GPIO_INPUT, GPIO_PULLUP);  // SDA 上拉
GPIO_SetupPinOptions(41, GPIO_INPUT, GPIO_PULLUP);  // SCL 上拉
}

//
// I2CA_Init - Initialize I2CA settings
//
void I2CB_Init(void)
{
    I2cbRegs.I2CSAR.all = 0x0050;     // Slave address - EEPROM control code
//时钟频率的计算公式如下：
//
//    I2C模块时钟（I2C Module Clock） = 系统时钟（SYSCLK） / (I2CPSC + 1)
//    I2C总线时钟（I2C Master Clock） = I2C模块时钟（I2C Module Clock） / (ICCL + d) + (ICCH + d)
//IPSC = 0, d = 7; IPSC = 1, d = 6; IPSC > 1, d = 5.

    I2cbRegs.I2CPSC.all = 19;         // Prescaler - need 7-12 Mhz on module clk
                                      //19=0x13;IPSC=3 >1, d = 5;
    I2cbRegs.I2CCLKL = 10;            // NOTE: must be non zero 400K 10
    I2cbRegs.I2CCLKH = 5;             // NOTE: must be non zero  5
    I2cbRegs.I2CIER.all = 0x24;       // Enable SCD & ARDY __interrupts
//将I2C模块从复位状态中释放出来，并设置了在挂起时停止I2C模块;使能I2C
    I2cbRegs.I2CMDR.all = 0x0020;     // Take I2C out of reset
                                      // Stop I2C when suspended
  //  寄存器I2CMDR的XA位控制地址的位数
 //  I2cbRegs.I2CMDR.bit.XA=0;//0：7位地址；1:10位地址
//启用了TXFIFO（发送FIFO）
    I2cbRegs.I2CFFTX.all = 0x6000;    // Enable FIFO mode and TXFIFO
    I2cbRegs.I2CFFRX.all = 0x2040;    // Enable RXFIFO, clear RXFFINT,
//启用了RXFIFO（接收FIFO）

    //清除接收消息缓冲区
    // Clear incoming message buffer
    //
    int i=0;
        for (i = 0; i < I2C_MAX_BUFFER_SIZE; i++)
        {
            I2cMsgIn1.MsgBuffer[i] = 0x0000;
        }
//开中断
        // Enable I2C __interrupt 1 in the PIE: Group 8 __interrupt 1
        //
        PieCtrlRegs.PIEIER8.bit.INTx3 = 1;//i2cb int8.3
        //

    return;
}

//write func
void Write_Func(void)
{
    Uint16 Error;
    I2cMsgOut1.MsgStatus = I2C_MSGSTAT_SEND_WITHSTOP;
    if(I2cMsgOut1.MsgStatus == I2C_MSGSTAT_SEND_WITHSTOP)
    {
        Error = I2CB_WriteData(&I2cMsgOut1);

        //
        // If communication is correctly initiated, set msg status to busy
        // and update CurrentMsgPtr for the __interrupt service routine.
        // Otherwise, do nothing and try again next loop. Once message is
        // initiated, the I2C __interrupts will handle the rest. Search for
        // ICINTR1A_ISR in the i2c_eeprom_isr.c file.
        //
        if (Error == I2C_SUCCESS)
        {
            CurrentMsgPtr = &I2cMsgOut1;
            //将CurrentMsgPtr指针设置为指向I2cMsgOut1，这样中断服务例程就可以访问当前正在处理的消息
            I2cMsgOut1.MsgStatus = I2C_MSGSTAT_WRITE_BUSY;//表示消息正在忙于写入操作
            COM_flag=2;
        }
    }
}

//write func
void Write_Function(Uint16 reg_address,Uint16 *data,Uint16 data_length)
{
    Uint16 Error;
    int i=0;

    I2cMsgOut1.RegAddr=reg_address;
    I2cMsgOut1.NumOfBytes=data_length;
    for(i=0;i<data_length;i++)
    {
        I2cMsgOut1.MsgBuffer[i]=data[i];
    }

    I2cMsgOut1.MsgStatus = I2C_MSGSTAT_SEND_WITHSTOP;
    if(I2cMsgOut1.MsgStatus == I2C_MSGSTAT_SEND_WITHSTOP)
    {
        Error = I2CB_WriteData(&I2cMsgOut1);

        //
        // If communication is correctly initiated, set msg status to busy
        // and update CurrentMsgPtr for the __interrupt service routine.
        // Otherwise, do nothing and try again next loop. Once message is
        // initiated, the I2C __interrupts will handle the rest. Search for
        // ICINTR1A_ISR in the i2c_eeprom_isr.c file.
        //
        if (Error == I2C_SUCCESS)
        {
            CurrentMsgPtr = &I2cMsgOut1;
            //将CurrentMsgPtr指针设置为指向I2cMsgOut1，这样中断服务例程就可以访问当前正在处理的消息
            I2cMsgOut1.MsgStatus = I2C_MSGSTAT_WRITE_BUSY;//表示消息正在忙于写入操作

        }
    }
}

//数据存储寄存器访问
void  DataMemory_Com_Write(Uint16 *com_address,Uint16 com_len,Uint16 *value,Uint16 val_len )
        {
            Uint16 sum=0;
            Uint16 sum1=0;
            Uint16 sum2=0;
            Uint16 JY_Val=0;
            Uint16 JY_len=0;//数据长度，包括（即缓冲区数据的长度 + 0x3E 和 0x3F 的长度 + 0x60 和 0x61 的长
            //度）

            int i=0;

            Write_Function(0x3E,com_address,com_len);//写入存储器地址
            DELAY_US(1000);//单位us
            Write_Function(0x40,value,val_len);//写入值
            DELAY_US(1000);//单位us
            //计算校验和
            for(i=0;i<com_len;i++)//命令和
            {
                sum1=sum1+com_address[i];
            }

            for(i=0;i<val_len;i++)//值和
            {
                sum2=sum2+value[i];
            }
            sum=sum1+sum2;//和
            JY_Val=~sum;
            JY_Val=__byte(&JY_Val, 0);//低八位按位取反。
            Write_Function(0x60,&JY_Val,1);//写入校验和
            DELAY_US(1000);//单位us
            JY_len=com_len+val_len+2;
            Write_Function(0x61,&JY_len,1);//写入长度
        }


//read func
void Read_Func(void)
{
    I2cMsgIn1.MsgStatus=I2C_MSGSTAT_SEND_NOSTOP;

    while(I2cMsgIn1.MsgStatus != I2C_MSGSTAT_INACTIVE)//
    //if(I2cMsgOut1.MsgStatus == I2C_MSGSTAT_INACTIVE)
    {//检查I2cMsgOut1结构体中的MsgStatus字段，以确定是否可以进行读取操作。
        //如果状态是I2C_MSGSTAT_INACTIVE，表示没有正在进行的写入操作，可以进行读取。
        //
        // Check incoming message status.
        //
       // w_flag++;
        if(I2cMsgIn1.MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)
        {
            //
            // EEPROM address setup portion
            //
            while(I2CB_ReadData(&I2cMsgIn1) != I2C_SUCCESS)
            {//循环尝试发送读取地址到EEPROM，直到操作成功。
                //
                // Maybe setup an attempt counter to break an infinite while
                // loop. The EEPROM will send back a NACK while it is
                // performing a write operation. Even though the write
                // communique is complete at this point, the EEPROM could
                // still be busy programming the data. Therefore, multiple
                // attempts are necessary.
                //
            }

            //更新当前消息指针和消息状态:
            // Update current message pointer and message status
            //
            CurrentMsgPtr = &I2cMsgIn1;
            I2cMsgIn1.MsgStatus = I2C_MSGSTAT_SEND_NOSTOP_BUSY;
        }
        //
        // Once message has progressed past setting up the internal address
        // of the EEPROM, send a restart to read the data bytes from the
        // EEPROM. Complete the communique with a stop bit. MsgStatus is
        // updated in the __interrupt service routine.
        //发送读取地址后，需要发送一个重复开始条件（Restart），以切换I2C总线上的主设备角色从发送者模式到接收者模式。
        else if(I2cMsgIn1.MsgStatus == I2C_MSGSTAT_RESTART)//在中断函数中切换为此
        {
            //
            // Read data portion
            //
            while(I2CB_ReadData(&I2cMsgIn1) != I2C_SUCCESS)
            {
                //
                // Maybe setup an attempt counter to break an infinite while
                // loop.
                //
            }
            //
            // Update current message pointer and message status
            //
            CurrentMsgPtr = &I2cMsgIn1;
            I2cMsgIn1.MsgStatus = I2C_MSGSTAT_READ_BUSY;
        }
    }
}

//read func
int Read_Function(Uint16 reg_address,Uint16 data_length)
{
    I2cMsgIn1.RegAddr=reg_address;
    I2cMsgIn1.NumOfBytes=data_length;
    I2cMsgIn1.MsgStatus=I2C_MSGSTAT_SEND_NOSTOP;

    int loop1=0;
    while(I2cMsgIn1.MsgStatus != I2C_MSGSTAT_INACTIVE)//等待读取成功
    {//检查I2cMsgOut1结构体中的MsgStatus字段，以确定是否可以进行读取操作。
        //如果状态是I2C_MSGSTAT_INACTIVE，表示没有正在进行的写入操作，可以进行读取。
        //
        // Check incoming message status.
        //
        if(loop1++>20000)
            {
            I2C_ERROR_FLAG=3;
            return 0;//最后发送失败
            }

        if(I2cMsgIn1.MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)
        {
            //
            // EEPROM address setup portion
            //
            int loop2=0;
            while(I2CB_ReadData(&I2cMsgIn1) != I2C_SUCCESS)
            {//循环尝试发送读取地址到从机，直到操作成功。
                //
                // Maybe setup an attempt counter to break an infinite while
                // loop. The EEPROM will send back a NACK while it is
                // performing a write operation. Even though the write
                // communique is complete at this point, the EEPROM could
                // still be busy programming the data. Therefore, multiple
                // attempts are necessary.
                //
                if(loop2++>20000)
                    {
                    I2C_ERROR_FLAG=1;
                    return 0;//从机无应答
                    }
            }

            //更新当前消息指针和消息状态:
            // Update current message pointer and message status
            //
            CurrentMsgPtr = &I2cMsgIn1;
            I2cMsgIn1.MsgStatus = I2C_MSGSTAT_SEND_NOSTOP_BUSY;
        }
        //
        // Once message has progressed past setting up the internal address
        // of the EEPROM, send a restart to read the data bytes from the
        // EEPROM. Complete the communique with a stop bit. MsgStatus is
        // updated in the __interrupt service routine.
        //发送读取地址后，需要发送一个重复开始条件（Restart），以切换I2C总线上的主设备角色从发送者模式到接收者模式。
        else if(I2cMsgIn1.MsgStatus == I2C_MSGSTAT_RESTART)//在中断函数中切换为此，因此程序需要在此循环判断
        {
            //
            // Read data portion
            //
            int loop3=0;
            while(I2CB_ReadData(&I2cMsgIn1) != I2C_SUCCESS)
            {
                //
                // Maybe setup an attempt counter to break an infinite while
                // loop.
                //
                if(loop3++>20000)
                    {
                    I2C_ERROR_FLAG=2;
                    return 0;//重复开始失败
                    }
            }
            //
            // Update current message pointer and message status
            //
            CurrentMsgPtr = &I2cMsgIn1;
            I2cMsgIn1.MsgStatus = I2C_MSGSTAT_READ_BUSY;
        }
    }
    I2C_ERROR_FLAG=0;
    return 0;
}

//
// I2CA_WriteData - Transmit I2CA message
//
Uint16 I2CB_WriteData(struct I2CMSG *msg)
{
    Uint16 i;

    //
    // Wait until the STP bit is cleared from any previous master communication.
    // Clearing of this bit by the module is delayed until after the SCD bit is
    // set. If this bit is not checked prior to initiating a new message, the
    // I2C could get confused.
    //等待STP位清除.如果STP位为1，则返回一个错误码,表示I2C模块还没有准备好发送新的数据
    // STP位在发送完数据后会清零
    if(I2cbRegs.I2CMDR.bit.STP == 1)
    {
        return I2C_STP_NOT_READY_ERROR;
    }

    //
    // Setup slave address
    //设置从设备地址
    I2cbRegs.I2CSAR.all = msg->SlaveAddress;

    //
    // Check if bus busy
    //检查I2C状态寄存器中的BB（总线忙）位是否为1
    if(I2cbRegs.I2CSTR.bit.BB == 1)
    {
        return I2C_BUS_BUSY_ERROR;
    }

    //
    // Setup number of bytes to send
    // MsgBuffer + Address
    //设置要发送的字节数，包括了寄存器地址
    I2cbRegs.I2CCNT = msg->NumOfBytes+1;
//写入之前先把FIFO指针清零。
     I2cbRegs.I2CFFTX.bit.TXFFRST = 0;    //复位 TXFIFO 。指针复位到0000。
     I2cbRegs.I2CFFTX.bit.TXFFRST=1;    //使能 FIFO
    //
    // Setup data to send
    //
 //   I2cbRegs.I2CDXR.all = msg->MemoryHighAddr;//发送EEPROM的高地址
    I2cbRegs.I2CDXR.all = msg->RegAddr;//发送寄存器地址

    for (i=0; i < msg->NumOfBytes; i++)
    {
        I2cbRegs.I2CDXR.all = *(msg->MsgBuffer+i);//将消息缓冲区中的每个字节写入到I2C数据寄存器中，以便发送
    }//这里I2C工作在FIFO队列（先进先出队列）模式，不会覆盖。

    //
    // Send start as master transmitter
    //发送开始条件作为主发送器
    // FREE、STT、STP、MST、TRX、IRS
    I2cbRegs.I2CMDR.all = 0x6E20;//I2CMDR:I2C模式配置寄存器

    return I2C_SUCCESS;
}

//
// I2CA_ReadData - Read I2CA Message
//
Uint16 I2CB_ReadData(struct I2CMSG *msg)
{
    //
    // Wait until the STP bit is cleared from any previous master communication.
    // Clearing of this bit by the module is delayed until after the SCD bit is
    // set. If this bit is not checked prior to initiating a new message, the
    // I2C could get confused.
    //等待STP位清除
    if(I2cbRegs.I2CMDR.bit.STP == 1)
    {
        return I2C_STP_NOT_READY_ERROR;
    }
   // 设置从设备地址
    I2cbRegs.I2CSAR.all = msg->SlaveAddress;
   //处理不发送STOP信号的情况
    if(msg->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP)//不发送STOP信号
    {
        //
        // Check if bus busy
        //检查总线忙
        if(I2cbRegs.I2CSTR.bit.BB == 1)
        {
            return I2C_BUS_BUSY_ERROR;
        }
        I2cbRegs.I2CCNT = 1;//设置计数寄存器
//写入寄存器地址之前一定要先清零发送FIFO指针
        I2cbRegs.I2CFFTX.bit.TXFFRST = 0;    //复位 TXFIFO 。指针复位到0000。
        I2cbRegs.I2CFFTX.bit.TXFFRST=1;    //使能 FIFO
       // I2cbRegs.I2CDXR.all = msg->MemoryHighAddr;
        I2cbRegs.I2CDXR.all = msg->RegAddr;//寄存器地址.发送寄存器的地址。
        I2cbRegs.I2CMDR.all = 0x2620; // Send data to setup EEPROM address//这里比前面发送函数少了一个停止位
    }//发送START条件，设置为发送器模式，不发送STOP信号。
    // dsp发送内存地址后，状态切换为：I2C_MSGSTAT_RESTART，开始接收数据
    else if(msg->MsgStatus == I2C_MSGSTAT_RESTART)
    {
        I2cbRegs.I2CCNT = msg->NumOfBytes;    // Setup how many bytes to expect//设置计数寄存器，表示要接收的字节数
        I2cbRegs.I2CMDR.all = 0x2C20;         // Send restart as master receiver//发送RESTART条件，设置为接收器模式。
    }

    return I2C_SUCCESS;
}


// 函数用于将16位补码转换为原始负数值
int convertTwosComplementToDecimal(int twosComplement) {
    // 检查最高位是否为1（负数）
    if (twosComplement & 0x8000) {
        // 如果是负数，则进行补码转换
        return -((~twosComplement + 1) & 0xFFFF);
    } else {
        // 如果是正数，直接返回
        return twosComplement;
    }
}


/////////////////读取命令 //////////////////
/////////////////读取警告和故障信息//////////////////////////////////
void Read_BMS_Information(int flag)
{
//0x08:电池板  0x20：电容板
      if(flag==100)
        {
        I2cMsgOut1.SlaveAddress=0x20;//更新地址 电容板
        I2cMsgIn1.SlaveAddress=0x20;
        COM_flag=200;//切换设备，供下一次读取
        }
        else
        {
            I2cMsgOut1.SlaveAddress=0x08;//更新地址 电池板
            I2cMsgIn1.SlaveAddress=0x08;
            COM_flag=100;//切换设备，供下一次读取
        }
//        Read_Function(Safety_Alert_A_Addr,1);//读取A类警告
//        Safety_Alert_A=I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//延时1ms
        Read_Function(Safety_Status_A_Addr,1);//读取A类故障
        Safety_Status_A=I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//延时1ms

    /////////////////读FET状态
//        Read_Function(FET_Status_Addr,1);//读取FET状态
//        FET_Status=I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//延时1ms

    ////////////读电压与电流///////////////////////////////////////////////
//
//        Read_Function(Cell1_Voltage_Addr,2);
//        Cell1_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//延时1ms
//        Read_Function(Cell2_Voltage_Addr,2);
//        Cell2_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//延时1ms
//        Read_Function(Cell3_Voltage_Addr,2);
//        Cell3_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//延时1ms
//        Read_Function(Cell4_Voltage_Addr,2);
//        Cell4_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//延时1ms
//        Read_Function(Cell5_Voltage_Addr,2);
//        Cell5_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//延时1ms
//        Read_Function(Cell6_Voltage_Addr,2);
//        Cell6_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//延时1ms
//        Read_Function(Cell7_Voltage_Addr,2);
//        Cell7_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//延时1ms
//        Read_Function(Cell8_Voltage_Addr,2);
//        Cell8_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//延时1ms
//        Read_Function(Cell9_Voltage_Addr,2);
//        Cell9_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//延时1ms
//        Read_Function(Cell10_Voltage_Addr,2);
//        Cell10_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//延时1ms
        //电池组顶部电压 单位10mv
        Read_Function(Stack_Voltage_Addr,2);
        Stack_Voltage_mv=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        DELAY_US(1000);//延时1ms
        //电流
        Read_Function(CC2_Current_Addr,2);
        CC2_Current_mA=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        CC2_Current_mA_Real=convertTwosComplementToDecimal(CC2_Current_mA);
        DELAY_US(1000);//延时1ms
    ////////////读温度///////////////////////////////////////////////
        Read_Function(Int_Temperature_Addr,2);
        Int_Temperature_Value=convertTwosComplementToDecimal((I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0])*0.1-273.15;
        DELAY_US(1000);//延时1ms

//        Read_Function(TS1_Temperature_Addr,2);
//        TS1_Temperature_Value=convertTwosComplementToDecimal((I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0])*0.1-273.15;
//        DELAY_US(1000);//延时1ms
//        Read_Function(TS2_Temperature_Addr,2);
//        TS2_Temperature_Value=convertTwosComplementToDecimal((I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0])*0.1-273.15;
//        DELAY_US(1000);//延时1ms
//        Read_Function(TS3_Temperature_Addr,2);
//        TS3_Temperature_Value=convertTwosComplementToDecimal((I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0])*0.1-273.15;
//        DELAY_US(1000);//延时1ms
    ////////////读主动均衡的电池   子命令读取///////////////////////////////////////////////
//        Write_Function(0x3E,CB_ACTIVE_CELLSA_Addr,2);//
//        DELAY_US(10000);//延时10ms
//        Read_Function(0x40,2);
//        CB_ACTIVE_CELLS=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
    ///////////读最大最小电压   子命令读取///////////////////////////////////////////////
        Write_Function(0x3E,Cells_MaxMinVoltage,2);//
        DELAY_US(10000);//延时10ms
        Read_Function(0x40,8);//读8个字节
        Cells_MaxVoltage_mv=(I2cMsgIn1.MsgBuffer[5]<<8)+I2cMsgIn1.MsgBuffer[4];
        Cells_MinVoltage_mv=(I2cMsgIn1.MsgBuffer[7]<<8)+I2cMsgIn1.MsgBuffer[6];
    ////////////读电池状态///////////////////////////////////////////////

//        DELAY_US(1000);//延时1ms
//        Read_Function(Battery_Status_Addr,2);
//        Battery_Status=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
//        DELAY_US(1000);//延时1ms
//        //读FET控制模式FET_ControlMODE
//        Write_Function(0x3E,FET_ControlMODE_Addr,2);//读回过压阈值
//        DELAY_US(10000);//单位us。等待从机准备数据
//        Read_Function(0x40,2);//从从机的数据缓冲区读数据
//        FET_ControlMODE=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];

        //读取的信息传递
        //电池组
        if(flag==100)//电容板
        {
            //电容组
             Cap_ERR_STATUS=Safety_Status_A;
             Cap_PACK_Voltage=Stack_Voltage_mv;
             Cap_PACK_Current=CC2_Current_mA_Real;
             Cap_CELL_MaxVoltage=Cells_MaxVoltage_mv;
             Cap_CELL_MinVoltage=Cells_MinVoltage_mv;
        }
        else
        {
         Bat_ERR_STATUS=Safety_Status_A;
         Bat_PACK_Voltage=Stack_Voltage_mv;
         Bat_PACK_Current=CC2_Current_mA_Real;
         Bat_CELL_MaxVoltage=Cells_MaxVoltage_mv;
         Bat_CELL_MinVoltage=Cells_MinVoltage_mv;
        }

}

///////////////快速读取电容电流///////////////////////////////
void Read_Cap_Current(void)
{
//0x08:电池板  0x20：电容板
        I2cMsgOut1.SlaveAddress=0x20;//更新地址 电容板
        I2cMsgIn1.SlaveAddress=0x20;
        //电流
        Read_Function(CC2_Current_Addr,2);
        CC2_Current_mA=(I2cMsgIn1.MsgBuffer[1]<<8)+I2cMsgIn1.MsgBuffer[0];
        CC2_Current_mA_Real=convertTwosComplementToDecimal(CC2_Current_mA);
        Cap_PACK_Current=CC2_Current_mA_Real;
}



//
// i2c_int1a_isr - I2CA ISR
//中断服务程序会检查CurrentMsgPtr指向的消息结构体的MsgStatus字段，以确定当前操作的状态
__interrupt void i2c_int1a_isr(void)
{
    Uint16 IntSource, i;
    //
    // Read __interrupt source
    //
    IntSource = I2cbRegs.I2CISRC.all;//中断源

    //
    // Interrupt source = stop condition detected
    //检查是否检测到了停止条件中断
    //在I2C协议中，停止条件是由主设备生成的，用来通知总线上的所有设备通信即将结束。
    //停止条件的特点是SCL（时钟线）为高电平期间SDA（数据线）从低电平变为高电平。
    if(IntSource == I2C_SCD_ISRC)//STOP//先判断停止条件！！！！表明当前写、读操作已经结束
    {
        //
        // If completed message was writing data, reset msg to inactive state
        //如果当前消息状态是I2C_MSGSTAT_WRITE_BUSY（正在写入），则将其重置为非活动状态
        if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_WRITE_BUSY)
        {
            CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;//写完成

        }
        else
        {
            //
            // If a message receives a NACK during the address setup portion of
            // the EEPROM read, the code further below included in the register
            // access ready __interrupt source code will generate a stop
            // condition. After the stop condition is received (here), set the
            // message status to try again. User may want to limit the number of
            // retries before generating an error.

            //如果当前消息状态是I2C_MSGSTAT_SEND_NOSTOP_BUSY（正在发送地址，不停止），
            //则将其重置为准备发送状态（I2C_MSGSTAT_SEND_NOSTOP），以便重试。
            if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
            {
                CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_SEND_NOSTOP;
            }
            //
            // If completed message was reading EEPROM data, reset msg to
            // inactive state and read data from FIFO.
            //
            //如果当前消息状态是I2C_MSGSTAT_READ_BUSY（正在读取），则将其重置为非活动状态，
            //并从FIFO中读取数据。
            else if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_READ_BUSY)//读完成
            {
                CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_INACTIVE;

                for(i=0; i < I2cMsgIn1.NumOfBytes; i++)
                {
                    CurrentMsgPtr->MsgBuffer[i] = I2cbRegs.I2CDRR.all;
                }

               // COM_flag=3;//读完成
                I2cbRegs.I2CFFRX.bit.RXFFRST = 0; //复位 RXFIFO
                I2cbRegs.I2CFFRX.bit.RXFFRST = 1;    // Enable RXFIFO, clear RXFFINT

            }
        }
    }

    //
    // Interrupt source = Register Access Ready
    // This __interrupt is used to determine when the EEPROM address setup
    // portion of the read data communication is complete. Since no stop bit is
    // commanded, this flag tells us when the message has been sent instead of
    // the SCD flag. If a NACK is received, clear the NACK bit and command a
    // stop. Otherwise, move on to the read data portion of the communication.
    //处理寄存器访问就绪中断
    else if(IntSource == I2C_ARDY_ISRC)
    {
        if(I2cbRegs.I2CSTR.bit.NACK == 1)//如果检测到NACK,则发送停止条件并清除NACK位。
        {
            I2cbRegs.I2CMDR.bit.STP = 1;
            I2cbRegs.I2CSTR.all = I2C_CLR_NACK_BIT;
        }
        else if(CurrentMsgPtr->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP_BUSY)
        {
            CurrentMsgPtr->MsgStatus = I2C_MSGSTAT_RESTART;//更新为准备重启状态,以便进行数据读取
        }
        invalid_interrupt_flag=0;
    }
    else
    {
        //处理无效中断源:
        // Generate some error due to invalid __interrupt source
        //
//        __asm("   ESTOP0");
        invalid_interrupt_flag=1;
    }

    //
    // Enable future I2C (PIE Group 8) __interrupts
    //重新启用I2C中断
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;

   // 在中断处理程序中可以通过I2caRegs.I2CMDR.bit.IRS位清零从而复位I2CSTR状态寄存器，实现I2C模块的复位。
}

