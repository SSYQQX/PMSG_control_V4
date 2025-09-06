#ifndef BSP_I2C_H_
#define BSP_I2C_H_
#include "F28x_Project.h"

//
// Defines
//
#define I2C_SLAVE_ADDR        0x08//从设备的地址。7位地址;8位地址为0x10
#define I2C_NUMBYTES          4  //数据字节数
#define REG_ADDR             0x66//

extern int COM_flag;//读写设备号
extern int I2C_ERROR_FLAG;//I2C故障标志
extern int COM_Allow;//I2C访问控制位，=2时通信

extern struct I2CMSG I2cMsgOut1;//声名
extern struct I2CMSG I2cMsgIn1;
extern struct I2CMSG *CurrentMsgPtr;
//字节计数
extern Uint16 PassCount;
extern Uint16 FailCount;
// Function Prototypes
//当I2C总线处于空闲状态时（没有数据传输），SCL和SDA都保持高电平。
/////////////////I2C////////////////////
__interrupt void i2c_int1a_isr(void);
void I2CB_GpioInit(void);
void   I2CB_Init(void);
Uint16 I2CB_WriteData(struct I2CMSG *msg);
Uint16 I2CB_ReadData(struct I2CMSG *msg);

void Write_Func(void);
void Read_Func(void);

void  DataMemory_Com_Write(Uint16 *com_address,Uint16 com_len,Uint16 *value,Uint16 val_len );
void Write_Function(Uint16 reg_address,Uint16 *data,Uint16 data_length);
int Read_Function(Uint16 reg_address,Uint16 data_length);

void Read_BMS_Information(int flag);//读BMS信息
void Read_Cap_Current(void);//////快速读取电容电流
// 函数用于将16位补码转换为原始负数值
int convertTwosComplementToDecimal(int complement);

//BMS 信息变量
extern Uint16  FET_Status;//FET状态  ALRT_PIN DDSG_PIN DCHG_PIN PDSG_FET DSG_FET PCHG_FET CHG_FET
extern Uint16 Safety_Alert_A;//A类警告
extern Uint16 Safety_Status_A;//A类故障
extern Uint16 Safety_Alert_B;//B类警告
extern Uint16 Safety_Status_B;//B类故障
extern Uint16 Safety_Alert_C;//C类警告
extern Uint16 Safety_Status_C;//C类故障

extern Uint16  PF_Alert_A;//
extern Uint16  PF_Status_A;//
extern Uint16  PF_Alert_B;//
extern Uint16  PF_Status_B;//
extern Uint16  PF_Alert_C;//
extern Uint16  PF_Status_C;//
extern Uint16  PF_Alert_D;//
extern Uint16  PF_Status_D;//
//电压电流
extern Uint16  Cell1_Voltage_mv;//电芯1电压
extern Uint16  Cell2_Voltage_mv;//电芯2电压
extern Uint16  Cell3_Voltage_mv;//电芯3电压
extern Uint16  Cell4_Voltage_mv;//电芯4电压
extern Uint16  Cell5_Voltage_mv;//电芯5电压
extern Uint16  Cell6_Voltage_mv;//电芯6电压
extern Uint16  Cell7_Voltage_mv;//电芯7电压
extern Uint16  Cell8_Voltage_mv;//电芯8电压
extern Uint16  Cell9_Voltage_mv;//电芯9电压
extern Uint16  Cell10_Voltage_mv;//电芯10电压
extern Uint16  Stack_Voltage_mv;//电池组顶部电压
extern Uint16  PACK_Pin_Voltage_mv;//PACK引脚上电压
extern Uint16  CC2_Current_mA;//CC2 滤波器生成的电流测量值
extern int16  CC2_Current_mA_Real;//CC2 滤波器生成的电流测量值
//温度测量值
extern int16 Int_Temperature_Value;//内部温度  单位0.1k
extern int16 TS1_Temperature_Value;//Ts1温度
extern int16 TS2_Temperature_Value;//Ts2温度
extern int16 TS3_Temperature_Value;//Ts3温度
//电池均衡
extern int16 CB_ACTIVE_CELLS;//
//电池状态
extern Uint16 Battery_Status;

extern Uint16 I2C_MODE_Val;//I2C 模式
extern Uint16 Vcell_Num;//电池节数
extern Uint16 COV_Threshold_mv;//过压保护阈值
extern Uint16 CUV_Threshold_mv;//过压保护阈值
extern Uint16 SCD_Threshold_A;//放电短路保护阈值
extern int OCC_Threshold_A;//充电过流保护阈值
extern Uint16 OCD1_Threshold_A;//放电过流保护阈值1
extern int OCD2_Threshold_A;//放电过流保护阈值2
extern Uint16 FET_ControlMODE;//FET控制模式
extern Uint16 FET_Options;//地址 0x9308，默认值 0x0D。。FET控制模式。串并联设置等
extern Uint16 Protections_A;//启用各种保护
extern Uint16 Manufacturing_Status_Value;

//电池组
extern Uint16 Bat_ERR_STATUS;
//电容组
extern Uint16 Cap_ERR_STATUS;

#endif
