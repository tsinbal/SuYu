#ifndef __NIMOTION_H
#define __NIMOTION_H
#include "stm32f10x.h"
#include "can.h"
#include "task.h"
typedef struct {
    uint8_t Nimotion_Addr;  //设备地址
    uint8_t Nimotion_Status; //设备状态
    uint32_t Nimotion_Position; //设备当前位置
    uint32_t Nimotion_Zero_Position;//设备初始位置
    uint32_t Nimotion_Max_Speed; //设备最大速度
    uint32_t Nimotion_Acc; //设备加速度
} Nimotion_InitTypeDef;
typedef enum {
    Nimotion_SDO_Mode_Read = 0x01,
    Nimotion_SDO_Mode_Write = 0x02,
		Nimotion_SDO_Mode_SelfCheck = 0x03
} Nimotion_SDO_Mode_TypeDef;  //SDO 服务类型，读或者写，自检
typedef enum {
    Nimotion_Move_Mode = 0x01, //运行的模式，value值为1位置模式,3速度模式,6原点回归模式
    Nimotion_Move_Acc = 0x07, //运行加速度
    Nimotion_Move_Max_Seed = 0x18, //运行最大速度
    Nimotion_Sync_RPDO3 = 0x1e, //RPDO3传输方式，同步异步
    Nimotion_Sync_TPDO3 = 0x1f,  //TPDO3传输方式，同步异步
		Nimotion_Interval_PDO3 = 0x8b,
		Nimotion_PDO_Mode = 0x80

} Nimotion_SDO_Index_TypeDef; //SDO 索引类型，见PDF
typedef enum{
	Nimotion_StateMachine_StartUp = 0x06, //启动状态
	Nimotion_StateMachine_MotorEnable = 0x07, //电机使能
	Nimotion_StateMachine_Velocity_Run = 0x0F, //Speed Mode Run
	Nimotion_StateMachine_Position_GotoCommand = 0x0F, //运行Goto指令
	Nimotion_StateMachine_Position_GotoPosition = 0x1F, //发送Goto位置
	Nimotion_StateMachine_Position_MoveCommand = 0x4F, //运行Move指令
	Nimotion_StateMachine_Position_MovePosition = 0x5F //发送新位置
}Nimotion_StateMachine_TypeDef; //状态机


#define Nimotion_SDO_Addr 0x600
#define Nimotion_SDO_Mode_Pos 0x00
#define Nimotion_SDO_Index_Pos 0x01
#define Nimotion_SDO_ValueLow_Pos 0x04
#define Nimotion_SDO_ValueMidLow_Pos 0x05
#define Nimotion_SDO_ValueMidHigh_Pos 0x06
#define Nimotion_SDO_ValueHigh_Pos 0x07


#define Nimotion_TPDO1_Addr 0x200
#define Nimotion_TPDO2_Addr 0x300
#define Nimotion_TPDO3_Addr 0x400
#define Nimotion_TPDO4_Addr 0x500

#define Nimotion_TPDO_CtrlWordLow_Pos 0x00
#define Nimotion_TPDO_CtrlWordHigh_Pos 0x01
#define Nimotion_TPDO_Dir_Pos 0x03
#define Nimotion_TPDO_ValueLow_Pos 0x04
#define Nimotion_TPDO_ValueMidLow_Pos 0x05
#define Nimotion_TPDO_ValueMidHigh_Pos 0x06
#define Nimotion_TPDO_ValueHigh_Pos 0x07
void Nimotion_Init();
void Nimotion_SDO_SendValue(Nimotion_SDO_Mode_TypeDef Mode_Structure, Nimotion_SDO_Index_TypeDef Index_Structure, uint32_t value);
void Nimotion_Position_SendValue(Nimotion_StateMachine_TypeDef Control_Word,uint8_t Direction,uint32_t Position);
void Nimotion_Velocity_SendValue(Nimotion_StateMachine_TypeDef Control_Word,uint8_t Direction,uint32_t velocity);

#endif


