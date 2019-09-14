#ifndef __NIMOTION_H
#define __NIMOTION_H
#include "stm32f10x.h"
#include "can.h"
#include "task.h"
typedef struct {
    uint8_t Nimotion_Addr;  //�豸��ַ
    uint8_t Nimotion_Status; //�豸״̬
    uint32_t Nimotion_Position; //�豸��ǰλ��
    uint32_t Nimotion_Zero_Position;//�豸��ʼλ��
    uint32_t Nimotion_Max_Speed; //�豸����ٶ�
    uint32_t Nimotion_Acc; //�豸���ٶ�
} Nimotion_InitTypeDef;
typedef enum {
    Nimotion_SDO_Mode_Read = 0x01,
    Nimotion_SDO_Mode_Write = 0x02,
		Nimotion_SDO_Mode_SelfCheck = 0x03
} Nimotion_SDO_Mode_TypeDef;  //SDO �������ͣ�������д���Լ�
typedef enum {
    Nimotion_Move_Mode = 0x01, //���е�ģʽ��valueֵΪ1λ��ģʽ,3�ٶ�ģʽ,6ԭ��ع�ģʽ
    Nimotion_Move_Acc = 0x07, //���м��ٶ�
    Nimotion_Move_Max_Seed = 0x18, //��������ٶ�
    Nimotion_Sync_RPDO3 = 0x1e, //RPDO3���䷽ʽ��ͬ���첽
    Nimotion_Sync_TPDO3 = 0x1f,  //TPDO3���䷽ʽ��ͬ���첽
		Nimotion_Interval_PDO3 = 0x8b,
		Nimotion_PDO_Mode = 0x80

} Nimotion_SDO_Index_TypeDef; //SDO �������ͣ���PDF
typedef enum{
	Nimotion_StateMachine_StartUp = 0x06, //����״̬
	Nimotion_StateMachine_MotorEnable = 0x07, //���ʹ��
	Nimotion_StateMachine_Velocity_Run = 0x0F, //Speed Mode Run
	Nimotion_StateMachine_Position_GotoCommand = 0x0F, //����Gotoָ��
	Nimotion_StateMachine_Position_GotoPosition = 0x1F, //����Gotoλ��
	Nimotion_StateMachine_Position_MoveCommand = 0x4F, //����Moveָ��
	Nimotion_StateMachine_Position_MovePosition = 0x5F //������λ��
}Nimotion_StateMachine_TypeDef; //״̬��


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


