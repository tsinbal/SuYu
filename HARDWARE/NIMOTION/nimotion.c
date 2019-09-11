#include "nimotion.h"

u8 Nimotion_SendBuf[8];



void Nimotion_Delay_Ms(uint32_t delay)
{
	 vTaskDelay(delay);

}
void Nimotion_SDO_Transfer(uint8_t *Send_Buf,Nimotion_InitTypeDef *Nimotion_InitStructure)
{
    Can_Send_Msg(Send_Buf,8,Nimotion_SDO_Addr+Nimotion_InitStructure->Nimotion_Addr);
}
void Nimotion_TPDO3_Transfer(uint8_t *Send_Buf,Nimotion_InitTypeDef *Nimotion_InitStructure)
{
    Can_Send_Msg(Send_Buf,8,Nimotion_TPDO3_Addr+Nimotion_InitStructure->Nimotion_Addr);
}


Nimotion_InitTypeDef Nimotion_InitStructure;
void Nimotion_Init(uint32_t Zero_Position)
{
    Nimotion_InitStructure.Nimotion_Addr = 1;
		Nimotion_InitStructure.Nimotion_Zero_Position = Zero_Position;
    Nimotion_SDO_SendValue(Nimotion_SDO_Mode_Read, Nimotion_Move_Mode, 0);
		Nimotion_Delay_Ms(50);
	  Nimotion_SDO_SendValue(Nimotion_SDO_Mode_Write,Nimotion_Sync_RPDO3,0xff);
		Nimotion_Delay_Ms(50);
		Nimotion_SDO_SendValue(Nimotion_SDO_Mode_Write,Nimotion_Sync_TPDO3,0xff);
		Nimotion_Delay_Ms(50);
		Nimotion_SDO_SendValue(Nimotion_SDO_Mode_Write,Nimotion_Move_Acc,0xffff);
		Nimotion_Delay_Ms(50);
		Nimotion_SDO_SendValue(Nimotion_SDO_Mode_Write,Nimotion_Move_Max_Seed,0xffff);
		Nimotion_Delay_Ms(50);
	  Nimotion_Position_SendValue(Nimotion_StateMachine_StartUp,0,0);
	  Nimotion_Delay_Ms(50);
		Nimotion_SDO_SendValue(Nimotion_SDO_Mode_Read, Nimotion_Move_Mode, 1); //λ��ģʽ
		Nimotion_Delay_Ms(50);
	  Nimotion_Position_SendValue(Nimotion_StateMachine_MotorEnable,0,0);
	  Nimotion_Delay_Ms(50);
		Nimotion_Position_SendValue(Nimotion_StateMachine_GotoCommand,0,0);
	  Nimotion_Delay_Ms(50);
		Nimotion_Position_SendValue(Nimotion_StateMachine_GotoPosition,0,Zero_Position);
	  Nimotion_Delay_Ms(50);

}


void Nimotion_SDO_SendValue(Nimotion_SDO_Mode_TypeDef Mode_Structure, Nimotion_SDO_Index_TypeDef Index_Structure, uint32_t value)
{
    Nimotion_SendBuf[Nimotion_SDO_Mode_Pos] = Mode_Structure;
    Nimotion_SendBuf[Nimotion_SDO_Index_Pos] = Index_Structure;
    Nimotion_SendBuf[2] = 0;
    Nimotion_SendBuf[3] = 0;
    Nimotion_SendBuf[Nimotion_SDO_ValueLow_Pos] = value & 0xff;
    Nimotion_SendBuf[Nimotion_SDO_ValueMidLow_Pos] = (value>>8) & 0xff;
    Nimotion_SendBuf[Nimotion_SDO_ValueMidHigh_Pos] = (value>>16) & 0xff;
    Nimotion_SendBuf[Nimotion_SDO_ValueHigh_Pos] = (value>>24) & 0xff;
    Nimotion_SDO_Transfer(Nimotion_SendBuf,&Nimotion_InitStructure);
}
void Nimotion_Position_SendValue(Nimotion_StateMachine_TypeDef Control_Word,uint8_t Direction,uint32_t Position)
{
	Nimotion_SendBuf[Nimotion_TPDO_CtrlWordLow_Pos] = Control_Word && 0xff;
	Nimotion_SendBuf[Nimotion_TPDO_CtrlWordHigh_Pos] = (Control_Word>>8) && 0xff;
	Nimotion_SendBuf[2] = 0;
  Nimotion_SendBuf[3] = Direction;
	Nimotion_SendBuf[Nimotion_TPDO_ValueLow_Pos] = Position & 0xff;
  Nimotion_SendBuf[Nimotion_TPDO_ValueMidLow_Pos] = (Position>>8) & 0xff;
  Nimotion_SendBuf[Nimotion_TPDO_ValueMidHigh_Pos] = (Position>>16) & 0xff;
  Nimotion_SendBuf[Nimotion_TPDO_ValueHigh_Pos] = (Position>>24) & 0xff;
	Nimotion_TPDO3_Transfer(Nimotion_SendBuf,&Nimotion_InitStructure);
}




