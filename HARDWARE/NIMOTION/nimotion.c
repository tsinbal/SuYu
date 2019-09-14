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
void Nimotion_TPDO2_Transfer(uint8_t *Send_Buf,Nimotion_InitTypeDef *Nimotion_InitStructure)
{
    Can_Send_Msg(Send_Buf,8,Nimotion_TPDO2_Addr+Nimotion_InitStructure->Nimotion_Addr);
}

Nimotion_InitTypeDef Nimotion_InitStructure;
Nimotion_InitTypeDef Driver_InitStructure;
void Nimotion_Init(uint32_t Zero_Position)
{
    Nimotion_InitStructure.Nimotion_Addr = 1;
		Driver_InitStructure.Nimotion_Addr = 2;
		Nimotion_InitStructure.Nimotion_Zero_Position = Zero_Position;
    Nimotion_SDO_SendValue(Nimotion_SDO_Mode_Read, Nimotion_Move_Mode, 0);
		Nimotion_Delay_Ms(100);
	  Nimotion_SDO_SendValue(Nimotion_SDO_Mode_Write,Nimotion_Sync_RPDO3,0xff);
		Nimotion_Delay_Ms(100);
		Nimotion_SDO_SendValue(Nimotion_SDO_Mode_Write,Nimotion_Sync_TPDO3,0xff);
		Nimotion_Delay_Ms(100);
		Nimotion_SDO_SendValue(Nimotion_SDO_Mode_Write,Nimotion_Move_Acc,0xffff);
		Nimotion_Delay_Ms(100);
		Nimotion_SDO_SendValue(Nimotion_SDO_Mode_Write,Nimotion_Move_Max_Seed,0xffff);
		Nimotion_Delay_Ms(100);
		Nimotion_SDO_SendValue(Nimotion_SDO_Mode_Write,Nimotion_PDO_Mode,0x04);
		Nimotion_Delay_Ms(200);
		Nimotion_SDO_SendValue(Nimotion_SDO_Mode_Read,Nimotion_Interval_PDO3,0x0a);
		Nimotion_Delay_Ms(200);
	
	
	
	
	  Nimotion_Position_SendValue(Nimotion_StateMachine_StartUp,0,0);
	  Nimotion_Delay_Ms(100);
		Nimotion_SDO_SendValue(Nimotion_SDO_Mode_Read, Nimotion_Move_Mode, 1); //1Î»ÖÃÄ£Ê½ 3 Speed mode
		Nimotion_Delay_Ms(100);
	  Nimotion_Position_SendValue(Nimotion_StateMachine_MotorEnable,0,0);
	  Nimotion_Delay_Ms(100);
		Nimotion_Position_SendValue(Nimotion_StateMachine_Position_GotoCommand,0,0);
	  Nimotion_Delay_Ms(100);
		Nimotion_Position_SendValue(Nimotion_StateMachine_Position_GotoPosition,0,Zero_Position);
		printf("nimotion inited\r\n");
	  Nimotion_Delay_Ms(3000);
		
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
	Nimotion_SendBuf[Nimotion_TPDO_CtrlWordLow_Pos] = Control_Word & 0xff;
	Nimotion_SendBuf[Nimotion_TPDO_CtrlWordHigh_Pos] = (Control_Word>>8) & 0xff;
	Nimotion_SendBuf[2] = 0;
  Nimotion_SendBuf[3] = Direction;
	Nimotion_SendBuf[Nimotion_TPDO_ValueLow_Pos] = Position & 0xff;
  Nimotion_SendBuf[Nimotion_TPDO_ValueMidLow_Pos] = (Position>>8) & 0xff;
  Nimotion_SendBuf[Nimotion_TPDO_ValueMidHigh_Pos] = (Position>>16) & 0xff;
  Nimotion_SendBuf[Nimotion_TPDO_ValueHigh_Pos] = (Position>>24) & 0xff;
	Nimotion_TPDO3_Transfer(Nimotion_SendBuf,&Nimotion_InitStructure);
}
void Nimotion_Velocity_SendValue(Nimotion_StateMachine_TypeDef Control_Word,uint8_t Direction,uint32_t velocity)
{
	Nimotion_SendBuf[Nimotion_TPDO_CtrlWordLow_Pos] = Control_Word & 0xff;
	Nimotion_SendBuf[Nimotion_TPDO_CtrlWordHigh_Pos] = (Control_Word>>8) & 0xff;
	Nimotion_SendBuf[2] = 0;
  Nimotion_SendBuf[3] = Direction;
	Nimotion_SendBuf[Nimotion_TPDO_ValueLow_Pos] = velocity & 0xff;
  Nimotion_SendBuf[Nimotion_TPDO_ValueMidLow_Pos] = (velocity>>8) & 0xff;
  Nimotion_SendBuf[Nimotion_TPDO_ValueMidHigh_Pos] = (velocity>>16) & 0xff;
  Nimotion_SendBuf[Nimotion_TPDO_ValueHigh_Pos] = (velocity>>24) & 0xff;
	Nimotion_TPDO2_Transfer(Nimotion_SendBuf,&Nimotion_InitStructure);
}

void Nimotion_Driver_SendValue(u8 left_dir,u8 right_dir,u8 left_speed,u8 right_speed)
{
	Nimotion_SendBuf[0] = 0;
	Nimotion_SendBuf[1] = 0;
	Nimotion_SendBuf[2] = 0;
  Nimotion_SendBuf[3] = left_dir | (right_dir<<1);
	Nimotion_SendBuf[4] = right_speed;
  Nimotion_SendBuf[5] = 0;
  Nimotion_SendBuf[6] = left_speed;
  Nimotion_SendBuf[7] = 0;
	Nimotion_TPDO2_Transfer(Nimotion_SendBuf,&Driver_InitStructure);
}



