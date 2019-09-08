#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"

#include "ajsr.h"
#include "adc.h"
#include "dac.h"
#include "stmflash.h"
#include "config.h"
#include <stdlib.h>
#include <stdio.h>

#define BlueToothUartDebug 0
/************************************************
 ALIENTEK Mini STM32F103 ������ FreeRTOSʵ��4-1
 FreeRTOS���񴴽���ɾ��(��̬����)-�⺯���汾
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 �������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

//�������ȼ�
#define START_TASK_PRIO		1
//�����ջ��С	
#define START_STK_SIZE 		128  
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

//�������ȼ�
#define CAN_TASK_PRIO		30
//�����ջ��С	
#define CAN_STK_SIZE 		256  
//������
TaskHandle_t CANTask_Handler;
//������
void can_task(void *pvParameters);

int main(void)
{
	//SCB->VTOR = FLASH_BASE |0x8000;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4	 
	delay_init();	    				//��ʱ������ʼ��	
	LED_Init();	
	Initial_UART1(115200);
	//500kB
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
	//1M
	//CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);
			  					//��ʼ��LED	
	//updateAppCrc32();
	xTaskCreate((TaskFunction_t )start_task,            //������
							(const char*    )"start_task",          //��������
							(uint16_t       )START_STK_SIZE,        //�����ջ��С
							(void*          )NULL,                  //���ݸ��������Ĳ���
							(UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
							(TaskHandle_t*  )&StartTask_Handler);   //������              
	vTaskStartScheduler();          //�����������
}



//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
    //����TASK1����
	 xTaskCreate((TaskFunction_t )can_task,             
                (const char*    )"can_task",           
                (uint16_t       )CAN_STK_SIZE,        
                (void*          )NULL,                  
                (UBaseType_t    )CAN_TASK_PRIO,        
                (TaskHandle_t*  )&CANTask_Handler); 
		vTaskDelete(StartTask_Handler); //ɾ����ʼ����
		BreakLight=0;
		taskEXIT_CRITICAL();            //�˳��ٽ���														
		
   
}



void can_task(void *pvParameters)
{
	
	
	
	
	u8 canbuf1[8] = {0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 canbuf2[8] = {0x02,0x01,0x00,0x01,0x00,0x00,0x00,0x00};
	u8 canbuf3[8] = {0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 canbuf4[8] = {0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	u8 canbuf5[8] = {0x1F,0x00,0x00,0x00,0xff,0x3f,0x00,0x00};
	u8 canbuf6[8] = {0x1F,0x00,0x00,0x00,0xff,0x1f,0x00,0x00};
	
	
	
//	vTaskDelay(1000);
//	printf("start!");		
//	Can_Send_Msg(canbuf1,8,0x401);
//	vTaskDelay(100);
//	printf("run state!");		
//	Can_Send_Msg(canbuf2,8,0x601);
//	vTaskDelay(100);
//	printf("position mode!");		
//	Can_Send_Msg(canbuf3,8,0x401);
//	vTaskDelay(100);
//	printf("enable state!");		
//	Can_Send_Msg(canbuf4,8,0x401);
//	vTaskDelay(500);
//	printf("goto command!");	
//	printf("runinng.....");		
//	Can_Send_Msg(canbuf5,8,0x401);
//	printf("backing.....");		
//	Can_Send_Msg(canbuf5,8,0x401);
//	vTaskDelay(100);
//		
//	while(1)
//	{
//		vTaskDelay(1000);
//		printf("send can message!");							
//	}

	u8 canbuf7[8] = {0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
	
	u8 rec[8];
	u32 stdid = 0;
	u8 len = 0;

		
	while(1)
	{	
		printf("start!");		
		Can_Send_Msg(canbuf7,8,0x601);
		vTaskDelay(100);
		len = Can_Receive_Msg(rec,&stdid);
		printf("len:%d stdid:%d data %d %d %d %d %d %d %d %d\r\n",len,stdid,rec[0],rec[1],rec[2],rec[3],rec[4],rec[5],rec[6],rec[7]);
		
		vTaskDelay(100);
		
		
		
	}
}
