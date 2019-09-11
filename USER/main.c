#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "ax_nrf24l01.h"
#include "nimotion.h"

#include <stdlib.h>
#include <stdio.h>

#define BlueToothUartDebug 0
/************************************************
 ALIENTEK Mini STM32F103 ������ FreeRTOSʵ��4-1
 FreeRTOS���񴴽���ɾ��(��̬����)-�⺯���汾
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾
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

//�������ȼ�
#define NRF_TASK_PRIO		29
//�����ջ��С
#define NRF_STK_SIZE 		256
//������
TaskHandle_t NRFTask_Handler;
//������
void nrf_task(void *pvParameters);
u8 nrf_status = 0;
int main(void)
{
		u8 time = 0;
	u8 res = 0;
	    uint8_t buf[5]= {0XA5,0XA5,0XA5,0XA5,0XA5};

    //SCB->VTOR = FLASH_BASE |0x8000;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
    delay_init();	    				//��ʱ������ʼ��
	delay_ms(2000);
    //LED_Init();
    Initial_UART1(115200);
    //500kB	
    CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
    nrf_status = AX_NRF24L01_Init();
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
    xTaskCreate((TaskFunction_t )nrf_task,
                (const char*    )"nrf_task",
                (uint16_t       )NRF_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )NRF_TASK_PRIO,
                (TaskHandle_t*  )&NRFTask_Handler);
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����

    taskEXIT_CRITICAL();            //�˳��ٽ���


}
uint16_t ch1 = 128;
uint16_t ch2 = 128;

void nrf_task(void *pvParameters)
{

    vTaskDelay(1000);
    printf("nrf start!\r\n");
	  printf("nrf status : %d\r\n",nrf_status);
    while(1) {
        if(ax_flag_nrf24l01_rx_ok) {
					ch1 = ax_nrf24l01_rxbuf[0]<<8|ax_nrf24l01_rxbuf[1];
					ch2 = ax_nrf24l01_rxbuf[2]<<8|ax_nrf24l01_rxbuf[3];
            printf("ch1 %d ch2 %d ch3 %d ch4 %d ch5 %d ",ch1
                   ,ch2,ax_nrf24l01_rxbuf[4]<<8|ax_nrf24l01_rxbuf[5],
                   ax_nrf24l01_rxbuf[6]<<8|ax_nrf24l01_rxbuf[7],
                   ax_nrf24l01_rxbuf[9]);
            ax_flag_nrf24l01_rx_ok = 0;
        }
        vTaskDelay(100);


    }

}


void can_task(void *pvParameters)
{
	Nimotion_Init(1280);
	vTaskDelay(1000);
  while(1) {
		Nimotion_Position_SendValue(Nimotion_StateMachine_GotoCommand,0,0);
    vTaskDelay(50);
		Nimotion_Position_SendValue(Nimotion_StateMachine_GotoPosition,0,ch1 * 10);
		vTaskDelay(50);
		printf("goto position %d\r\n",ch1);
  }
}

