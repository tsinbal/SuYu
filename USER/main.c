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
 ALIENTEK Mini STM32F103 开发板 FreeRTOS实验4-1
 FreeRTOS任务创建和删除(动态方法)-库函数版本
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司
 作者：正点原子 @ALIENTEK
************************************************/

//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小
#define START_STK_SIZE 		128
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define CAN_TASK_PRIO		30
//任务堆栈大小
#define CAN_STK_SIZE 		256
//任务句柄
TaskHandle_t CANTask_Handler;
//任务函数
void can_task(void *pvParameters);

//任务优先级
#define NRF_TASK_PRIO		29
//任务堆栈大小
#define NRF_STK_SIZE 		256
//任务句柄
TaskHandle_t NRFTask_Handler;
//任务函数
void nrf_task(void *pvParameters);
u8 nrf_status = 0;
int main(void)
{
		u8 time = 0;
	u8 res = 0;
	    uint8_t buf[5]= {0XA5,0XA5,0XA5,0XA5,0XA5};

    //SCB->VTOR = FLASH_BASE |0x8000;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
    delay_init();	    				//延时函数初始化
	delay_ms(2000);
    //LED_Init();
    Initial_UART1(115200);
    //500kB	
    CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_Normal);
    nrf_status = AX_NRF24L01_Init();
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄
    vTaskStartScheduler();          //开启任务调度
}



//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    //创建TASK1任务
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
    vTaskDelete(StartTask_Handler); //删除开始任务

    taskEXIT_CRITICAL();            //退出临界区


}
uint16_t ch1 = 128;
uint16_t ch2 = 128;
uint16_t ch3 = 128;
uint16_t ch4 = 128;
u8 left_speed = 0;
u8 right_speed = 0;
u8 dir =1;
void nrf_task(void *pvParameters)
{

    vTaskDelay(1000);
    printf("nrf start!\r\n");
	  printf("nrf status : %d\r\n",nrf_status);
	
    while(1) {
        if(ax_flag_nrf24l01_rx_ok) {
					ch1 = ax_nrf24l01_rxbuf[0]<<8|ax_nrf24l01_rxbuf[1];
					ch2 = ax_nrf24l01_rxbuf[2]<<8|ax_nrf24l01_rxbuf[3];
					ch3 = ax_nrf24l01_rxbuf[4]<<8|ax_nrf24l01_rxbuf[5];
					ch4 = ax_nrf24l01_rxbuf[6]<<8|ax_nrf24l01_rxbuf[7];
					
					
					
//          printf("ch1 %d ch2 %d ch3 %d ch4 %d ch5 %d ",ch1
//                   ,ch2,ax_nrf24l01_rxbuf[4]<<8|ax_nrf24l01_rxbuf[5],
//                   ax_nrf24l01_rxbuf[6]<<8|ax_nrf24l01_rxbuf[7],
//                   ax_nrf24l01_rxbuf[9]);
            ax_flag_nrf24l01_rx_ok = 0;
        }
        vTaskDelay(30);


    }

}
u16 abs_a_b(u16 a, u16 b)
{
	if(a > b)
		return a - b;
	else if(a < b)
		return b - a;
	else 
		return 0;
}
void can_task(void *pvParameters)
{
	Nimotion_Init(1280);
	vTaskDelay(1000);
  while(1) {
		left_speed = abs_a_b(ch3,128)*100/127;
		right_speed = left_speed;
		dir = (ch3 > 128) ? 1 : 0;
		
		Nimotion_Position_SendValue(Nimotion_StateMachine_Position_GotoCommand,0,0);
		vTaskDelay(20);
		Nimotion_Position_SendValue(Nimotion_StateMachine_Position_GotoPosition,0,ch1 * 10);
		vTaskDelay(20);
		Nimotion_Driver_SendValue(dir,dir,left_speed,right_speed);
		vTaskDelay(20);
//		if(ch1 > 128) dir = 1;
//		else dir = 0;
//		
//		Nimotion_Velocity_SendValue(Nimotion_StateMachine_Velocity_Run,dir,abs_a_b(ch1 , (u16)128));
//		vTaskDelay(30);
		//printf("goto position %d\r\n",abs_a_b(ch1 , 128)*10);
  }
}

