#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "ax_nrf24l01.h"
#include "config.h"
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
    //SCB->VTOR = FLASH_BASE |0x8000;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
    delay_init();	    				//延时函数初始化
    //LED_Init();
    Initial_UART1(115200);
    //500kB
    CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
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
void nrf_task(void *pvParameters)
{

    vTaskDelay(1000);
    printf("nrf start!\r\n");
	  printf("nrf status : %d\r\n",nrf_status);
    while(1) {
        if(ax_flag_nrf24l01_rx_ok) {

            printf("ch1 %d ch2 %d ch3 %d ch4 %d ch5 %d ",
                   ax_nrf24l01_rxbuf[0]<<8|ax_nrf24l01_rxbuf[1],
                   ax_nrf24l01_rxbuf[2]<<8|ax_nrf24l01_rxbuf[3],
                   ax_nrf24l01_rxbuf[4]<<8|ax_nrf24l01_rxbuf[5],
                   ax_nrf24l01_rxbuf[6]<<8|ax_nrf24l01_rxbuf[7],
                   ax_nrf24l01_rxbuf[9]);
            ax_flag_nrf24l01_rx_ok = 0;
        }
        vTaskDelay(100);


    }

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


    while(1) {
        //printf("start!");
        Can_Send_Msg(canbuf7,8,0x601);
        vTaskDelay(100);
        len = Can_Receive_Msg(rec,&stdid);
        //printf("len:%d stdid:%d data %d %d %d %d %d %d %d %d\r\n",len,stdid,rec[0],rec[1],rec[2],rec[3],rec[4],rec[5],rec[6],rec[7]);

        vTaskDelay(100);



    }
}

