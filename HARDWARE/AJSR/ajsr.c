#include "ajsr.h"
#include "delay.h"
#include "usart.h"
#define TRIG_PIN    GPIO_Pin_4
#define ECHO_PIN    GPIO_Pin_6
u16 uart_distance=0;
u8 uart3_start=0;
static volatile uint32_t measurement; 
void Initial_UART3(u32 bound){ //wifi
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART1，GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
	//USART3_TX   GPIOB10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART3_RX	  GPIOB11
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB11

  //Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=6;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART3 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART3, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口1 

}


void USART3_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
	static u8 sum=0;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART3);	//读取接收到的数据
		if(Res==0xff&&uart3_start==0)
			uart3_start=1;
		else if(uart3_start==1)
		{
			uart_distance=(Res<<8);
		
			sum=Res;
			uart3_start=2;
		}
		else if(uart3_start==2)
		{
			uart_distance|=Res;
			
			sum+=Res;
			uart3_start=3;
		}
		else if(uart3_start==3)
		{
		
			if(Res==sum)
			{
				measurement=uart_distance;
			}
			uart3_start=0;
			uart_distance=0;
		}
  }
		USART_ClearITPendingBit(USART3,USART3_IRQn);		
} 
void SRInit(void) 
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = TRIG_PIN; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = ECHO_PIN; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, ECHO_PIN); 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6); 

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); /* TIME5 base configuration */ 
	TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF; //  
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72-1; // ?????,F=72MHz/72=1MHz,T=1us 
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure); 
	TIM_Cmd(TIM5, DISABLE); 
}
void EXTI6_CMD(FunctionalState NewState) 
{
	EXTI_InitTypeDef EXTI_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=8;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	EXTI_ClearITPendingBit(EXTI_Line6); 
	EXTI_InitStructure.EXTI_Line = EXTI_Line6; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
	EXTI_InitStructure.EXTI_LineCmd = NewState; 
	EXTI_Init(&EXTI_InitStructure); /* Enable TIM5 clock */ 
}
void SRStartRanging(void) 
{
	TIM_Cmd(TIM5, DISABLE);	
	EXTI6_CMD(DISABLE);
	TIM_SetCounter(TIM5, 0); 	
	
	GPIO_SetBits(GPIOC, TRIG_PIN); 
	delay_us(1100); //  The width of trig signal must be greater than 10us 
	GPIO_ResetBits(GPIOC, TRIG_PIN); 
	EXTI6_CMD(ENABLE);
} 
float SRGetDistance(void) 
{ 
	float distance = measurement / 10.0; // measurement-units:us 
	return distance; 
} 
static void ECHO_EXTI_IRQHandler(void) 
{ 
	if (EXTI_GetITStatus(EXTI_Line6) != RESET)
	{ 
		if (GPIO_ReadInputDataBit(GPIOA, ECHO_PIN) != 0) 
		{ 
				TIM_Cmd(TIM5, ENABLE); 
		} 
		else 
		{ 
			TIM_Cmd(TIM5, DISABLE); 
			measurement = TIM_GetCounter(TIM5); 
			TIM_SetCounter(TIM5, 0); 
			EXTI6_CMD(DISABLE);
		} 
	} 
		EXTI_ClearITPendingBit(EXTI_Line6); 
} 
void EXTI9_5_IRQHandler(void) 
{ 
ECHO_EXTI_IRQHandler(); 
}
