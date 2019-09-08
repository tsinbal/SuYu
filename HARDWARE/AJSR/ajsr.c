#include "ajsr.h"
#include "delay.h"
#include "usart.h"
#define TRIG_PIN    GPIO_Pin_4
#define ECHO_PIN    GPIO_Pin_6
u16 uart_distance=0;
u8 uart3_start=0;
static volatile uint32_t measurement; 
void Initial_UART3(u32 bound){ //wifi
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USART1��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
	//USART3_TX   GPIOB10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART3_RX	  GPIOB11
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB11

  //Usart3 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=6;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART3 ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART3, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���1 

}


void USART3_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
	static u8 sum=0;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART3);	//��ȡ���յ�������
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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=8;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
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
