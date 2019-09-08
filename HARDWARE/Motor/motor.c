#include "motor.h"
#include "dac.h"
#include "led.h"
#include "FreeRTOS.h"		
#include "config.h"
/*void PWM_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);//??GPIO??
    //???????PA.8-->TIM1_CH1,PA.9-->TIM1_CH2,PA.10-->TIM1_CH2
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;    //PA8 PA11
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;        //??????
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA,&GPIO_InitStructure);                //??GPIOA
	
    //???????PB.13-->TIM1_CH1N,PB.14-->TIM1_CH2N,PB.15-->TIM1_CH3N
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;    //PA8 PA11
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;        //??????
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;   
    GPIO_Init(GPIOB,&GPIO_InitStructure);                //??GPIOB
	
	   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;    //PA8 PA11
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;        //??????
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;   
    GPIO_Init(GPIOC,&GPIO_InitStructure);
}
void PWM_Init(uint16_t arr,uint16_t psc)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	
    TIM_DeInit(TIM1);                                                                                        
   
    TIM_TimeBaseStructure.TIM_Period=arr;                                               
    TIM_TimeBaseStructure.TIM_Prescaler=psc;                                          
    TIM_TimeBaseStructure.TIM_ClockDivision=0X00;                                   
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;            
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(TIM1,DISABLE); 

    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(TIM8,DISABLE);	
   
    TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;                               
    TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;         
    TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;   
    TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;                
    TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCPolarity_High;              
    TIM_OCInitStructure.TIM_Pulse=1;                                                   
    TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;             
    TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCIdleState_Reset;       

    TIM_OC1Init(TIM1,&TIM_OCInitStructure);
   
    TIM_OC2Init(TIM1,&TIM_OCInitStructure);

    TIM_OC3Init(TIM1,&TIM_OCInitStructure);
		
		TIM_OC1Init(TIM8,&TIM_OCInitStructure);

    TIM_BDTRInitStructure.TIM_OSSIState=TIM_OSSIState_Enable;              
    TIM_BDTRInitStructure.TIM_OSSRState=TIM_OSSRState_Enable;                
    TIM_BDTRInitStructure.TIM_LOCKLevel=TIM_LOCKLevel_OFF;                    
    TIM_BDTRInitStructure.TIM_DeadTime=0xCD;
    TIM_BDTRInitStructure.TIM_Break=TIM_Break_Disable;           
    TIM_BDTRInitStructure.TIM_BreakPolarity=TIM_BreakPolarity_Low;  
    TIM_BDTRInitStructure.TIM_AutomaticOutput=TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(TIM1,&TIM_BDTRInitStructure);
    TIM_CtrlPWMOutputs(TIM1,ENABLE);        
    TIM_ARRPreloadConfig(TIM1,ENABLE);    
    TIM_Cmd(TIM1,ENABLE);  

		TIM_BDTRConfig(TIM8,&TIM_BDTRInitStructure);
    TIM_CtrlPWMOutputs(TIM8,ENABLE);        
    TIM_ARRPreloadConfig(TIM8,ENABLE);    
    TIM_Cmd(TIM8,ENABLE);  
}*/
void PWM_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB,ENABLE);//??GPIO??
   
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;  
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;        
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA,&GPIO_InitStructure);                
	
		   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;  
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;        
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;    
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;      
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;   
    GPIO_Init(GPIOB,&GPIO_InitStructure);                
	
	   GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;    
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;   
    GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA,GPIO_Pin_7);
	GPIO_SetBits(GPIOB,GPIO_Pin_13);
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	GPIO_SetBits(GPIOB,GPIO_Pin_15);
}
void PWM_Init(u16 arr,u16 psc)
{  
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_TIM8, ENABLE);	//使能定时器3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
 
 

   //初始化TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM3 Channel2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Set;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC1
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC2
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 OC3
	TIM_OC1Init(TIM8,&TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM1, ENABLE);   
	TIM_ARRPreloadConfig(TIM8, ENABLE);   
	TIM_CtrlPWMOutputs(TIM1, ENABLE); 
	TIM_CtrlPWMOutputs(TIM8, ENABLE); 
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
	TIM_Cmd(TIM8, ENABLE);  //使能TIM8

}
void Motor_Init()
{
	PWM_GPIO_Config();
	PWM_Init(99,35);
}
int lastspeed=0;
extern int direction;
extern float back_max_speed;
extern float max_speed;

#define Motor_Limit 80
void Motor_SetSpeed(int speed)
{
	u8 voice=0;
	u8 maxspeed=0;
	//voice=SW1+SW2;
	maxspeed=car_info.maxspeed;
	if(maxspeed==0)maxspeed=15;
	

	if(direction>0)
	{
			car_info.velocity=(maxspeed*speed*1.0/100+0.5);
			if(car_info.velocity<0)car_info.velocity=-car_info.velocity;
			if(speed>Motor_Limit)speed=Motor_Limit;
			if(speed<-Motor_Limit)speed=-Motor_Limit;	
			Motor_SetSpeedR(direction*speed*max_speed);
			Motor_SetSpeedL(direction*speed*max_speed);
	}else
	{
			car_info.velocity=(maxspeed*back_max_speed*1.0/100+0.5);
			if(car_info.velocity<0)car_info.velocity=-car_info.velocity;
			if(speed>Motor_Limit)speed=Motor_Limit;
			if(speed<-Motor_Limit)speed=-Motor_Limit;	
		
			Motor_SetSpeedR(direction*speed*back_max_speed);
			Motor_SetSpeedL(direction*speed*back_max_speed);
	}

	Set_Vol(speed);
	Set_Voice(voice);
}

void Motor_SetSpeedR(int speed)
{
		if(speed>0)
		{
				TIM_SetCompare3(TIM1,0);
				TIM_SetCompare2(TIM1,speed);
					
		}
		else if(speed==0)
		{
				
				TIM_SetCompare3(TIM1,0);
				TIM_SetCompare2(TIM1,0);
		}
		else{
			
				TIM_SetCompare3(TIM1,-speed);
				TIM_SetCompare2(TIM1,0);				
			
		}			
			
}	
void Motor_SetSpeedL(int speed)
{
		if(speed>0)
		{
					TIM_SetCompare1(TIM1,0);	
				TIM_SetCompare1(TIM8,speed);
				
		}
		else if(speed==0)
		{
				TIM_SetCompare1(TIM1,0);	
				TIM_SetCompare1(TIM8,0);
		}
		else{		
			TIM_SetCompare1(TIM1,-speed);	
				TIM_SetCompare1(TIM8,0);	
			
			
				
		}			
			
}
void Motor_Go(int need_speed)
{
	if((need_speed-lastspeed)>0)
	{
		lastspeed=lastspeed++;
	
	}
	else if((need_speed-lastspeed)<0)
	{
		lastspeed=lastspeed--;
	}

	if(direction==0)
	{
		lastspeed=0;
	}
	Motor_SetSpeed(lastspeed);	
	
	
}
void Motor_Brake(float ratio)
{

	u16 temp=lastspeed*ratio/3;
	if(lastspeed==0)
	{
		Motor_SetSpeed(lastspeed);
		return;
	}
	Motor_SetSpeed(-temp);
	lastspeed--;
	if(lastspeed<0)lastspeed=0;
}
void Motor_Stop(u8 deep)
{
	lastspeed=lastspeed-deep;
	if(lastspeed<0) lastspeed=0;
	Motor_SetSpeed(lastspeed);
}