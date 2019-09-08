#include "dac.h"

void Voice_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    DAC_InitTypeDef DAC_InitType;





    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE );	  //使能PORTA通道时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );	  //使能DAC通道时钟

    /*On OFF*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //LED0-->PB.5 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
    GPIO_ResetBits(GPIOA,GPIO_Pin_1);						 //PB.5 输出高

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_2;				 //LED0-->PB.5 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5

    GPIO_ResetBits(GPIOC,GPIO_Pin_3|GPIO_Pin_2);
    /*DAC*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				 // 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 		 //模拟输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA,GPIO_Pin_4)	;//PA.4 输出高

    DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//不使用触发功能 TEN1=0
    DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//不使用波形发生
    DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//屏蔽、幅值设置
    DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1输出缓存关闭 BOFF1=1
    DAC_Init(DAC_Channel_1,&DAC_InitType);	 //初始化DAC通道1

    DAC_Cmd(DAC_Channel_1, ENABLE);  //使能DAC1

    DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12位右对齐数据格式设置DAC值

}

//0-100
void Set_Vol(u16 vol)
{
    float temp=0;
    if(vol<0) vol=-vol;
    temp=vol;
    temp/=100;
    temp=temp*4096;
    DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12位右对齐数据格式设置DAC值
}


void Set_Voice(u8 voice)
{
    if(voice==0) {
        PCout(2)=0;
        PCout(3)=0;
    } else if(voice==1) {
        PCout(2)=1;
        PCout(3)=0;
    } else if(voice==2) {
        PCout(2)=0;
        PCout(3)=1;
    } else if(voice==3) {
        PCout(2)=1;
        PCout(3)=1;
    }

}


















































