#include "dac.h"

void Voice_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    DAC_InitTypeDef DAC_InitType;





    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE );	  //ʹ��PORTAͨ��ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE );	  //ʹ��DACͨ��ʱ��

    /*On OFF*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //LED0-->PB.5 �˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
    GPIO_ResetBits(GPIOA,GPIO_Pin_1);						 //PB.5 �����

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_2;				 //LED0-->PB.5 �˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOC, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5

    GPIO_ResetBits(GPIOC,GPIO_Pin_3|GPIO_Pin_2);
    /*DAC*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				 // �˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 		 //ģ������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA,GPIO_Pin_4)	;//PA.4 �����

    DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//��ʹ�ô������� TEN1=0
    DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//��ʹ�ò��η���
    DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//���Ρ���ֵ����
    DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1�������ر� BOFF1=1
    DAC_Init(DAC_Channel_1,&DAC_InitType);	 //��ʼ��DACͨ��1

    DAC_Cmd(DAC_Channel_1, ENABLE);  //ʹ��DAC1

    DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12λ�Ҷ������ݸ�ʽ����DACֵ

}

//0-100
void Set_Vol(u16 vol)
{
    float temp=0;
    if(vol<0) vol=-vol;
    temp=vol;
    temp/=100;
    temp=temp*4096;
    DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12λ�Ҷ������ݸ�ʽ����DACֵ
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


















































