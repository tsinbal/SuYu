/**			____                   _____ _________   __
		   / __ \                 |  __ \__   __\ \ / /
		  | |  | |_ __   ___ _ __ | |__) | | |   \ V /
		  | |  | | '_ \ / _ \ '_ \|  _  /  | |    > <
		  | |__| | |_) |  __/ | | | | \ \  | |   / . \
		   \____/| .__/ \___|_| |_|_|  \_\ |_|  /_/ \_\
				 | |
				 |_|  X-REMOTEң���� - OpenRTX��Դ���ϵͳ

  ******************************************************************************
  *
  * ��Ȩ���У� XTARK@���˴���  ��Ȩ���У�����ؾ�
  * ������վ�� www.xtark.cn
  * �Ա����̣� https://shop246676508.taobao.com
  * ����ý�壺 www.cnblogs.com/xtark�����ͣ�
  *
  ******************************************************************************
  * @��  ��  ax_nrf24l01.c
  * @��  ��  Musk Han@XTARK
  * @��  ��  V1.0
  * @��  ��  2018-10-26
  * @��  ��  X-REMOTE NRF24L01����ͨ��
  *
  ******************************************************************************
  * @˵  ��
  *
  * 1.NRF24L01�����շ�����Ĭ��Ϊ����ģʽ�����ͺ����Զ��л�Ϊ����ģʽ
  *   ������ɺ��Զ��л�������ģʽ���ȴ���������
  * 2.IRQ���� �����жϴ���ʽ��ͨ����ȡNRF24L01�жϱ�־�Ĵ�����ȡ�ж�����
  * 3.LED�ƽ��з���״ָ̬ʾ�����ͳɹ���LEDȡ����˸�������ط�������LEDϨ��
  * 4.����ͽ��յ�ַĬ��ǰ��λΪ0xAA�����һλ��ͨ���ӿ��޸�
  * 5.ÿ�����ݷ����ֽڹ̶�Ϊ32�ֽڣ�NRF24L01����ֽ�
  *
  ******************************************************************************
  */

#include "ax_nrf24l01.h"
#include "task.h"

#include <stdio.h>

//ȫ�ֱ���
uint8_t ax_flag_nrf24l01_rx_ok = 0;
uint8_t ax_nrf24l01_rxbuf[NRF24L01_RX_PLOAD_WIDTH];  //���ݽ��ջ����������ݷ��ͻ�����
/*����ͨ��0����������ȷ����Ϣ����˷��Ͷ˵�����ͨ��0�ĵ�ַ������ڷ��͵�ַ�����������յ�Ӧ����Ϣ*/
uint8_t AX_NRF24L01_TX_ADDRESS[NRF24L01_TX_ADR_WIDTH]= {0xAA,0xAA,0xAA,0xAA,0x01};   //Զ�˵�ַ
uint8_t AX_NRF24L01_RX_ADDRESS[NRF24L01_RX_ADR_WIDTH] = {0xAA,0xAA,0xAA,0xAA,0x01};   //������ַ

//IO�����궨��
#define NRF24L01_CE_H()   GPIO_SetBits(GPIOB, GPIO_Pin_0)
#define NRF24L01_CE_L()   GPIO_ResetBits(GPIOB, GPIO_Pin_0)
#define NRF24L01_CSN_H()  GPIO_SetBits(GPIOB, GPIO_Pin_14)
#define NRF24L01_CSN_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_14)
#define NRF24L01_IRQ     (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1))

//NRF24L01 ����״̬
#define NRF24L01_TX_MAX  	 0x10    //�ﵽ����ʹ�����־
#define NRF24L01_TX_DS   	 0x20    //����������ɱ�־
#define NRF24L01_RX_DR   	 0x40    //����������ɱ�־

//NRF24L01 �Ĵ���ָ��
#define NREAD_REG       0x00  	// ���Ĵ���ָ��
#define NWRITE_REG      0x20 	// д�Ĵ���ָ��
#define RD_RX_PLOAD     0x61  	// ��ȡ��������ָ��
#define WR_TX_PLOAD     0xA0  	// д��������ָ��
#define FLUSH_TX        0xE1 	// ��ϴ���� FIFOָ��
#define FLUSH_RX        0xE2  	// ��ϴ���� FIFOָ��
#define REUSE_TX_PL     0xE3  	// �����ظ�װ������ָ��
#define NOP             0xFF  	// ����

//NRF24L01 �Ĵ�����ַ
#define CONFIG          0x00    // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           0x01    // �Զ�Ӧ��������
#define EN_RXADDR       0x02    // �����ŵ�����
#define SETUP_AW        0x03    // �շ���ַ�������
#define SETUP_RETR      0x04    // �Զ��ط���������
#define RF_CH           0x05    // ����Ƶ������
#define RF_SETUP        0x06    // �������ʡ����Ĺ�������
#define STATUS          0x07    // ״̬�Ĵ���
#define OBSERVE_TX      0x08    // ���ͼ�⹦��
#define CD              0x09    // ��ַ���           
#define RX_ADDR_P0      0x0A    // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1      0x0B    // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2      0x0C    // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3      0x0D    // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4      0x0E    // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5      0x0F    // Ƶ��5�������ݵ�ַ
#define TX_ADDR         0x10    // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11    // ����Ƶ��0�������ݳ���
#define RX_PW_P1        0x12    // ����Ƶ��1�������ݳ���
#define RX_PW_P2        0x13    // ����Ƶ��2�������ݳ���
#define RX_PW_P3        0x14    // ����Ƶ��3�������ݳ���
#define RX_PW_P4        0x15    // ����Ƶ��4�������ݳ���
#define RX_PW_P5        0x16    // ����Ƶ��5�������ݳ���
#define FIFO_STATUS     0x17    // FIFOջ��ջ��״̬�Ĵ�������

//�ļ��ڲ���������
static uint8_t SPI1_Send_Byte(uint8_t dat);
static uint8_t NRF24L01_ReadReg(uint8_t reg);
static uint8_t NRF24L01_WriteReg(uint8_t reg, uint8_t dat);
static uint8_t NRF24L01_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t len);
static uint8_t NRF24L01_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len);
static void NRF24L01_SetRxMode(void);  //NRF24L01���ý���ģʽ
static void NRF24L01_ReadPacket(uint8_t *pbuf);  //��ȡ���յ����ݰ�

/**
  * @��  ��  NRF24L01��ʼ����
  * @��  ��  ��
  * @����ֵ  ��ʼ�����  1 ��ʼ���ɹ� 0 ��ʼ��ʧ��
  */
uint8_t AX_NRF24L01_Init(void)
{
    uint8_t buf[5]= {0XA5,0XA5,0XA5,0XA5,0XA5};

    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_initStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    //��GPIOʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);

    //SCK
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB13/14/15�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOB

    //CE
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //CSN
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //IRQ��NRF�жϲ���ʱ��IRQ���Żᱻ����
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //SPI����
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;  //SPI_CPOL_High=ģʽ3��ʱ�ӿ���Ϊ�� //SPI_CPOL_Low=ģʽ0��ʱ�ӿ���Ϊ��
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//SPI_CPHA_2Edge;//SPI_CPHA_1Edge, SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;   //SPI_NSS_Soft;//SPI_NSS_Hard
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//10.5M;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//���ݴӸ�λ��ʼ����
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    //Enable SPI
    SPI_Cmd(SPI1, ENABLE);

    //����IRQ�ж�Դ
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;//������ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//���������ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);

    //����IRQ�ж�
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource1);//����GPIO�ܽŵ��ж���·

    EXTI_initStructure.EXTI_Line = EXTI_Line1;
    EXTI_initStructure.EXTI_LineCmd = ENABLE;
    EXTI_initStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_initStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//�½��ش���
    EXTI_Init(&EXTI_initStructure);

    /******NRF24L01����******/
    NRF24L01_CE_L();    //ʹ��оƬ
    NRF24L01_CSN_H();   //��ʹ��SPIƬѡ

    //���NRF24L01�Ƿ���λ���ж϶�����ַ�Ƿ���д���ַ��ͬ
    NRF24L01_WriteBuf(NWRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.
    NRF24L01_ReadBuf(TX_ADDR,buf,5); //����д��ĵ�ַ

    if(buf[0] == 0XA5) { //��⵽24L01,����NRF24L01������ģʽ
        NRF24L01_WriteBuf(NWRITE_REG + TX_ADDR, AX_NRF24L01_TX_ADDRESS, NRF24L01_TX_ADR_WIDTH);       //����Զ�˵�ַ
        NRF24L01_WriteBuf(NWRITE_REG + RX_ADDR_P0, AX_NRF24L01_RX_ADDRESS, NRF24L01_RX_ADR_WIDTH);    //���ñ�����ַ

        NRF24L01_WriteReg(NWRITE_REG + SETUP_RETR, 0x1a);   //�����Զ��ط�ʱ��ʹ���500us + 86us,�ط�����10��
        NRF24L01_WriteReg(NWRITE_REG + EN_AA, 0x01);       //�������ݺ�����Ƶ��0�Զ�Ӧ��
        NRF24L01_WriteReg(NWRITE_REG + EN_RXADDR, 0x01);   //ʹ��ͨ��0�Ľ��յ�ַ
        NRF24L01_WriteReg(NWRITE_REG + RF_SETUP, 0x0f);    //���÷�������Ϊ2MHZ�����书��Ϊ���ֵ0dB ���������濪��
        NRF24L01_WriteReg(NWRITE_REG + RF_CH, 40);         //  �����ŵ�����Ϊ2.4GHZ���շ�����һ��
        NRF24L01_WriteReg(NWRITE_REG + RX_PW_P0, NRF24L01_RX_PLOAD_WIDTH); //���ý������ݳ��ȣ���������Ϊ32�ֽ�
        NRF24L01_WriteReg(NWRITE_REG + CONFIG, 0x0f);     //���û�������ģʽ�Ĳ�����PWR_UP,EN_CRC,16BIT_CRC,����ģʽ��	�������� �����ж�

        //��������жϱ�־
        NRF24L01_WriteReg(NWRITE_REG+STATUS, NRF24L01_RX_DR);   //���յ����ݺ�RX_DR,TX_DS,MAX_RT���ø�Ϊ1��ͨ��д1������жϱ�־
        NRF24L01_WriteReg(FLUSH_RX,0xff);      // ��RX FIFO�Ĵ���

        NRF24L01_CE_H();
        return 1;
    } else { //δ��⵽24L01��24L01Ӳ������
        NRF24L01_CE_H();
        return 0;
    }
}

/**
  * @��  ��  NRF24L01 ���÷��͵�ַ(�Է��Ľ��յ�ַ)
  * @��  ��  addr�����һλ��ַ��ǰ���ַΪ0xAA�����磺0xAA,0xAA,0xAA,0xAA,0x01
  * @����ֵ  ��
  */
void AX_NRF24L01_SetTxAddress(uint8_t addr)
{
    //���һλ��Ϊ���õ�ַ
    AX_NRF24L01_TX_ADDRESS[NRF24L01_TX_ADR_WIDTH - 1] = addr;

    NRF24L01_CE_L();		//����CE���������ģʽ��׼��д������

    NRF24L01_WriteBuf(NWRITE_REG + TX_ADDR, AX_NRF24L01_TX_ADDRESS, NRF24L01_TX_ADR_WIDTH);       //����Զ�˵�ַ

    //��������жϱ�־
    NRF24L01_WriteReg(NWRITE_REG+STATUS, NRF24L01_RX_DR);   //���յ����ݺ�RX_DR,TX_DS,MAX_RT���ø�Ϊ1��ͨ��д1������жϱ�־
    NRF24L01_WriteReg(FLUSH_RX,0xff);      // ��RX FIFO�Ĵ���

    NRF24L01_CE_H();		 //�ø�CE���������ݷ���
}

/**
  * @��  ��  NRF24L01���ý��յ�ַ���������յ�ַ��
  * @��  ��  addr�����һλ��ַ��ǰ���ַΪ0xAA�����磺0xAA,0xAA,0xAA,0xAA,0x01
  * @����ֵ  ��
  */
void AX_NRF24L01_SetRxAddress(uint8_t addr)
{
    //���һλ��Ϊ���õ�ַ
    AX_NRF24L01_RX_ADDRESS[NRF24L01_RX_ADR_WIDTH - 1] = addr;

    NRF24L01_CE_L();		//����CE���������ģʽ��׼��д������

    NRF24L01_WriteBuf(NWRITE_REG + RX_ADDR_P0, AX_NRF24L01_RX_ADDRESS, NRF24L01_RX_ADR_WIDTH);    //���ñ�����ַ

    //��������жϱ�־
    NRF24L01_WriteReg(NWRITE_REG+STATUS, NRF24L01_RX_DR);   //���յ����ݺ�RX_DR,TX_DS,MAX_RT���ø�Ϊ1��ͨ��д1������жϱ�־
    NRF24L01_WriteReg(FLUSH_RX,0xff);      // ��RX FIFO�Ĵ���

    NRF24L01_CE_H();		 //�ø�CE���������ݷ���
}

/**
  * @��  ��  NRF24L01����Ƶ�ʵ�
  * @��  ��  chr��Ƶ�ʵ㣬��ΧΪ0~127
  * @����ֵ  ��
  */
void AX_NRF24L01_SetFrqChannel(uint8_t chr)
{
    //��������
    if(chr>127)
        chr = 127;

    //����CE���������ģʽ��׼��д������
    NRF24L01_CE_L();

    //�����ŵ�����Ϊ2.4GHZ���շ�����һ��
    NRF24L01_WriteReg(NWRITE_REG + RF_CH, chr);

    //��������жϱ�־
    NRF24L01_WriteReg(NWRITE_REG+STATUS, NRF24L01_RX_DR);   //���յ����ݺ�RX_DR,TX_DS,MAX_RT���ø�Ϊ1��ͨ��д1������жϱ�־
    NRF24L01_WriteReg(FLUSH_RX,0xff);      // ��RX FIFO�Ĵ���

    //�ø�CE���������ݷ���
    NRF24L01_CE_H();

}

/**
  * @��  ��  NRF24L01���ÿ�������
  * @��  ��  rate���������ʣ���0-250K 1-1M 2-2M��
  * @����ֵ  ��
  */
void AX_NRF24L01_SetSpeed(uint8_t rate)
{
    NRF24L01_CE_L();		//����CE���������ģʽ��׼��д������

    if(rate == 0)
        NRF24L01_WriteReg(NWRITE_REG + RF_SETUP, 0x27);    //���÷�������Ϊ250KHZ�����书��Ϊ���ֵ0dB ���������濪��
    else if(rate == 1)
        NRF24L01_WriteReg(NWRITE_REG + RF_SETUP, 0x07);    //���÷�������Ϊ1MHZ�����书��Ϊ���ֵ0dB ���������濪��
    else if(rate == 2)
        NRF24L01_WriteReg(NWRITE_REG + RF_SETUP, 0x0f);    //���÷�������Ϊ2MHZ�����书��Ϊ���ֵ0dB ���������濪��

    //��������жϱ�־
    NRF24L01_WriteReg(NWRITE_REG+STATUS, NRF24L01_RX_DR);   //���յ����ݺ�RX_DR,TX_DS,MAX_RT���ø�Ϊ1��ͨ��д1������жϱ�־
    NRF24L01_WriteReg(FLUSH_RX,0xff);      // ��RX FIFO�Ĵ���

    NRF24L01_CE_H();		 //�ø�CE���������ݷ���
}

/**
  * @��  ��  NRF24L01�������ݰ�
  * @��  ��  *pbuf������������ָ��
  * @����ֵ  ��
  */
void AX_NRF24L01_SendPacket(uint8_t *pbuf)
{
    //����CE���������ģʽ��׼��д������
    NRF24L01_CE_L();

    //������д��TX�˵�FIFO��,д��ĸ�����TX_PLOAD_WIDTH����ֵ��ͬ
    NRF24L01_WriteBuf(WR_TX_PLOAD, pbuf, NRF24L01_TX_PLOAD_WIDTH);

    NRF24L01_WriteReg(NWRITE_REG + CONFIG, 0x0e);     //���û�������ģʽ�Ĳ�����PWR_UP,EN_CRC,16BIT_CRC,����ģʽ��	���������ж�
    NRF24L01_WriteReg(NWRITE_REG + STATUS, 0x7e);	  //д0111 xxxx ��STATUS����������жϱ�־����ֹһ�������ģʽ�ʹ����ж�

    //�ø�CE���������ݷ���
    NRF24L01_CE_H();

    //��ʱ1ms
    vTaskDelay(1);
}

/**
  * @��  ��  NRF24L01 IRQ�жϴ���
  * @��  ��  ��
  * @����ֵ  ��
  */
void EXTI1_IRQHandler(void)
{
    uint8_t status;

    //�Ƿ������EXTI Line�ж�
    if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
        //�жϴ���
        NRF24L01_CE_L();		//����CE����ȡ����
        status = NRF24L01_ReadReg(STATUS);	// ��ȡ״̬�Ĵ������ж����ݽ���״��

        //���ݷ��ͳɹ��жϣ��յ�Ӧ���źţ�
        if( status & NRF24L01_TX_DS) {
            //���ý���ģʽ���ȴ���������
            NRF24L01_SetRxMode();
            NRF24L01_WriteReg(NWRITE_REG + STATUS, NRF24L01_TX_DS);  //���TX_DS�жϱ�־
            NRF24L01_WriteReg(FLUSH_TX, 0xff);   //��� TX_FIFO �Ĵ���

            //�û�����

        }

        //���ճɹ��ж�
        else if( status & NRF24L01_RX_DR) {
            //��ȡ����
            NRF24L01_ReadPacket(ax_nrf24l01_rxbuf);
            NRF24L01_WriteReg(NWRITE_REG+STATUS, NRF24L01_RX_DR);   //���յ����ݺ�RX_DR,TX_DS,MAX_RT���ø�Ϊ1��ͨ��д1������жϱ�־

            //�û�����
            ax_flag_nrf24l01_rx_ok = 1;
        }

        //�����ط������ж�
        else if(status & NRF24L01_TX_MAX) {
            //�������ģʽ������жϡ�FIFO
            NRF24L01_SetRxMode();
            NRF24L01_WriteReg(NWRITE_REG + STATUS, NRF24L01_TX_MAX);  //��� TX_DS �� MAX_RT�жϱ�־
            NRF24L01_WriteReg(FLUSH_TX, 0xff);   //��� TX_FIFO �Ĵ���

            //�û��������

        }

        //����жϱ�־λ
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}


/* NRF24L01 �ڲ��ײ��������-----------------------------------------------------*/
/**
  * @��  ��  ����Ϊ����ģʽ
  * @��  ��  ��
  * @����ֵ  ��
  */
static void NRF24L01_SetRxMode(void)
{
    NRF24L01_CE_L();	//����CE���������ģʽ��׼����ʼ��SI24R1�еļĴ�����д������

    NRF24L01_WriteReg(NWRITE_REG + CONFIG, 0x0f);    //���û�������ģʽ�Ĳ�����PWR_UP,EN_CRC,16BIT_CRC,����ģʽ��	�������� �����ж�
    NRF24L01_WriteReg(NWRITE_REG + STATUS, 0x7e);	//д0111 xxxx ��STATUS����������жϱ�־����ֹһ�������ģʽ�ʹ����ж�

    NRF24L01_CE_H(); 	//����CE��׼�����ܴ��ⲿ���͹���������
}
/**
  * @��  ��  NRF24L01��ȡ���ݰ���
  * @��  ��  *pbuf��������������ָ��
  * @����ֵ  ��
  */
static void NRF24L01_ReadPacket(uint8_t *pbuf)
{
    NRF24L01_CE_L();	//����CE���������ģʽ��׼����ʼ��SI24R1�еļĴ�����д������

    NRF24L01_ReadBuf(RD_RX_PLOAD,pbuf, NRF24L01_RX_PLOAD_WIDTH);  //��RX�˵�FIFO�ж�ȡ���ݣ�������ָ��������
    NRF24L01_WriteReg(FLUSH_RX,0xff);      // ��RX FIFO�Ĵ���

    NRF24L01_CE_H(); 	//����CE��׼�����ܴ��ⲿ���͹���������
}

/**
  * @��  ��  SPI2д���ȡһ���ֽں���
  * @��  ��  dat��Ҫд����ֽ�
  * @����ֵ  �����ֽ�
  */
static uint8_t SPI1_Send_Byte(uint8_t dat)
{
    /* Loop while DR register in not emplty */
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

    /* Send byte through the SPI2 peripheral */
    SPI_I2S_SendData(SPI1, dat);

    /* Wait to receive a byte */
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

    /* Return the byte read from the SPI bus */
    return SPI_I2S_ReceiveData(SPI1);
}

/**
  * @��  ��  NRF24L01��ȡָ���Ĵ������ݺ���
  * @��  ��  reg���Ĵ�����ַ
  * @����ֵ  �Ĵ�������
  */
static uint8_t NRF24L01_ReadReg(uint8_t reg)
{
    uint8_t reg_val;

    NRF24L01_CSN_L();                            // CSN low, initialize SPI communication...
    SPI1_Send_Byte(reg);            // Select register to read from..
    reg_val = SPI1_Send_Byte(0);    // ..then read registervalue
    NRF24L01_CSN_H();                            // CSN high, terminate SPI communication

    return(reg_val);                    // return register value
}

/**
  * @��  ��  NRF24L01д�����ݵ�ָ���Ĵ�������
  * @��  ��  reg���Ĵ�����ַ
  *          dat��д������
  * @����ֵ  ״ֵ̬
  */
static uint8_t NRF24L01_WriteReg(uint8_t reg, uint8_t dat)
{
    uint8_t status;

    NRF24L01_CSN_L();                            // CSN low, init SPI transaction
    status = SPI1_Send_Byte(reg);   // select register
    SPI1_Send_Byte(dat);          // ..and write value to it..
    NRF24L01_CSN_H();                            // CSN high again

    return(status);                     // return nRF24L01 status uint8_t
}

/**
  * @��  ��  ָ���Ĵ�����ַ����ָ���������ݺ���
  * @��  ��  reg���Ĵ�����ַ
  *          *pBuf������ָ��
  *          len�����ݳ���
  * @����ֵ  ״ֵ̬
  */
static uint8_t NRF24L01_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status,i;

    NRF24L01_CSN_L();                    		// Set CSN low, init SPI tranaction
    status = SPI1_Send_Byte(reg);       		// Select register to write to and read status uint8_t

    for(i=0; i<len; i++) {
        pBuf[i] = SPI1_Send_Byte(0);
    }

    NRF24L01_CSN_H();

    return(status);                    // return nRF24L01 status uint8_t
}
/**
  * @��  ��  ָ���Ĵ�����ַд��ָ���������ݺ���
  * @��  ��  reg���Ĵ�����ַ
  *          *pBuf������ָ��
  *          len�����ݳ���
  * @����ֵ  ״ֵ̬
  */
static uint8_t NRF24L01_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status,i;

    NRF24L01_CSN_L();            //SPIʹ��
    status = SPI1_Send_Byte(reg);
    for(i=0; i<len; i++) { //
        SPI1_Send_Byte(*pBuf);
        pBuf ++;
    }
    NRF24L01_CSN_H();           //�ر�SPI
    return(status);    //
}


/******************* (C) ��Ȩ 2018 XTARK **************************************/
