/**			____                   _____ _________   __
		   / __ \                 |  __ \__   __\ \ / /
		  | |  | |_ __   ___ _ __ | |__) | | |   \ V /
		  | |  | | '_ \ / _ \ '_ \|  _  /  | |    > <
		  | |__| | |_) |  __/ | | | | \ \  | |   / . \
		   \____/| .__/ \___|_| |_|_|  \_\ |_|  /_/ \_\
				 | |
				 |_|  X-REMOTE遥控器 - OpenRTX开源软件系统

  ******************************************************************************
  *
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 官网网站： www.xtark.cn
  * 淘宝店铺： https://shop246676508.taobao.com
  * 塔克媒体： www.cnblogs.com/xtark（博客）
  *
  ******************************************************************************
  * @文  件  ax_nrf24l01.c
  * @作  者  Musk Han@XTARK
  * @版  本  V1.0
  * @日  期  2018-10-26
  * @内  容  X-REMOTE NRF24L01无线通信
  *
  ******************************************************************************
  * @说  明
  *
  * 1.NRF24L01无线收发程序，默认为接收模式，发送函数自动切换为发射模式
  *   发送完成后，自动切换到接收模式，等待接收数据
  * 2.IRQ处理 采用中断处理方式，通过读取NRF24L01中断标志寄存器获取中断类型
  * 3.LED灯进行发送状态指示，发送成功则LED取反闪烁，超过重发次数则LED熄灭
  * 4.发射和接收地址默认前四位为0xAA，最后一位可通过接口修改
  * 5.每次数据发送字节固定为32字节，NRF24L01最大字节
  *
  ******************************************************************************
  */

#include "ax_nrf24l01.h"
#include "task.h"

#include <stdio.h>

//全局变量
uint8_t ax_flag_nrf24l01_rx_ok = 0;
uint8_t ax_nrf24l01_rxbuf[NRF24L01_RX_PLOAD_WIDTH];  //数据接收缓冲区，数据发送缓冲区
/*数据通道0被用来接收确认信息，因此发送端的数据通道0的地址必须等于发送地址，这样才能收到应答信息*/
uint8_t AX_NRF24L01_TX_ADDRESS[NRF24L01_TX_ADR_WIDTH]= {0xAA,0xAA,0xAA,0xAA,0x01};   //远端地址
uint8_t AX_NRF24L01_RX_ADDRESS[NRF24L01_RX_ADR_WIDTH] = {0xAA,0xAA,0xAA,0xAA,0x01};   //本机地址

//IO操作宏定义
#define NRF24L01_CE_H()   GPIO_SetBits(GPIOB, GPIO_Pin_0)
#define NRF24L01_CE_L()   GPIO_ResetBits(GPIOB, GPIO_Pin_0)
#define NRF24L01_CSN_H()  GPIO_SetBits(GPIOB, GPIO_Pin_14)
#define NRF24L01_CSN_L()  GPIO_ResetBits(GPIOB, GPIO_Pin_14)
#define NRF24L01_IRQ     (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1))

//NRF24L01 操作状态
#define NRF24L01_TX_MAX  	 0x10    //达到最大发送次数标志
#define NRF24L01_TX_DS   	 0x20    //发送数据完成标志
#define NRF24L01_RX_DR   	 0x40    //接收数据完成标志

//NRF24L01 寄存器指令
#define NREAD_REG       0x00  	// 读寄存器指令
#define NWRITE_REG      0x20 	// 写寄存器指令
#define RD_RX_PLOAD     0x61  	// 读取接收数据指令
#define WR_TX_PLOAD     0xA0  	// 写待发数据指令
#define FLUSH_TX        0xE1 	// 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2  	// 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NOP             0xFF  	// 保留

//NRF24L01 寄存器地址
#define CONFIG          0x00    // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01    // 自动应答功能设置
#define EN_RXADDR       0x02    // 可用信道设置
#define SETUP_AW        0x03    // 收发地址宽度设置
#define SETUP_RETR      0x04    // 自动重发功能设置
#define RF_CH           0x05    // 工作频率设置
#define RF_SETUP        0x06    // 发射速率、功耗功能设置
#define STATUS          0x07    // 状态寄存器
#define OBSERVE_TX      0x08    // 发送监测功能
#define CD              0x09    // 地址检测           
#define RX_ADDR_P0      0x0A    // 频道0接收数据地址
#define RX_ADDR_P1      0x0B    // 频道1接收数据地址
#define RX_ADDR_P2      0x0C    // 频道2接收数据地址
#define RX_ADDR_P3      0x0D    // 频道3接收数据地址
#define RX_ADDR_P4      0x0E    // 频道4接收数据地址
#define RX_ADDR_P5      0x0F    // 频道5接收数据地址
#define TX_ADDR         0x10    // 发送地址寄存器
#define RX_PW_P0        0x11    // 接收频道0接收数据长度
#define RX_PW_P1        0x12    // 接收频道1接收数据长度
#define RX_PW_P2        0x13    // 接收频道2接收数据长度
#define RX_PW_P3        0x14    // 接收频道3接收数据长度
#define RX_PW_P4        0x15    // 接收频道4接收数据长度
#define RX_PW_P5        0x16    // 接收频道5接收数据长度
#define FIFO_STATUS     0x17    // FIFO栈入栈出状态寄存器设置

//文件内部函数声明
static uint8_t SPI1_Send_Byte(uint8_t dat);
static uint8_t NRF24L01_ReadReg(uint8_t reg);
static uint8_t NRF24L01_WriteReg(uint8_t reg, uint8_t dat);
static uint8_t NRF24L01_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t len);
static uint8_t NRF24L01_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len);
static void NRF24L01_SetRxMode(void);  //NRF24L01设置接收模式
static void NRF24L01_ReadPacket(uint8_t *pbuf);  //读取接收的数据包

/**
  * @简  述  NRF24L01初始化。
  * @参  数  无
  * @返回值  初始化结果  1 初始化成功 0 初始化失败
  */
uint8_t AX_NRF24L01_Init(void)
{
    uint8_t buf[5]= {0XA5,0XA5,0XA5,0XA5,0XA5};

    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_initStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    //开GPIO时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);

    //SCK
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB13/14/15复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOB

    //CE
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //CSN
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //IRQ，NRF中断产生时，IRQ引脚会被拉低
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //SPI配置
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;  //SPI_CPOL_High=模式3，时钟空闲为高 //SPI_CPOL_Low=模式0，时钟空闲为低
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//SPI_CPHA_2Edge;//SPI_CPHA_1Edge, SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;   //SPI_NSS_Soft;//SPI_NSS_Hard
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//10.5M;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//数据从高位开始发送
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    //Enable SPI
    SPI_Cmd(SPI1, ENABLE);

    //配置IRQ中断源
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;//配置抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//配置子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能中断通道
    NVIC_Init(&NVIC_InitStructure);

    //开启IRQ中断
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource1);//开启GPIO管脚的中断线路

    EXTI_initStructure.EXTI_Line = EXTI_Line1;
    EXTI_initStructure.EXTI_LineCmd = ENABLE;
    EXTI_initStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_initStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//下降沿触发
    EXTI_Init(&EXTI_initStructure);

    /******NRF24L01设置******/
    NRF24L01_CE_L();    //使能芯片
    NRF24L01_CSN_H();   //不使能SPI片选

    //检测NRF24L01是否在位，判断读出地址是否与写入地址相同
    NRF24L01_WriteBuf(NWRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.
    NRF24L01_ReadBuf(TX_ADDR,buf,5); //读出写入的地址

    if(buf[0] == 0XA5) { //检测到24L01,设置NRF24L01，接收模式
        NRF24L01_WriteBuf(NWRITE_REG + TX_ADDR, AX_NRF24L01_TX_ADDRESS, NRF24L01_TX_ADR_WIDTH);       //配置远端地址
        NRF24L01_WriteBuf(NWRITE_REG + RX_ADDR_P0, AX_NRF24L01_RX_ADDRESS, NRF24L01_RX_ADR_WIDTH);    //配置本机地址

        NRF24L01_WriteReg(NWRITE_REG + SETUP_RETR, 0x1a);   //设置自动重发时间和次数500us + 86us,重发次数10次
        NRF24L01_WriteReg(NWRITE_REG + EN_AA, 0x01);       //接收数据后，允许频道0自动应答
        NRF24L01_WriteReg(NWRITE_REG + EN_RXADDR, 0x01);   //使能通道0的接收地址
        NRF24L01_WriteReg(NWRITE_REG + RF_SETUP, 0x0f);    //设置发射速率为2MHZ，发射功率为最大值0dB 低噪声增益开启
        NRF24L01_WriteReg(NWRITE_REG + RF_CH, 40);         //  设置信道工作为2.4GHZ，收发必须一致
        NRF24L01_WriteReg(NWRITE_REG + RX_PW_P0, NRF24L01_RX_PLOAD_WIDTH); //设置接收数据长度，本次设置为32字节
        NRF24L01_WriteReg(NWRITE_REG + CONFIG, 0x0f);     //配置基本工作模式的参数；PWR_UP,EN_CRC,16BIT_CRC,接收模式，	开启所有 接收中断

        //清除接收中断标志
        NRF24L01_WriteReg(NWRITE_REG+STATUS, NRF24L01_RX_DR);   //接收到数据后RX_DR,TX_DS,MAX_RT都置高为1，通过写1来清除中断标志
        NRF24L01_WriteReg(FLUSH_RX,0xff);      // 清RX FIFO寄存器

        NRF24L01_CE_H();
        return 1;
    } else { //未检测到24L01或24L01硬件故障
        NRF24L01_CE_H();
        return 0;
    }
}

/**
  * @简  述  NRF24L01 设置发送地址(对方的接收地址)
  * @参  数  addr：最后一位地址，前面地址为0xAA，例如：0xAA,0xAA,0xAA,0xAA,0x01
  * @返回值  无
  */
void AX_NRF24L01_SetTxAddress(uint8_t addr)
{
    //最后一位改为设置地址
    AX_NRF24L01_TX_ADDRESS[NRF24L01_TX_ADR_WIDTH - 1] = addr;

    NRF24L01_CE_L();		//拉低CE，进入待机模式，准备写入数据

    NRF24L01_WriteBuf(NWRITE_REG + TX_ADDR, AX_NRF24L01_TX_ADDRESS, NRF24L01_TX_ADR_WIDTH);       //配置远端地址

    //清除接收中断标志
    NRF24L01_WriteReg(NWRITE_REG+STATUS, NRF24L01_RX_DR);   //接收到数据后RX_DR,TX_DS,MAX_RT都置高为1，通过写1来清除中断标志
    NRF24L01_WriteReg(FLUSH_RX,0xff);      // 清RX FIFO寄存器

    NRF24L01_CE_H();		 //置高CE，激发数据发送
}

/**
  * @简  述  NRF24L01设置接收地址（本机接收地址）
  * @参  数  addr：最后一位地址，前面地址为0xAA，例如：0xAA,0xAA,0xAA,0xAA,0x01
  * @返回值  无
  */
void AX_NRF24L01_SetRxAddress(uint8_t addr)
{
    //最后一位改为设置地址
    AX_NRF24L01_RX_ADDRESS[NRF24L01_RX_ADR_WIDTH - 1] = addr;

    NRF24L01_CE_L();		//拉低CE，进入待机模式，准备写入数据

    NRF24L01_WriteBuf(NWRITE_REG + RX_ADDR_P0, AX_NRF24L01_RX_ADDRESS, NRF24L01_RX_ADR_WIDTH);    //配置本机地址

    //清除接收中断标志
    NRF24L01_WriteReg(NWRITE_REG+STATUS, NRF24L01_RX_DR);   //接收到数据后RX_DR,TX_DS,MAX_RT都置高为1，通过写1来清除中断标志
    NRF24L01_WriteReg(FLUSH_RX,0xff);      // 清RX FIFO寄存器

    NRF24L01_CE_H();		 //置高CE，激发数据发送
}

/**
  * @简  述  NRF24L01设置频率点
  * @参  数  chr：频率点，范围为0~127
  * @返回值  无
  */
void AX_NRF24L01_SetFrqChannel(uint8_t chr)
{
    //参数过滤
    if(chr>127)
        chr = 127;

    //拉低CE，进入待机模式，准备写入数据
    NRF24L01_CE_L();

    //设置信道工作为2.4GHZ，收发必须一致
    NRF24L01_WriteReg(NWRITE_REG + RF_CH, chr);

    //清除接收中断标志
    NRF24L01_WriteReg(NWRITE_REG+STATUS, NRF24L01_RX_DR);   //接收到数据后RX_DR,TX_DS,MAX_RT都置高为1，通过写1来清除中断标志
    NRF24L01_WriteReg(FLUSH_RX,0xff);      // 清RX FIFO寄存器

    //置高CE，激发数据发送
    NRF24L01_CE_H();

}

/**
  * @简  述  NRF24L01设置空中速率
  * @参  数  rate：空中速率，（0-250K 1-1M 2-2M）
  * @返回值  无
  */
void AX_NRF24L01_SetSpeed(uint8_t rate)
{
    NRF24L01_CE_L();		//拉低CE，进入待机模式，准备写入数据

    if(rate == 0)
        NRF24L01_WriteReg(NWRITE_REG + RF_SETUP, 0x27);    //设置发射速率为250KHZ，发射功率为最大值0dB 低噪声增益开启
    else if(rate == 1)
        NRF24L01_WriteReg(NWRITE_REG + RF_SETUP, 0x07);    //设置发射速率为1MHZ，发射功率为最大值0dB 低噪声增益开启
    else if(rate == 2)
        NRF24L01_WriteReg(NWRITE_REG + RF_SETUP, 0x0f);    //设置发射速率为2MHZ，发射功率为最大值0dB 低噪声增益开启

    //清除接收中断标志
    NRF24L01_WriteReg(NWRITE_REG+STATUS, NRF24L01_RX_DR);   //接收到数据后RX_DR,TX_DS,MAX_RT都置高为1，通过写1来清除中断标志
    NRF24L01_WriteReg(FLUSH_RX,0xff);      // 清RX FIFO寄存器

    NRF24L01_CE_H();		 //置高CE，激发数据发送
}

/**
  * @简  述  NRF24L01发送数据包
  * @参  数  *pbuf：待发送数组指针
  * @返回值  无
  */
void AX_NRF24L01_SendPacket(uint8_t *pbuf)
{
    //拉低CE，进入待机模式，准备写入数据
    NRF24L01_CE_L();

    //将数据写入TX端的FIFO中,写入的个数与TX_PLOAD_WIDTH设置值相同
    NRF24L01_WriteBuf(WR_TX_PLOAD, pbuf, NRF24L01_TX_PLOAD_WIDTH);

    NRF24L01_WriteReg(NWRITE_REG + CONFIG, 0x0e);     //配置基本工作模式的参数；PWR_UP,EN_CRC,16BIT_CRC,发送模式，	开启所有中断
    NRF24L01_WriteReg(NWRITE_REG + STATUS, 0x7e);	  //写0111 xxxx 给STATUS，清除所有中断标志，防止一进入接收模式就触发中断

    //置高CE，激发数据发送
    NRF24L01_CE_H();

    //延时1ms
    vTaskDelay(1);
}

/**
  * @简  述  NRF24L01 IRQ中断处理
  * @参  数  无
  * @返回值  无
  */
void EXTI1_IRQHandler(void)
{
    uint8_t status;

    //是否产生了EXTI Line中断
    if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
        //中断处理
        NRF24L01_CE_L();		//拉低CE，读取数据
        status = NRF24L01_ReadReg(STATUS);	// 读取状态寄存其来判断数据接收状况

        //数据发送成功中断（收到应答信号）
        if( status & NRF24L01_TX_DS) {
            //设置接收模式，等待接收数据
            NRF24L01_SetRxMode();
            NRF24L01_WriteReg(NWRITE_REG + STATUS, NRF24L01_TX_DS);  //清除TX_DS中断标志
            NRF24L01_WriteReg(FLUSH_TX, 0xff);   //清除 TX_FIFO 寄存器

            //用户代码

        }

        //接收成功中断
        else if( status & NRF24L01_RX_DR) {
            //读取数据
            NRF24L01_ReadPacket(ax_nrf24l01_rxbuf);
            NRF24L01_WriteReg(NWRITE_REG+STATUS, NRF24L01_RX_DR);   //接收到数据后RX_DR,TX_DS,MAX_RT都置高为1，通过写1来清除中断标志

            //用户代码
            ax_flag_nrf24l01_rx_ok = 1;
        }

        //超出重发次数中断
        else if(status & NRF24L01_TX_MAX) {
            //进入接收模式，清除中断、FIFO
            NRF24L01_SetRxMode();
            NRF24L01_WriteReg(NWRITE_REG + STATUS, NRF24L01_TX_MAX);  //清除 TX_DS 或 MAX_RT中断标志
            NRF24L01_WriteReg(FLUSH_TX, 0xff);   //清除 TX_FIFO 寄存器

            //用户处理代码

        }

        //清除中断标志位
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}


/* NRF24L01 内部底层操作函数-----------------------------------------------------*/
/**
  * @简  述  设置为接收模式
  * @参  数  无
  * @返回值  无
  */
static void NRF24L01_SetRxMode(void)
{
    NRF24L01_CE_L();	//拉低CE，进入待机模式，准备开始往SI24R1中的寄存器中写入数据

    NRF24L01_WriteReg(NWRITE_REG + CONFIG, 0x0f);    //配置基本工作模式的参数；PWR_UP,EN_CRC,16BIT_CRC,接收模式，	开启所有 接收中断
    NRF24L01_WriteReg(NWRITE_REG + STATUS, 0x7e);	//写0111 xxxx 给STATUS，清除所有中断标志，防止一进入接收模式就触发中断

    NRF24L01_CE_H(); 	//拉高CE，准备接受从外部发送过来的数据
}
/**
  * @简  述  NRF24L01读取数据包。
  * @参  数  *pbuf：接收数据数组指针
  * @返回值  无
  */
static void NRF24L01_ReadPacket(uint8_t *pbuf)
{
    NRF24L01_CE_L();	//拉低CE，进入待机模式，准备开始往SI24R1中的寄存器中写入数据

    NRF24L01_ReadBuf(RD_RX_PLOAD,pbuf, NRF24L01_RX_PLOAD_WIDTH);  //从RX端的FIFO中读取数据，并存入指定的区域
    NRF24L01_WriteReg(FLUSH_RX,0xff);      // 清RX FIFO寄存器

    NRF24L01_CE_H(); 	//拉高CE，准备接受从外部发送过来的数据
}

/**
  * @简  述  SPI2写入读取一个字节函数
  * @参  数  dat：要写入的字节
  * @返回值  读出字节
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
  * @简  述  NRF24L01读取指定寄存器数据函数
  * @参  数  reg：寄存器地址
  * @返回值  寄存器数据
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
  * @简  述  NRF24L01写入数据到指定寄存器函数
  * @参  数  reg：寄存器地址
  *          dat：写入数据
  * @返回值  状态值
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
  * @简  述  指定寄存器地址读出指定长度数据函数
  * @参  数  reg：寄存器地址
  *          *pBuf：数据指针
  *          len：数据长度
  * @返回值  状态值
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
  * @简  述  指定寄存器地址写入指定长度数据函数
  * @参  数  reg：寄存器地址
  *          *pBuf：数据指针
  *          len：数据长度
  * @返回值  状态值
  */
static uint8_t NRF24L01_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status,i;

    NRF24L01_CSN_L();            //SPI使能
    status = SPI1_Send_Byte(reg);
    for(i=0; i<len; i++) { //
        SPI1_Send_Byte(*pBuf);
        pBuf ++;
    }
    NRF24L01_CSN_H();           //关闭SPI
    return(status);    //
}


/******************* (C) 版权 2018 XTARK **************************************/
