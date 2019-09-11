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
  * @文  件  ax_nrf24l01.h
  * @作  者  Musk Han@XTARK
  * @版  本  V1.0
  * @日  期  2018-10-26
  * @内  容  X-REMOTE NRF24L01无线通信
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_NRF24L01_H
#define __AX_NRF24L01_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "FreeRTOS.h"
//NRF24L01 地址宽度
#define NRF24L01_TX_ADR_WIDTH    5   	//发送地址宽度
#define NRF24L01_RX_ADR_WIDTH    5   	//接收地址宽度

//NRF24L01 数据宽度
#define NRF24L01_TX_PLOAD_WIDTH  32  	//发送数据宽度
#define NRF24L01_RX_PLOAD_WIDTH  32  	//接收数据宽度	 

/*******X-SOFT软件生态 接口规范的代码共享软件生态 ******/
extern uint8_t ax_flag_nrf24l01_rx_ok;
extern uint8_t ax_nrf24l01_rxbuf[NRF24L01_RX_PLOAD_WIDTH];
//X-REMOTE NRF24L01无线通信操作函数
uint8_t AX_NRF24L01_Init(void);  //NRF24L01初始化

void AX_NRF24L01_SetTxAddress(uint8_t addr);  //NRF24L01 设置发送地址(对方的接收地址)
void AX_NRF24L01_SetRxAddress(uint8_t addr);  //NRF24L01设置接收地址（本机接收地址）
void AX_NRF24L01_SetFrqChannel(uint8_t chr);  // NRF24L01设置频率点
void AX_NRF24L01_SetSpeed(uint8_t rate);  //NRF24L01设置空中速率
uint8_t NRF24L01_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_ReadBuf       (uint8_t reg, uint8_t *pBuf, uint8_t len);

void AX_NRF24L01_SendPacket(uint8_t *pbuf);    //NRF24L01发送数据包

#endif

/******************* (C) 版权 2018 XTARK **************************************/
