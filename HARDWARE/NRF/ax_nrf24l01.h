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
  * @��  ��  ax_nrf24l01.h
  * @��  ��  Musk Han@XTARK
  * @��  ��  V1.0
  * @��  ��  2018-10-26
  * @��  ��  X-REMOTE NRF24L01����ͨ��
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_NRF24L01_H
#define __AX_NRF24L01_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "FreeRTOS.h"
//NRF24L01 ��ַ���
#define NRF24L01_TX_ADR_WIDTH    5   	//���͵�ַ���
#define NRF24L01_RX_ADR_WIDTH    5   	//���յ�ַ���

//NRF24L01 ���ݿ��
#define NRF24L01_TX_PLOAD_WIDTH  32  	//�������ݿ��
#define NRF24L01_RX_PLOAD_WIDTH  32  	//�������ݿ��	 

/*******X-SOFT�����̬ �ӿڹ淶�Ĵ��빲�������̬ ******/
extern uint8_t ax_flag_nrf24l01_rx_ok;
extern uint8_t ax_nrf24l01_rxbuf[NRF24L01_RX_PLOAD_WIDTH];
//X-REMOTE NRF24L01����ͨ�Ų�������
uint8_t AX_NRF24L01_Init(void);  //NRF24L01��ʼ��

void AX_NRF24L01_SetTxAddress(uint8_t addr);  //NRF24L01 ���÷��͵�ַ(�Է��Ľ��յ�ַ)
void AX_NRF24L01_SetRxAddress(uint8_t addr);  //NRF24L01���ý��յ�ַ���������յ�ַ��
void AX_NRF24L01_SetFrqChannel(uint8_t chr);  // NRF24L01����Ƶ�ʵ�
void AX_NRF24L01_SetSpeed(uint8_t rate);  //NRF24L01���ÿ�������
uint8_t NRF24L01_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t len);
uint8_t NRF24L01_ReadBuf       (uint8_t reg, uint8_t *pBuf, uint8_t len);

void AX_NRF24L01_SendPacket(uint8_t *pbuf);    //NRF24L01�������ݰ�

#endif

/******************* (C) ��Ȩ 2018 XTARK **************************************/
