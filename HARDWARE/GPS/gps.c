#include "gps.h"
#include "led.h"
#include "delay.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
u8 USART4_RX_BUF[USART4_MAX_RECV_LEN]; 				//接收缓冲,最大USART3_MAX_RECV_LEN个字节.
u8  USART4_TX_BUF[USART4_MAX_SEND_LEN]; 			//发送缓冲,最大USART3_MAX_SEND_LEN字节
const u32 BAUD_id[9]= {4800,9600,19200,38400,57600,115200,230400,460800,921600}; //模块支持波特率数组
//通过判断接收连续2个字符之间的时间差不大于10ms来决定是不是一次连续的数据.
//如果2个字符接收间隔超过10ms,则认为不是1次连续数据.也就是超过10ms没有接收到
//任何数据,则表示此次接收完毕.
//接收到的数据状态
//[15]:0,没有接收到数据;1,接收到了一批数据.
//[14:0]:接收到的数据长度
vu16 USART4_RX_STA=0;

extern TaskHandle_t GPSTask_Handler;	//接收任务通知的任务句柄
#define GNRMC_LEN 6
u8 GPS_GNRMC[GNRMC_LEN+1]= {"$GNRMC"};
u8 gps_complete=1;
void UART4_IRQHandler(void)
{
    u8 res;
    static u8 gps_start=0;

    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) { //接收到数据
        if(gps_complete) {
            res =USART_ReceiveData(UART4);
            if((res==GPS_GNRMC[USART4_RX_STA]||gps_start==1)&&USART4_RX_STA<USART4_MAX_RECV_LEN) {
                USART4_RX_BUF[USART4_RX_STA++]=res;
            } else {
                USART4_RX_STA=0;
                gps_start=0;
                gps_complete=1;
            }

            if(USART4_RX_STA==GNRMC_LEN)
                gps_start=1;

            if(gps_start&&res=='\n') {
                gps_start=0;
                gps_complete=0;
                // USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
                USART_Cmd(UART4, DISABLE);
                vTaskNotifyGiveFromISR(GPSTask_Handler,NULL);//发送任务通知
            }
        }

        //while(1)
        //{
        //LED0=!LED0;
        //	delay_xms(10);
        ///	printf("abcde\r\n");
        //}
        USART_ClearITPendingBit(UART4,UART4_IRQn);

    }

}



//初始化IO 串口4
//pclk1:PCLK1时钟频率(Mhz)
//bound:波特率
void GPS_Init(u32 bound)
{

    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	// GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE); //串口3时钟使能


    USART_DeInit(UART4);
    //USART4_TX   PC10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure); //初始化PB10

    //USART4_RX	  PC11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOC, &GPIO_InitStructure);  //初始化PB11

    USART_InitStructure.USART_BaudRate = bound;//波特率一般设置为9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    //设置中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=10 ;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器


    //TIM7_Int_Init(1000-1,7200-1);		//10ms中断
    USART4_RX_STA=0;		//清零
    //TIM_Cmd(TIM7,DISABLE);			//关闭定时器7
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启中断
    USART_Init(UART4, &USART_InitStructure); //初始化串口	3
    USART_Cmd(UART4, DISABLE);

}

//串口3,printf 函数
//确保一次发送数据不超过USART3_MAX_SEND_LEN字节
void u4_printf(char* fmt,...)
{
    u16 i,j;
    va_list ap;
    va_start(ap,fmt);
    vsprintf((char*)USART4_TX_BUF,fmt,ap);
    va_end(ap);
    i=strlen((const char*)USART4_TX_BUF);		//此次发送数据的长度
    for(j=0; j<i; j++) {						//循环发送数据
        while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕
        USART_SendData(UART4,USART4_TX_BUF[j]);
    }

}


//从buf里面得到第cx个逗号所在的位置
//返回值:0~0XFE,代表逗号所在位置的偏移.
//       0XFF,代表不存在第cx个逗号
u8 NMEA_Comma_Pos(u8 *buf,u8 cx)
{
    u8 *p=buf;
    while(cx) {
        if(*buf=='*'||*buf<' '||*buf>'z')return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
        if(*buf==',')cx--;
        buf++;
    }
    return buf-p;
}
//m^n函数
//返回值:m^n次方.
u32 NMEA_Pow(u8 m,u8 n)
{
    u32 result=1;
    while(n--)result*=m;
    return result;
}
//str转换为数字,以','或者'*'结束
//buf:数字存储区
//dx:小数点位数,返回给调用函数
//返回值:转换后的数值
int NMEA_Str2num(u8 *buf,u8*dx)
{
    u8 *p=buf;
    u32 ires=0,fres=0;
    u8 ilen=0,flen=0,i;
    u8 mask=0;
    int res;
    while(1) { //得到整数和小数的长度
        if(*p=='-') {
            mask|=0X02;    //是负数
            p++;
        }
        if(*p==','||(*p=='*'))break;//遇到结束了
        if(*p=='.') {
            mask|=0X01;    //遇到小数点了
            p++;
        } else if(*p>'9'||(*p<'0')) {	//有非法字符
            ilen=0;
            flen=0;
            break;
        }
        if(mask&0X01)flen++;
        else ilen++;
        p++;
    }
    if(mask&0X02)buf++;	//去掉负号
    for(i=0; i<ilen; i++) {	//得到整数部分数据
        ires+=NMEA_Pow(10,ilen-1-i)*(buf[i]-'0');
    }
    if(flen>5)flen=5;	//最多取5位小数
    *dx=flen;	 		//小数点位数
    for(i=0; i<flen; i++) {	//得到小数部分数据
        fres+=NMEA_Pow(10,flen-1-i)*(buf[ilen+1+i]-'0');
    }
    res=ires*NMEA_Pow(10,flen)+fres;
    if(mask&0X02)res=-res;
    return res;
}
//分析GPGSV信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPGSV_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p,*p1,dx;
    u8 len,i,j,slx=0;
    u8 posx;
    p=buf;
    p1=(u8*)strstr((const char *)p,"$GPGSV");
    len=p1[7]-'0';								//得到GPGSV的条数
    posx=NMEA_Comma_Pos(p1,3); 					//得到可见卫星总数
    if(posx!=0XFF)gpsx->svnum=NMEA_Str2num(p1+posx,&dx);
    for(i=0; i<len; i++) {
        p1=(u8*)strstr((const char *)p,"$GPGSV");
        for(j=0; j<4; j++) {
            posx=NMEA_Comma_Pos(p1,4+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
            else break;
            posx=NMEA_Comma_Pos(p1,5+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角
            else break;
            posx=NMEA_Comma_Pos(p1,6+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
            else break;
            posx=NMEA_Comma_Pos(p1,7+j*4);
            if(posx!=0XFF)gpsx->slmsg[slx].sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
            else break;
            slx++;
        }
        p=p1+1;//切换到下一个GPGSV信息
    }
}
//分析BDGSV信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_BDGSV_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p,*p1,dx;
    u8 len,i,j,slx=0;
    u8 posx;
    p=buf;
    p1=(u8*)strstr((const char *)p,"$BDGSV");
    len=p1[7]-'0';								//得到BDGSV的条数
    posx=NMEA_Comma_Pos(p1,3); 					//得到可见北斗卫星总数
    if(posx!=0XFF)gpsx->beidou_svnum=NMEA_Str2num(p1+posx,&dx);
    for(i=0; i<len; i++) {
        p1=(u8*)strstr((const char *)p,"$BDGSV");
        for(j=0; j<4; j++) {
            posx=NMEA_Comma_Pos(p1,4+j*4);
            if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_num=NMEA_Str2num(p1+posx,&dx);	//得到卫星编号
            else break;
            posx=NMEA_Comma_Pos(p1,5+j*4);
            if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_eledeg=NMEA_Str2num(p1+posx,&dx);//得到卫星仰角
            else break;
            posx=NMEA_Comma_Pos(p1,6+j*4);
            if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_azideg=NMEA_Str2num(p1+posx,&dx);//得到卫星方位角
            else break;
            posx=NMEA_Comma_Pos(p1,7+j*4);
            if(posx!=0XFF)gpsx->beidou_slmsg[slx].beidou_sn=NMEA_Str2num(p1+posx,&dx);	//得到卫星信噪比
            else break;
            slx++;
        }
        p=p1+1;//切换到下一个BDGSV信息
    }
}
//分析GNGGA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GNGGA_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;
    u8 posx;
    p1=(u8*)strstr((const char *)buf,"$GNGGA");
    posx=NMEA_Comma_Pos(p1,6);								//得到GPS状态
    if(posx!=0XFF)gpsx->gpssta=NMEA_Str2num(p1+posx,&dx);
    posx=NMEA_Comma_Pos(p1,7);								//得到用于定位的卫星数
    if(posx!=0XFF)gpsx->posslnum=NMEA_Str2num(p1+posx,&dx);
    posx=NMEA_Comma_Pos(p1,9);								//得到海拔高度
    if(posx!=0XFF)gpsx->altitude=NMEA_Str2num(p1+posx,&dx);
}
//分析GNGSA信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GNGSA_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;
    u8 posx;
    u8 i;
    p1=(u8*)strstr((const char *)buf,"$GNGSA");
    posx=NMEA_Comma_Pos(p1,2);								//得到定位类型
    if(posx!=0XFF)gpsx->fixmode=NMEA_Str2num(p1+posx,&dx);
    for(i=0; i<12; i++) {									//得到定位卫星编号
        posx=NMEA_Comma_Pos(p1,3+i);
        if(posx!=0XFF)gpsx->possl[i]=NMEA_Str2num(p1+posx,&dx);
        else break;
    }
    posx=NMEA_Comma_Pos(p1,15);								//得到PDOP位置精度因子
    if(posx!=0XFF)gpsx->pdop=NMEA_Str2num(p1+posx,&dx);
    posx=NMEA_Comma_Pos(p1,16);								//得到HDOP位置精度因子
    if(posx!=0XFF)gpsx->hdop=NMEA_Str2num(p1+posx,&dx);
    posx=NMEA_Comma_Pos(p1,17);								//得到VDOP位置精度因子
    if(posx!=0XFF)gpsx->vdop=NMEA_Str2num(p1+posx,&dx);
}
//分析GNRMC信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GNRMC_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;
    u8 posx;
    u32 temp;
    float rs;
    p1=(u8*)strstr((const char *)buf,"$GNRMC");//"$GNRMC",经常有&和GNRMC分开的情况,故只判断GPRMC.
    posx=NMEA_Comma_Pos(p1,1);								//得到UTC时间
    if(posx!=0XFF) {
        temp=NMEA_Str2num(p1+posx,&dx)/NMEA_Pow(10,dx);	 	//得到UTC时间,去掉ms
        gpsx->utc.hour=temp/10000;
        gpsx->utc.min=(temp/100)%100;
        gpsx->utc.sec=temp%100;
    }
    posx=NMEA_Comma_Pos(p1,3);								//得到纬度
    if(posx!=0XFF) {
        temp=NMEA_Str2num(p1+posx,&dx);
        gpsx->latitude=temp/NMEA_Pow(10,dx+2);	//得到°
        rs=temp%NMEA_Pow(10,dx+2);				//得到'
        gpsx->latitude=gpsx->latitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为°
    }
    posx=NMEA_Comma_Pos(p1,4);								//南纬还是北纬
    if(posx!=0XFF)gpsx->nshemi=*(p1+posx);
    posx=NMEA_Comma_Pos(p1,5);								//得到经度
    if(posx!=0XFF) {
        temp=NMEA_Str2num(p1+posx,&dx);
        gpsx->longitude=temp/NMEA_Pow(10,dx+2);	//得到°
        rs=temp%NMEA_Pow(10,dx+2);				//得到'
        gpsx->longitude=gpsx->longitude*NMEA_Pow(10,5)+(rs*NMEA_Pow(10,5-dx))/60;//转换为°
    }
    posx=NMEA_Comma_Pos(p1,6);								//东经还是西经
    if(posx!=0XFF)gpsx->ewhemi=*(p1+posx);
    posx=NMEA_Comma_Pos(p1,9);								//得到UTC日期
    if(posx!=0XFF) {
        temp=NMEA_Str2num(p1+posx,&dx);		 				//得到UTC日期
        gpsx->utc.date=temp/10000;
        gpsx->utc.month=(temp/100)%100;
        gpsx->utc.year=2000+temp%100;
    }
}
//分析GNVTG信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GNVTG_Analysis(nmea_msg *gpsx,u8 *buf)
{
    u8 *p1,dx;
    u8 posx;
    p1=(u8*)strstr((const char *)buf,"$GNVTG");
    posx=NMEA_Comma_Pos(p1,7);								//得到地面速率
    if(posx!=0XFF) {
        gpsx->speed=NMEA_Str2num(p1+posx,&dx);
        if(dx<3)gpsx->speed*=NMEA_Pow(10,3-dx);	 	 		//确保扩大1000倍
    }
}
//提取NMEA-0183信息
//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void GPS_Analysis(nmea_msg *gpsx,u8 *buf)
{
    NMEA_GPGSV_Analysis(gpsx,buf);	//GPGSV解析
    NMEA_BDGSV_Analysis(gpsx,buf);	//BDGSV解析
    NMEA_GNGGA_Analysis(gpsx,buf);	//GNGGA解析
    NMEA_GNGSA_Analysis(gpsx,buf);	//GPNSA解析
    NMEA_GNRMC_Analysis(gpsx,buf);	//GPNMC解析
    NMEA_GNVTG_Analysis(gpsx,buf);	//GPNTG解析
}
///////////////////////////////////////////UBLOX 配置代码/////////////////////////////////////
////检查CFG配置执行情况
////返回值:0,ACK成功
////       1,接收超时错误
////       2,没有找到同步字符
////       3,接收到NACK应答
u8 SkyTra_Cfg_Ack_Check(void)
{
    u16 len=0,i;
    u8 rval=0;
    while((USART4_RX_STA&0X8000)==0 && len<100) { //等待接收到应答
        len++;
        delay_ms(5);
    }
    if(len<100) { 	//超时错误.
        len=USART4_RX_STA&0X7FFF;	//此次接收到的数据长度
        for(i=0; i<len; i++) {
            if(USART4_RX_BUF[i]==0X83)break;
            else if(USART4_RX_BUF[i]==0X84) {
                rval=3;
                break;
            }
        }
        if(i==len)rval=2;						//没有找到同步字符
    } else rval=1;								//接收超时错误
    USART4_RX_STA=0;							//清除接收
    return rval;
}
//配置SkyTra_GPS/北斗模块波特率
//baud_id:0~8，对应波特率,4800/9600/19200/38400/57600/115200/230400/460800/921600
//返回值:0,执行成功;其他,执行失败(这里不会返回0了)
u8 SkyTra_Cfg_Prt(u8 baud_id)
{
    SkyTra_baudrate *cfg_prt=(SkyTra_baudrate *)USART4_TX_BUF;
    cfg_prt->sos=0XA1A0;		//引导序列(小端模式)
    cfg_prt->PL=0X0400;			//有效数据长度(小端模式)
    cfg_prt->id=0X05;		    //配置波特率的ID
    cfg_prt->com_port=0X00;			//操作串口1
    cfg_prt->Baud_id=baud_id;	 	////波特率对应编号
    cfg_prt->Attributes=1; 		  //保存到SRAM&FLASH
    cfg_prt->CS=cfg_prt->id^cfg_prt->com_port^cfg_prt->Baud_id^cfg_prt->Attributes;
    cfg_prt->end=0X0A0D;        //发送结束符(小端模式)
    SkyTra_Send_Date((u8*)cfg_prt,sizeof(SkyTra_baudrate));//发送数据给SkyTra
    delay_ms(200);				//等待发送完成
    GPS_Init(BAUD_id[baud_id]);	//重新初始化串口3
    return SkyTra_Cfg_Ack_Check();//这里不会反回0,因为UBLOX发回来的应答在串口重新初始化的时候已经被丢弃了.
}
//配置SkyTra_GPS模块的时钟脉冲宽度
//width:脉冲宽度1~100000(us)
//返回值:0,发送成功;其他,发送失败.
u8 SkyTra_Cfg_Tp(u32 width)
{
    u32 temp=width;
    SkyTra_pps_width *cfg_tp=(SkyTra_pps_width *)USART4_TX_BUF;
    temp=(width>>24)|((width>>8)&0X0000FF00)|((width<<8)&0X00FF0000)|((width<<24)&0XFF000000);//小端模式
    cfg_tp->sos=0XA1A0;		    //cfg header(小端模式)
    cfg_tp->PL=0X0700;        //有效数据长度(小端模式)
    cfg_tp->id=0X65	;			    //cfg tp id
    cfg_tp->Sub_ID=0X01;			//数据区长度为20个字节.
    cfg_tp->width=temp;		  //脉冲宽度,us
    cfg_tp->Attributes=0X01;  //保存到SRAM&FLASH
    cfg_tp->CS=cfg_tp->id^cfg_tp->Sub_ID^(cfg_tp->width>>24)^(cfg_tp->width>>16)&0XFF^(cfg_tp->width>>8)&0XFF^cfg_tp->width&0XFF^cfg_tp->Attributes;    	//用户延时为0ns
    cfg_tp->end=0X0A0D;       //发送结束符(小端模式)
    SkyTra_Send_Date((u8*)cfg_tp,sizeof(SkyTra_pps_width));//发送数据给NEO-6M
    return SkyTra_Cfg_Ack_Check();
}
//配置SkyTraF8-BD的更新速率
//Frep:（取值范围:1,2,4,5,8,10,20,25,40,50）测量时间间隔，单位为Hz，最大不能大于50Hz
//返回值:0,发送成功;其他,发送失败.
u8 SkyTra_Cfg_Rate(u8 Frep)
{
    SkyTra_PosRate *cfg_rate=(SkyTra_PosRate *)USART4_TX_BUF;
    cfg_rate->sos=0XA1A0;	    //cfg header(小端模式)
    cfg_rate->PL=0X0300;			//有效数据长度(小端模式)
    cfg_rate->id=0X0E;	      //cfg rate id
    cfg_rate->rate=Frep;	 	  //更新速率
    cfg_rate->Attributes=0X01;	   	//保存到SRAM&FLASH	.
    cfg_rate->CS=cfg_rate->id^cfg_rate->rate^cfg_rate->Attributes;//脉冲间隔,us
    cfg_rate->end=0X0A0D;       //发送结束符(小端模式)
    SkyTra_Send_Date((u8*)cfg_rate,sizeof(SkyTra_PosRate));//发送数据给NEO-6M
    return SkyTra_Cfg_Ack_Check();
}
//发送一批数据给SkyTraF8-BD，这里通过串口3发送
//dbuf：数据缓存首地址
//len：要发送的字节数
void SkyTra_Send_Date(u8* dbuf,u16 len)
{
    u16 j;
    for(j=0; j<len; j++) { //循环发送数据
        while((UART4->SR&0X40)==0);//循环发送,直到发送完毕
        UART4->DR=dbuf[j];
    }
}







