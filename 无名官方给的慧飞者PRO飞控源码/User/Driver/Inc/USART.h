/*----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于无名科创团队，无名科创团队将飞控程序源码提供给购买者，
*               购买者要为无名科创团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人免费下载， 
*               更不能以此销售牟利，如发现上述行为，无名科创团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------/
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名科创开源飞控QQ群：540707961
*               CSDN博客：http://blog.csdn.net/u011992534
*               优酷ID：NamelessCotrun无名小哥
*               B站教学视频：https://space.bilibili.com/67803559/#/video
*               客户使用心得、改进意见征集贴：http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               百度贴吧:无名科创开源飞控
*               修改日期:2018/8/30
*               版本：慧飞者——V1.0.1
*               版权所有，盗版必究。
*               Copyright(C) 武汉科技大学无名科创团队 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
#ifndef __USART_H
#define __USART_H


void USART1_Init(unsigned long bound);
void USART1_Send(unsigned char *tx_buf, int len);
void USART1_Receive(unsigned char *rx_buf, int len);
void UART1_Send(unsigned char tx_buf);
void wust_sendccd(unsigned char *ccdaddr, int16_t ccdsize);
void wust_sendware(unsigned char *wareaddr, int16_t waresize);
void Quad_DMA1_USART1_SEND(u32 SendBuff,u16 len);//DMA---USART1传输
void DMA_Send_StateMachine(void);
void USART2_Init(unsigned long bound);

void USART3_Init(unsigned long bound);
void USART3_Send(unsigned char tx_buf);
void UART3_Send(unsigned char *tx_buf, int len);
void USART2_Send(unsigned char *tx_buf, int len);


void USART4_Init(unsigned long bound);
void SBUS_USART5_Init(void);

void ANO_Data_Send_Status(void);
void ANO_SEND_StateMachine(void);


extern uint8_t RecBag[3];
extern uint8 US_100_Cnt;


extern unsigned int GPS_Data_Cnt;
extern u16 GPS_ISR_CNT;
extern  u8 GPS_Buf[2][100];
extern uint8 Ublox_Data[95];
extern uint8_t ANO_Send_PID_Flag[6];
extern uint8_t ANO_Send_PID_Flag_USB[6];

extern uint16 GPS_Update_finished,GPS_Update_finished_Correct_Flag;
extern Testime GPS_Time_Delta;
extern RingBuff_t SBUS_Ringbuf,OpticalFlow_Ringbuf;

extern void ANO_DT_Data_Receive_Anl_USE_USB(u8 *data_buf,u8 num);
void ANO_SEND_StateMachine_USE_USB(void);
#endif


