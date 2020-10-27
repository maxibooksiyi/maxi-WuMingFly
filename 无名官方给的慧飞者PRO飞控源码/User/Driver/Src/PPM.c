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
#include "Headfile.h"
#include "PPM.h"

/***************************************************
函数名: void PPM_Init(void)
说明:	PPM接收初始化
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void PPM_Init()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;//GPIO_Pin_0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//输入下拉
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd	= ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void PPM_UP()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//下拉输入
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd= ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void PPM_DN()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd= ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}


uint16 PPM_Sample_Cnt=0;
uint16 PPM_Isr_Cnt=0;
u32 Last_PPM_Time=0;
u32 PPM_Time=0;
u16 PPM_Time_Delta=0;
uint16 PPM_Is_Okay=0;
uint16 PPM_Databuf[10]={0};
/***************************************************
函数名: void EXTI9_5_IRQHandler(void)
说明:	PPM接收中断函数
入口:	无
出口:	无
备注:	程序初始化后、始终运行
****************************************************/
static uint16 PPM_buf[8]={0};
void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line8) != RESET)
  {
    //系统运行时间获取，单位us
    Last_PPM_Time=PPM_Time;
    PPM_Time=10000*TIME_ISR_CNT+TIM2->CNT;//us
    PPM_Time_Delta=PPM_Time-Last_PPM_Time;
    //PPM中断进入判断
    if(PPM_Isr_Cnt<100)  PPM_Isr_Cnt++;
    //PPM解析开始
    if(PPM_Is_Okay==1)
    {
      if(PPM_Time_Delta>=800&&PPM_Time_Delta<=2200)
      {
        PPM_Sample_Cnt++;
        //对应通道写入缓冲区
        PPM_buf[PPM_Sample_Cnt-1]=PPM_Time_Delta;
        //单次解析结束
        if(PPM_Sample_Cnt>=8)
        {
          memcpy(PPM_Databuf,PPM_buf,PPM_Sample_Cnt*sizeof(uint16));
          PPM_Is_Okay=0;
        }
      }
      else
      {
        if(PPM_Time_Delta>=2500)//帧结束电平至少2ms=2000us，由于部分老版本遥控器、
          //接收机输出PPM信号不标准，当出现解析异常时，尝试改小此值，该情况仅出现一例：使用天地飞老版本遥控器
        {
          memcpy( PPM_Databuf,PPM_buf,PPM_Sample_Cnt*sizeof(uint16));
          PPM_Is_Okay = 1;
          PPM_Sample_Cnt=0;
        }
        else  PPM_Is_Okay=0;
      }
    }
    else if(PPM_Time_Delta>=2500)//帧结束电平至少2ms=2000us
    {
      PPM_Is_Okay=1;
      PPM_Sample_Cnt=0;
    }
  }
  EXTI_ClearITPendingBit(EXTI_Line8);
}


