/*----------------------------------------------------------------------------------------------------------------------/
*               ������ֻ��������ѧϰʹ�ã���Ȩ����Ȩ���������ƴ��Ŷӣ������ƴ��Ŷӽ��ɿس���Դ���ṩ�������ߣ�
*               ������ҪΪ�����ƴ��Ŷ��ṩ������δ��������ɣ����ý�Դ�����ṩ�����ˣ����ý�Դ����ŵ����Ϲ�����������أ� 
*               �������Դ�����Ĳ�����緢��������Ϊ�������ƴ��Ŷӽ���֮�Է��ɽ��������
-----------------------------------------------------------------------------------------------------------------------/
*               ������Ϣ���ܶ���ֹ��ǰ�����������˳���������
*               ��Դ���ף���ѧ����ϧ��ף������Ϯ�����׳ɹ�������
*               ѧϰ�����ߣ��������Ƽ���DJI��ZEROTECH��XAG��AEE��GDU��AUTEL��EWATT��HIGH GREAT�ȹ�˾��ҵ
*               ��ְ�����뷢�ͣ�15671678205@163.com���豸ע��ְ����λ����λ��������
*               �����ƴ���Դ�ɿ�QQȺ��540707961
*               CSDN���ͣ�http://blog.csdn.net/u011992534
*               �ſ�ID��NamelessCotrun����С��
*               Bվ��ѧ��Ƶ��https://space.bilibili.com/67803559/#/video
*               �ͻ�ʹ���ĵá��Ľ������������http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               �Ա����̣�https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               �ٶ�����:�����ƴ���Դ�ɿ�
*               �޸�����:2018/8/30
*               �汾���۷��ߡ���V1.0.1
*               ��Ȩ���У�����ؾ���
*               Copyright(C) �人�Ƽ���ѧ�����ƴ��Ŷ� 2017-2025
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/

#include "Headfile.h"
#include "delay.h"
u8   fac_us;
u16  fac_ms;
void delay_init(u8 SYSCLK)
{
  SysTick->CTRL &= 0xfffffffb;
  fac_us=SYSCLK/8;
  fac_ms=(u16)fac_us*1000;
}
void delay_us(u32 nus)
{
  u32 temp;
  SysTick->LOAD = nus*fac_us;
  SysTick->VAL = 0x00;
  SysTick->CTRL = 0x01;
  temp = SysTick->CTRL;
  while((temp&0x01)&&(!(temp&(1<<16))))temp = SysTick->CTRL;
  SysTick->CTRL=0x00;
  SysTick->VAL =0X00;
  
}
void delay_ms(u16 nms)
{
  u32 temp;
  SysTick->LOAD = (u32)nms*fac_ms;
  SysTick->VAL = 0x00;
  SysTick->CTRL = 0x01;
  temp = SysTick->CTRL;
  while((temp&0x01)&&(!(temp&(1<<16)))) temp = SysTick->CTRL;
  SysTick->CTRL=0x00;
  SysTick->VAL =0X00;
}

/*
#include "Headfile.h"
#include "delay.h"
u8   fac_us;
u16  fac_ms;
void delay_init(u8 SYSCLK)
{
SysTick->CTRL &= 0xfffffffb;
fac_us=SYSCLK/8;
fac_ms=(u16)fac_us*1000;
}

void delay_ms(uint16_t nms)
{
uint32_t t0=micros();
while(micros() - t0 < nms * 1000);
}
void delay_us(u32 nus)
{
uint32_t t0=micros();
while(micros() - t0 < nus);
}
*/

void Delay(vu32 nCount)
{
  for(; nCount!= 0;nCount--);
}

/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: 
** ��������: Delay_Ms
** ��������: ��ʱ1MS (��ͨ���������ж�����׼ȷ��)           
** ����������time (ms) ע��time<65535 
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/  
void Delay_Ms(uint16_t time)  //��ʱ����  
{   
  uint16_t i,j;  
  for(i=0;i<time;i++)  
    for(j=0;j<10260;j++);  
}  
/*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: 
** ��������: Delay_Us 
** ��������: ��ʱ1us (��ͨ���������ж�����׼ȷ��) 
** ����������time (us) ע��time<65535                 
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/  
void Delay_Us(uint16_t time)  //��ʱ����  
{   
  uint16_t i,j;  
  for(i=0;i<time;i++)  
    for(j=0;j<9;j++);  
}  


