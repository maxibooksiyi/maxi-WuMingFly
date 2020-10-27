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
#include "Bling.h"

/***************************************************
������: void Bling_Init(void)
˵��:	LED״ָ̬ʾ�Ƴ�ʼ��
���:	��
����:	��
��ע:	�ϵ��ʼ��������һ��
****************************************************/
void Bling_Init()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|
                         RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|
                           RCC_APB2Periph_AFIO,ENABLE);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
	GPIO_SetBits(GPIOC,GPIO_Pin_6);
	GPIO_SetBits(GPIOC,GPIO_Pin_7);
	GPIO_SetBits(GPIOC,GPIO_Pin_8);
}


Bling_Light Light_1,Light_2,Light_3;
uint16_t Bling_Mode=0;
/***************************************************
������: void Bling_Set(Bling_Light *Light,
uint32_t Continue_time,//����ʱ��
uint16_t Period,//����100ms~1000ms
float Percent,//0~100%
uint16_t  Cnt,
GPIO_TypeDef* Port,
uint16_t Pin
,uint8_t Flag)
˵��:	״ָ̬ʾ�����ú���
���:	ʱ�䡢���ڡ�ռ�ձȡ��˿ڵ�
����:	��
��ע:	�����ʼ����ʼ������
****************************************************/
void Bling_Set(Bling_Light *Light,
               uint32_t Continue_time,//����ʱ��
               uint16_t Period,//����100ms~1000ms
               float Percent,//0~100%
               uint16_t  Cnt,
               GPIO_TypeDef* Port,
               uint16_t Pin
                 ,uint8_t Flag)
{
  Light->Bling_Contiune_Time=(uint16_t)(Continue_time/5);//����ʱ��
  Light->Bling_Period=Period;//����
  Light->Bling_Percent=Percent;//ռ�ձ�
  Light->Bling_Cnt=Cnt;
  Light->Port=Port;//�˿�
  Light->Pin=Pin;//����
  Light->Endless_Flag=Flag;//�޾�ģʽ
}

/***************************************************
������: void Bling_Process(Bling_Light *Light)//��˸�����߳�
˵��:	״ָ̬ʾ��ʵ��
���:	״̬�ƽṹ��
����:	��
��ע:	�����ʼ����ʼ������
****************************************************/
void Bling_Process(Bling_Light *Light)//��˸�����߳�
{
  if(Light->Bling_Contiune_Time>=1)  Light->Bling_Contiune_Time--;
  else GPIO_SetBits(Light->Port,Light->Pin);//�ø�,��
  if(Light->Bling_Contiune_Time!=0//��ʱ��δ��0
     ||Light->Endless_Flag==1)//�ж��޾�ģʽ�Ƿ���
  {
    Light->Bling_Cnt++;
    if(5*Light->Bling_Cnt>=Light->Bling_Period) Light->Bling_Cnt=0;//��������
    if(5*Light->Bling_Cnt<=Light->Bling_Period*Light->Bling_Percent)
      GPIO_ResetBits(Light->Port,Light->Pin);//�øߣ���
    else GPIO_SetBits(Light->Port,Light->Pin);//�õͣ���
  }
}

/***************************************************
������: Bling_Working(uint16 bling_mode)
˵��:	״ָ̬ʾ��״̬��
���:	��ǰģʽ
����:	��
��ע:	�����ʼ����ʼ������
****************************************************/
void Bling_Working(uint16 bling_mode)
{
  if(bling_mode==0)//ȫ��
  {
    Bling_Process(&Light_1);
    Bling_Process(&Light_2);
    Bling_Process(&Light_3);
  }
  else if(bling_mode==1)//���ٶȼ�6��У׼ģʽ
  {
    if(flight_direction==0)//��һ��У׼׼��
    {
      Bling_Process(&Light_1);
      GPIO_SetBits(Light_2.Port,Light_2.Pin);
      GPIO_SetBits(Light_3.Port,Light_3.Pin);
    }
    else if(flight_direction==1)//�ڶ���У׼׼��
    {
      Bling_Process(&Light_2);
      GPIO_SetBits(Light_1.Port,Light_1.Pin);
      GPIO_SetBits(Light_3.Port,Light_3.Pin);
    }
    else if(flight_direction==2)//������У׼׼��
    {
      Bling_Process(&Light_1);
      Bling_Process(&Light_2);
      GPIO_SetBits(Light_3.Port,Light_3.Pin);
    }
    else if(flight_direction==3)//������У׼׼��
    {
      Bling_Process(&Light_3);
      GPIO_SetBits(Light_1.Port,Light_1.Pin);
      GPIO_SetBits(Light_2.Port,Light_2.Pin);
    }
    else if(flight_direction==4)//������У׼׼��
    {
      Bling_Process(&Light_1);
      Bling_Process(&Light_3);
      GPIO_SetBits(Light_2.Port,Light_2.Pin);
    }
    else if(flight_direction==5)//������У׼׼��
    {
      GPIO_SetBits(Light_1.Port,Light_1.Pin);
      Bling_Process(&Light_2);
      Bling_Process(&Light_3);
    }
    else
    {
      Bling_Process(&Light_1);
      Bling_Process(&Light_2);
      Bling_Process(&Light_3);
    }
  }
  else if(bling_mode==2)//������У׼ģʽ
  {
    if(Mag_Calibration_Mode==0)//ˮƽ��У׼
    {
      Bling_Process(&Light_1);
      GPIO_SetBits(Light_2.Port,Light_2.Pin);
      GPIO_SetBits(Light_3.Port,Light_3.Pin);
    }
    else if(Mag_Calibration_Mode==1)////��ֱƽ��У׼
    {
      GPIO_SetBits(Light_1.Port,Light_1.Pin);
      Bling_Process(&Light_2);
      GPIO_SetBits(Light_3.Port,Light_3.Pin);
    }
    else if(Mag_Calibration_Mode==2)////��ֱƽ��У׼
    {
      GPIO_SetBits(Light_1.Port,Light_1.Pin);
      GPIO_SetBits(Light_2.Port,Light_2.Pin);
      Bling_Process(&Light_3);
    }
    else
    {
      Bling_Process(&Light_1);
      Bling_Process(&Light_2);
      Bling_Process(&Light_3);
    }
  }
  else if(bling_mode==3)//ȫ��
  {
    GPIO_SetBits(GPIOC,GPIO_Pin_6);
    GPIO_SetBits(GPIOC,GPIO_Pin_7);
    GPIO_SetBits(GPIOC,GPIO_Pin_8);
  }
  
  
}


/***************************************************
������: void Quad_Start_Bling(void)
˵��:	LED��ʼ���󿪻���˸
���:	��
����:	��
��ע:	�ϵ��ʼ��������һ��
****************************************************/
void Quad_Start_Bling()
{
  Bling_Set(&Light_1,2000,200,0.5,0,GPIOC,GPIO_Pin_6,0);
  Bling_Set(&Light_2,2000,500,0.5,0,GPIOC,GPIO_Pin_7,0);
  Bling_Set(&Light_3,2000,800,0.5,0,GPIOC,GPIO_Pin_8,0);
}

