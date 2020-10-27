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
#include "SBUS.h"

const float _sbusScale = 0.625749f; //Matlab��ϳ�����������
const float _sbusBias =  875.23353f;//Matlab��ϳ�����ƫ
uint16_t SBUS_Channel[16]={0};
uint16_t SBUS_Channel_User[16]={0};
/***************************************************
������: void Process(uint8_t *raw,uint16_t *result)
˵��:	SBUS���ݽ���
���:	ԭʼ���ݡ���������
����:	��
��ע:	��
ע���ߣ�����С��
****************************************************/
void Process(uint8_t *raw,uint16_t *result)
{
  uint8_t bitsToRead=3; // bitsToRead��ʾ��Ҫ����һ���ֽ��ж�ȡ����bit�����ɣ�bitsToRead ÿ���������� 3
  uint8_t bitsToShift;
  uint8_t startByte=21;
  uint8_t channelId=15;
  do
  {
    result[channelId]=raw[startByte];
    if(bitsToRead<=8)
    {
      result[channelId]<<=bitsToRead;
      bitsToShift=8-bitsToRead;
      result[channelId]+=(raw[startByte-1]>>bitsToShift);
    }
    else
    {
      result[channelId]<<=8;
      result[channelId]+=raw[startByte-1];
      startByte--;
      bitsToRead-=8;
      result[channelId]<<=bitsToRead;
      bitsToShift=8-bitsToRead;
      result[channelId]+=(raw[startByte-1]>>bitsToShift);
    }
    result[channelId]&=0x7FF;
    channelId--;
    startByte--;
    bitsToRead+=3;
  }while(startByte>0);
}


uint8_t Sbus_Receive_Flag=0;
/***************************************************
������: bool SBUS_Linear_Calibration(void)
˵��:	SBUS�������Ի�����
���:	��
����:	��
��ע:	��
ע���ߣ�����С��
****************************************************/
bool SBUS_Linear_Calibration(void)
{
  for(uint16_t i=0;i<=25;i++)
  {
    if(SBUS_Ringbuf.Ring_Buff[i]==0x0f&&SBUS_Ringbuf.Ring_Buff[i+24]==0x00)
    {
      Process((uint8_t *)(&SBUS_Ringbuf.Ring_Buff[i+1]),SBUS_Channel);
      for(uint8_t i = 0; i < 16; i++)//linear calibration
      {     
        SBUS_Channel_User[i] = (uint16_t)(SBUS_Channel[i] * _sbusScale + _sbusBias);
        //SBUS_Channel_User[i] = (uint16_t)(1000*SBUS_Channel[i]/2048)+1000;
      }    
      memcpy(PPM_Databuf,SBUS_Channel_User,10*sizeof(uint16));//��SBUS������ϵ����ݿ�����PPM������  
      Sbus_Receive_Flag=1;
      return TRUE;
    }
  }
  Sbus_Receive_Flag=0;
  return FALSE;
}



