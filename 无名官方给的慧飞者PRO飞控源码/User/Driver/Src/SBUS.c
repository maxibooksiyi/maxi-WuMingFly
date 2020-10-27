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
#include "SBUS.h"

const float _sbusScale = 0.625749f; //Matlab拟合出的量程因子
const float _sbusBias =  875.23353f;//Matlab拟合出的零偏
uint16_t SBUS_Channel[16]={0};
uint16_t SBUS_Channel_User[16]={0};
/***************************************************
函数名: void Process(uint8_t *raw,uint16_t *result)
说明:	SBUS数据解析
入口:	原始数据、解析数据
出口:	无
备注:	无
注释者：无名小哥
****************************************************/
void Process(uint8_t *raw,uint16_t *result)
{
  uint8_t bitsToRead=3; // bitsToRead表示需要从下一个字节中读取多少bit。规律：bitsToRead 每次总是增加 3
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
函数名: bool SBUS_Linear_Calibration(void)
说明:	SBUS数据线性化处理
入口:	无
出口:	无
备注:	无
注释者：无名小哥
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
      memcpy(PPM_Databuf,SBUS_Channel_User,10*sizeof(uint16));//将SBUS解析完毕的数据拷贝到PPM数据区  
      Sbus_Receive_Flag=1;
      return TRUE;
    }
  }
  Sbus_Receive_Flag=0;
  return FALSE;
}



