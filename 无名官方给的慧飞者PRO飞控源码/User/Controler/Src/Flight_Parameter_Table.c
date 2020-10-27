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
#include "Flight_Parameter_Table.h"

volatile FLASH_Status Parameter_Table_FLASHStatus = FLASH_COMPLETE;      //Flash操作状态变量

void ReadFlashParameterALL(FLIGHT_PARAMETER *WriteData)
{
  uint16_t i=0;
  uint32_t ReadAddress = (uint32_t)PARAMETER_TABLE_STARTADDR;
  for(i=0;i<FLIGHT_PARAMETER_TABLE_NUM;i++)
  {
    WriteData->Parameter_Table[i]=*(float *)(ReadAddress+4*i);
  }
  FLASH_LockBank1();
}


uint8_t ReadFlashParameterOne(uint16_t Label,float *ReadData)
{
  uint16_t i=0;
  uint8_t buf[4];
  float temp_data=0;
  uint32_t ReadAddress = (uint32_t)PARAMETER_TABLE_STARTADDR+Label*4;
  temp_data=*(float *)(ReadAddress);
  FLASH_LockBank1();
  for(i=0;i<4;i++)//单字节数据
  {
    *(buf+i)=*(__IO uint8_t*) ReadAddress++;
  }
  if((buf[0]==0xff&&buf[1]==0xff&&buf[2]==0xff&&buf[3]==0xff))
    return 0;
  else
  {
    *ReadData=temp_data;
    return 1;
  }
}

uint8_t ReadFlashParameterTwo(uint16_t Label,float *ReadData1,float *ReadData2)
{
  uint16_t i=0;
  uint8_t buf[8];
  float temp_data1=0,temp_data2=0;
  uint32_t ReadAddress = (uint32_t)PARAMETER_TABLE_STARTADDR+Label*4;
  temp_data1=*(float *)(ReadAddress);
  ReadAddress+=4;
  temp_data2=*(float *)(ReadAddress);
  
  FLASH_LockBank1();
  for(i=0;i<8;i++)//单字节数据
  {
    *(buf+i)=*(__IO uint8_t*) ReadAddress++;
  }
  if((buf[0]==0xff&&buf[1]==0xff&&buf[2]==0xff&&buf[3]==0xff)
     &&(buf[4]==0xff&&buf[5]==0xff&&buf[6]==0xff&&buf[7]==0xff))
    return 0;
  else
  {
    *ReadData1=temp_data1;
    *ReadData2=temp_data2;
    return 1;
  }
}

uint8_t ReadFlashParameterThree(uint16_t Label,float *ReadData1,float *ReadData2,float *ReadData3)
{
  uint16_t i=0;
  uint8_t buf[12];
  float temp_data1=0,temp_data2=0,temp_data3=0;
  uint32_t ReadAddress = (uint32_t)PARAMETER_TABLE_STARTADDR+Label*4;
  temp_data1=*(float *)(ReadAddress);
  ReadAddress+=4;
  temp_data2=*(float *)(ReadAddress);
  ReadAddress+=4;
  temp_data3=*(float *)(ReadAddress);
  
  FLASH_LockBank1();
  for(i=0;i<12;i++)//单字节数据
  {
    *(buf+i)=*(__IO uint8_t*) ReadAddress++;
  }
  if((buf[0]==0xff&&buf[1]==0xff&&buf[2]==0xff&&buf[3]==0xff)
     &&(buf[4]==0xff&&buf[5]==0xff&&buf[6]==0xff&&buf[7]==0xff)
       &&(buf[8]==0xff&&buf[9]==0xff&&buf[10]==0xff&&buf[11]==0xff))
    return 0;
  else
  {
    *ReadData1=temp_data1;
    *ReadData2=temp_data2;
    *ReadData3=temp_data3;
    return 1;
  }
}


FLIGHT_PARAMETER Table_Parameter;
void WriteFlashParameter(uint16_t Label,
                         float WriteData,
                         FLIGHT_PARAMETER *Table)
{
  uint16_t i=0;
  ReadFlashParameterALL(Table);//先把片区内的所有数据都都出来
  Table->Parameter_Table[Label]=WriteData;//将需要更改的字段赋新值
  FLASH_UnlockBank1();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
  Parameter_Table_FLASHStatus = FLASH_ErasePage(PARAMETER_TABLE_STARTADDR);
  if(Parameter_Table_FLASHStatus == FLASH_COMPLETE)
  {
    for(i=0;i<FLIGHT_PARAMETER_TABLE_NUM;i++)
    {
      Parameter_Table_FLASHStatus = FLASH_ProgramWord(PARAMETER_TABLE_STARTADDR + 4*i,*(uint32_t *)(&Table->Parameter_Table[i]));
    }
  }
  FLASH_LockBank1();
}


void WriteFlashParameter_Two(uint16_t Label,
                             float WriteData1,
                             float WriteData2,
                             FLIGHT_PARAMETER *Table)
{
  uint16_t i=0;
  ReadFlashParameterALL(Table);//先把片区内的所有数据都都出来
  Table->Parameter_Table[Label]=WriteData1;//将需要更改的字段赋新值
  Table->Parameter_Table[Label+1]=WriteData2;//将需要更改的字段赋新值
  FLASH_UnlockBank1();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
  Parameter_Table_FLASHStatus = FLASH_ErasePage(PARAMETER_TABLE_STARTADDR);
  if(Parameter_Table_FLASHStatus == FLASH_COMPLETE)
  {
    for(i=0;i<FLIGHT_PARAMETER_TABLE_NUM;i++)
    {
      Parameter_Table_FLASHStatus = FLASH_ProgramWord(PARAMETER_TABLE_STARTADDR + 4*i,*(uint32_t *)(&Table->Parameter_Table[i]));
    }
  }
  FLASH_LockBank1();
}

void WriteFlashParameter_Three(uint16_t Label,
                               float WriteData1,
                               float WriteData2,
                               float WriteData3,
                               FLIGHT_PARAMETER *Table)
{
  uint16_t i=0;
  ReadFlashParameterALL(Table);//先把片区内的所有数据都都出来
  Table->Parameter_Table[Label]=WriteData1;//将需要更改的字段赋新值
  Table->Parameter_Table[Label+1]=WriteData2;//将需要更改的字段赋新值
  Table->Parameter_Table[Label+2]=WriteData3;//将需要更改的字段赋新值
  FLASH_UnlockBank1();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
  Parameter_Table_FLASHStatus = FLASH_ErasePage(PARAMETER_TABLE_STARTADDR);
  if(Parameter_Table_FLASHStatus == FLASH_COMPLETE)
  {
    for(i=0;i<FLIGHT_PARAMETER_TABLE_NUM;i++)
    {
      Parameter_Table_FLASHStatus = FLASH_ProgramWord(PARAMETER_TABLE_STARTADDR + 4*i,*(uint32_t *)(&Table->Parameter_Table[i]));
    }
  }
  FLASH_LockBank1();
}

