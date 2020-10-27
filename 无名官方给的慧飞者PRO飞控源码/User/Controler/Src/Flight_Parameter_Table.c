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
#include "Flight_Parameter_Table.h"

volatile FLASH_Status Parameter_Table_FLASHStatus = FLASH_COMPLETE;      //Flash����״̬����

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
  for(i=0;i<4;i++)//���ֽ�����
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
  for(i=0;i<8;i++)//���ֽ�����
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
  for(i=0;i<12;i++)//���ֽ�����
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
  ReadFlashParameterALL(Table);//�Ȱ�Ƭ���ڵ��������ݶ�������
  Table->Parameter_Table[Label]=WriteData;//����Ҫ���ĵ��ֶθ���ֵ
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
  ReadFlashParameterALL(Table);//�Ȱ�Ƭ���ڵ��������ݶ�������
  Table->Parameter_Table[Label]=WriteData1;//����Ҫ���ĵ��ֶθ���ֵ
  Table->Parameter_Table[Label+1]=WriteData2;//����Ҫ���ĵ��ֶθ���ֵ
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
  ReadFlashParameterALL(Table);//�Ȱ�Ƭ���ڵ��������ݶ�������
  Table->Parameter_Table[Label]=WriteData1;//����Ҫ���ĵ��ֶθ���ֵ
  Table->Parameter_Table[Label+1]=WriteData2;//����Ҫ���ĵ��ֶθ���ֵ
  Table->Parameter_Table[Label+2]=WriteData3;//����Ҫ���ĵ��ֶθ���ֵ
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

