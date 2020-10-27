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
#include "stm32f10x_flash.h"//flash�����ӿ��ļ����ڿ��ļ��У�������Ҫ����
#include "FLASH.h"


union {
  float Bit32;
  unsigned char Bit8[4];
}flash;

/****************************************************************
*Function:	STM32F103ϵ���ڲ�Flash��д����
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:		�ó�����ֱ�ӱ������У�ֻ������Flash��д����
****************************************************************/
//#define  STARTADDR  0x08010000                   	 //STM32F103RB �����ͺŻ������ã�δ����
#define  STARTADDR  0x0803A000//0x080350A8   2K=2048=0x800
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;      //Flash����״̬����
/****************************************************************
*Name:		ReadFlashNBtye
*Function:	���ڲ�Flash��ȡN�ֽ�����
*Input:		ReadAddress�����ݵ�ַ��ƫ�Ƶ�ַ��ReadBuf������ָ��	ReadNum����ȡ�ֽ���
*Output:	��ȡ���ֽ���
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:
****************************************************************/
int ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadNum)
{
  int DataNum = 0;
  ReadAddress = (uint32_t)STARTADDR + ReadAddress;
  while(DataNum < ReadNum)
  {
    *(ReadBuf + DataNum) = *(__IO uint8_t*) ReadAddress++;
    DataNum++;
  }
  return DataNum;
}


uint8_t ReadFlashOneWord(uint32_t ReadAddress,uint32_t *ReadData)
{
  uint16_t i=0;
  uint8_t buf[4];
  uint32_t temp_data=0;
  temp_data=*(uint32_t *)(ReadAddress);
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


/****************************************************************
*Name:		WriteFlashOneWord
*Function:	���ڲ�Flashд��32λ����
*Input:		WriteAddress�����ݵ�ַ��ƫ�Ƶ�ַ��WriteData��д������
*Output:	NULL
*Author:    ValerianFan
*Date:		2014/04/09
*E-Mail:	fanwenjingnihao@163.com
*Other:
****************************************************************/
void WriteFlashOneWord(uint32_t WriteAddress,uint32_t WriteData)
{
  FLASH_UnlockBank1();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
  FLASHStatus = FLASH_ErasePage(WriteAddress);
  if(FLASHStatus == FLASH_COMPLETE)
  {
    FLASHStatus = FLASH_ProgramWord(WriteAddress, WriteData);    //flash.c ��API����
  }
  FLASH_LockBank1();
}



void WriteFlashHarfWord(uint32_t WriteAddress,uint16_t WriteData)
{
  FLASH_UnlockBank1();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
  FLASHStatus = FLASH_ErasePage(STARTADDR);
  if(FLASHStatus == FLASH_COMPLETE)
  {
    FLASHStatus = FLASH_ProgramHalfWord(STARTADDR + WriteAddress, WriteData);    //flash.c ��API����
  }
  FLASH_LockBank1();
}

void WriteFlashNineFloat(uint32_t WriteAddress,
                         float WriteData1,
                         float WriteData2,
                         float WriteData3,
                         float WriteData4,
                         float WriteData5,
                         float WriteData6,
                         float WriteData7,
                         float WriteData8,
                         float WriteData9)
{
  uint32_t Buf[9]={0};
  Buf[0]=*(uint32_t *)(&WriteData1);//���ڴ��������ĸ��ֽ�д�뵽Flash
  Buf[1]=*(uint32_t *)(&WriteData2);
  Buf[2]=*(uint32_t *)(&WriteData3);
  Buf[3]=*(uint32_t *)(&WriteData4);
  Buf[4]=*(uint32_t *)(&WriteData5);
  Buf[5]=*(uint32_t *)(&WriteData6);
  Buf[6]=*(uint32_t *)(&WriteData7);
  Buf[7]=*(uint32_t *)(&WriteData8);
  Buf[8]=*(uint32_t *)(&WriteData9);
  
  FLASH_UnlockBank1();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
  FLASHStatus = FLASH_ErasePage(STARTADDR);
  if(FLASHStatus == FLASH_COMPLETE)
  {
    FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress,Buf[0]);
    FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+4,Buf[1]);
    FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+8,Buf[2]);
    FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+12,Buf[3]);
    FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+16,Buf[4]);
    FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+20,Buf[5]);
    FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+24,Buf[6]);
    FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+28,Buf[7]);
    FLASHStatus = FLASH_ProgramWord(STARTADDR + WriteAddress+32,Buf[8]);
    
  }
  FLASH_LockBank1();
}

uint8_t ReadFlashThreeFloat(uint32_t ReadAddress,
                            float *WriteData1,
                            float *WriteData2,
                            float *WriteData3)
{
  uint8_t buf[12];
  uint16_t i=0;
  uint8_t flag=0x00;
  ReadAddress = (uint32_t)STARTADDR + ReadAddress;
  *WriteData1=*(float *)(ReadAddress);
  *WriteData2=*(float *)(ReadAddress+4);
  *WriteData3=*(float *)(ReadAddress+8);
  FLASH_LockBank1();
  
  for(i=0;i<12;i++)//���ֽ�����
  {
    *(buf+i)=*(__IO uint8_t*) ReadAddress++;
  }
  if((buf[0]==0xff&&buf[1]==0xff&&buf[2]==0xff&&buf[3]==0xff))
    flag=flag|0x01;
  if((buf[4]==0xff&&buf[5]==0xff&&buf[6]==0xff&&buf[7]==0xff))
    flag=flag|0x02;
  if((buf[8]==0xff&&buf[9]==0xff&&buf[10]==0xff&&buf[11]==0xff))
    flag=flag|0x04;
  return flag;
}


