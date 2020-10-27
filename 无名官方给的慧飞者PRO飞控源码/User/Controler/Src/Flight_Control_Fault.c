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
#include "Flight_Control_Fault.h"

Flight_Control_Fault_State  NCQ_Fault;


#define DIVIDER_RES_MAX  100//��ѹ����1
#define DIVIDER_RES_MIN  10//��ѹ����2
#define DIVIDER_RES_SCALE (DIVIDER_RES_MAX+DIVIDER_RES_MIN)/DIVIDER_RES_MIN

float Battery_Valtage=0;
void Battery_Fault_Check(void)
{
  static uint16_t battery_fault_cnt=0;
  Battery_Valtage=(DIVIDER_RES_SCALE*ADC_Value[1]*3.3f)/4096;
  if(Battery_Valtage<11.2)
  {
    battery_fault_cnt++;
    if(battery_fault_cnt>=100)//����100*5ms=500ms
    {
      NCQ_Fault.Low_Voltage_Fault_Flag=TRUE;
    }
  }
  else
  {
    battery_fault_cnt/=2;
    if(battery_fault_cnt==0)  NCQ_Fault.Low_Voltage_Fault_Flag=FALSE; 
  }
}

void Baro_Fault_Check(float baropress)
{
  static uint16_t baro_fault_cnt=0;
  static float last_baropress=0;
  static uint16_t baro_gap_cnt=0;
  baro_gap_cnt++;
  if(baro_gap_cnt>=40)//ÿ200ms���һ�Σ���Ϊ��ѹ�Ƹ������ڴ���5ms
  {
    baro_gap_cnt=0;
    if(last_baropress==baropress)
    {
      baro_fault_cnt++;
      if(baro_fault_cnt>5)  NCQ_Fault.Baro_Fault_Flag=TRUE;   
    }
    else
    {
      baro_fault_cnt/=2;
      if(baro_fault_cnt==0)  NCQ_Fault.Baro_Fault_Flag=FALSE; 
    }
    last_baropress=baropress; 
  }
}

void Accel_Fault_Check(Vector3i accel)
{
  static uint16_t accel_fault_cnt=0;
  static Vector3i last_accel={0};
  if(last_accel.x==accel.x
     &&last_accel.y==accel.y
       &&last_accel.z==accel.z)
  {
    accel_fault_cnt++;
    if(accel_fault_cnt>20)  NCQ_Fault.Accel_Fault_Flag=TRUE;   
  }
  else
  {
    accel_fault_cnt/=2;
    if(accel_fault_cnt==0)  NCQ_Fault.Accel_Fault_Flag=FALSE; 
  }
  last_accel=accel; 
}


void Gyro_Fault_Check(Vector3i gyro)
{
  static uint16_t gyro_fault_cnt=0;
  static Vector3i last_gyro={0};
  if(last_gyro.x==gyro.x
     &&last_gyro.y==gyro.y
       &&last_gyro.z==gyro.z)
  {
    gyro_fault_cnt++;
    if(gyro_fault_cnt>20)  NCQ_Fault.Gyro_Fault_Flag=TRUE;   
  }
  else
  {
    gyro_fault_cnt/=2;
    if(gyro_fault_cnt==0)  NCQ_Fault.Gyro_Fault_Flag=FALSE; 
  }
  last_gyro=gyro; 
}


void Compass_Fault_Check(Vector3i compass)
{
  static uint16_t compass_fault_cnt=0;
  static Vector3i last_compass={0};
  static uint16_t compass_gap_cnt=0;
  compass_gap_cnt++;
  if(compass_gap_cnt>=40)//ÿ200ms���һ�Σ���Ϊ�����Ƹ������ڴ���5ms
  {
    compass_gap_cnt=0;
    if(last_compass.x==compass.x
       &&last_compass.y==compass.y
         &&last_compass.z==compass.z)
    {
      compass_fault_cnt++;
      if(compass_fault_cnt>10)  NCQ_Fault.Compass_Fault_Flag=TRUE;   
    }
    else
    {
      compass_fault_cnt/=2;
      if(compass_fault_cnt==0)  NCQ_Fault.Compass_Fault_Flag=FALSE; 
    }
    last_compass=compass;
  }
}



void Ultrasonic_Fault_Check(float ultrasonic)
{
  static uint16_t ultrasonic_fault_cnt=0;
  static float last_ultrasonic;
  if(last_ultrasonic==ultrasonic)
  {
    ultrasonic_fault_cnt++;
    if(ultrasonic_fault_cnt>20)  NCQ_Fault.Ultrasonic_Fault_Flag=TRUE;   
  }
  else
  {
    ultrasonic_fault_cnt/=2;
    if(ultrasonic_fault_cnt==0)  NCQ_Fault.Ultrasonic_Fault_Flag=FALSE; 
  }
  last_ultrasonic=ultrasonic; 
}



void Opticalflow_Fault_Check(int16_t opticalflow_x,int16_t opticalflow_y)
{
  static uint16_t opticalflow_fault_cnt=0;
  static uint16_t opticalflow_fault_zero_cnt=0; 
  static int16_t last_opticalflow_x,last_opticalflow_y;
  if(last_opticalflow_x==opticalflow_x
     &&last_opticalflow_y==opticalflow_y
       &&last_opticalflow_x!=0
         &&last_opticalflow_y!=0)
  {
    opticalflow_fault_cnt++;
    if(opticalflow_fault_cnt>20)  NCQ_Fault.Opticalflow_Fault_Flag=TRUE;   
  }
  else if(last_opticalflow_x==opticalflow_x
          &&last_opticalflow_y==opticalflow_y
            &&last_opticalflow_x==0
              &&last_opticalflow_y==0)
  {
    opticalflow_fault_zero_cnt++;
    if(opticalflow_fault_zero_cnt>100)  NCQ_Fault.Opticalflow_Fault_Flag=TRUE;   
  }
  else
  {
    opticalflow_fault_cnt/=2;
    opticalflow_fault_zero_cnt/=2;
    if(opticalflow_fault_cnt==0)  NCQ_Fault.Opticalflow_Fault_Flag=FALSE; 
  }
  last_opticalflow_x=opticalflow_x;
  last_opticalflow_y=opticalflow_y;
}


Vector3i Accel,Gyro,Compass;
void Flight_Control_Fault_ALL(void)
{
  Battery_Fault_Check();
  Baro_Fault_Check(Altitude_Estimate);
  Accel_Fault_Check(Accel);
  Gyro_Fault_Check(Gyro);
  Compass_Fault_Check(Compass);
}


