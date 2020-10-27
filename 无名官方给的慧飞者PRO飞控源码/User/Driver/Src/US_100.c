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
#include "US_100.h"
float US_Distance=0;
float US_100_Distance(uint8 MSB,uint8 LSB)
{
  return (256*(MSB)+(LSB))/1.0;
}

float US_100_Temperature(uint8 data)
{
  return (data-45)/1.0;
}


Butter_BufferData Butter_Buffer_HCSR04;
Butter_Parameter Butter_2HZ_Parameter_HCSR04;
u32 Test_Cnt1[2]={0},Test_Cnt2[2]={0},Test_Delta=0;
uint8 HC_SR04_StartFlag=0;
float HC_SR04_Distance_Temp=0,HC_SR04_Distance=0,HC_SR04_Distance_Filter=0,HC_SR04_Bubble_Distance=0;
float HC_SR04_Distance_Record[5]={0};
float HC_SR04_Distance_Div[5]={0};
float HCSR04_Data[5];
uint16 HC_SR04_RSSI=1;
uint16 Sample_Cnt=0;
void HC_SR04_Start(void)
{
  float tempa,tempb,tempc,max,min;//用于防跳变滤波
  HC_SR04_RSSI--;
  if(HC_SR04_RSSI<=254&&HC_SR04_StartFlag==1)//通讯异常：中断抢占后超时、线路接触不良等
  {
    HC_SR04_StartFlag=0;
    Sample_Cnt=0;
    HC_SR04_OUT_LOW;//先强制拉低一段时间
    //EXTI->IMR &=~EXTI_Line1;//关闭外部中断
    EXTI_ClearITPendingBit(EXTI_Line1);
    Delay_Us(10);
  }
  else//正常接受
  {   
    if(Test_Delta<=Effective_Height_Time_Max)  //限幅约24000->400cm,15000->250cm，11765->200cm  5882->100
    {
      HC_SR04_Distance_Temp=Test_Delta*(340)/20000.0;
    }
    
    for(int16 i=5;i>0;i--)
    {
      HCSR04_Data[i]=HCSR04_Data[i-1];
    }
    HCSR04_Data[0]=HC_SR04_Distance_Temp;
    
    /******************************************/
    //均值滤波，保证得到数据不跳变，避免期望阶跃时，微分输出异常
    HC_SR04_Distance_Record[0]=HC_SR04_Distance_Temp;
    tempa=HC_SR04_Distance_Record[2];
    tempb=HC_SR04_Distance_Record[1];
    tempc=HC_SR04_Distance_Record[0];
    max = tempa > tempb ? tempa:tempb;
    max = max > tempc ? max:tempc;
    min = tempa < tempb ? tempa:tempb;
    min = min < tempc ? min:tempc;
    if(tempa > min && tempa < max)    HC_SR04_Distance_Record[0] = tempa;
    if(tempb > min  && tempb < max )  HC_SR04_Distance_Record[0] = tempb;
    if(tempc > min  &&  tempc < max)  HC_SR04_Distance_Record[0] = tempc;
    for(int16 i=5;i>0;i--)
    {
      HC_SR04_Distance_Record[i] = HC_SR04_Distance_Record[i-1];  
    }
       
    HC_SR04_Distance_Div[0]=HCSR04_Data[4]-HCSR04_Data[3];
    HC_SR04_Distance_Div[1]=HCSR04_Data[3]-HCSR04_Data[2];
    HC_SR04_Distance_Div[2]=HCSR04_Data[2]-HCSR04_Data[1];
    HC_SR04_Distance_Div[3]=HCSR04_Data[1]-HCSR04_Data[0];
    //if(ABS(HC_SR04_Distance_Div[0])<15
    //   &&ABS(HC_SR04_Distance_Div[1])<15
    //     &&ABS(HC_SR04_Distance_Div[2])<15
    //       &&ABS(HC_SR04_Distance_Div[3])<15
     //       &&ABS(HC_SR04_Distance_Div[0])<=1.0*ABS(NamelessQuad.Speed[_YAW])) 
    //{
        HC_SR04_Bubble_Distance=HC_SR04_Distance_Temp;
        HC_SR04_Distance=HC_SR04_Bubble_Distance;
    //}
    
    HC_SR04_Distance_Filter=LPButterworth(HC_SR04_Distance,&Butter_Buffer_HCSR04,&Butter_2HZ_Parameter_HCSR04);
    if(HC_SR04_Distance<=User_Height_Max&&HC_SR04_Distance>0)  Sensor_Flag.Hcsr04_Health=1;
    else  Sensor_Flag.Hcsr04_Health=0; 
  }
  
  if(HC_SR04_StartFlag==0)
  {
    HC_SR04_OUT_HIGH;
    Delay_Us(20);//不少于10us,软件延时时间约等于15us
    HC_SR04_OUT_LOW;
    HC_SR04_StartFlag=1;
    Sample_Cnt=0;
    HC_SR04_UP();
    HC_SR04_RSSI=255;
  }
}

void HC_SR04_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  //EXTI_InitTypeDef EXTI_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//下拉输入
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_SetBits(GPIOA,GPIO_Pin_0);//初始拉低
  delay_ms(10);//初始持续拉低一段时间
  HC_SR04_StartFlag=0;
  Sample_Cnt=0;
  
  Set_Cutoff_Frequency(10, 4 ,&Butter_2HZ_Parameter_HCSR04);
}

void HC_SR04_UP()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//下拉输入
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd= ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  EXTI->IMR |=EXTI_Line1;//关闭外部中断
  
}

void HC_SR04_DN()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd= ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}


uint8_t HCSR_04_Update_Flag=0;
void EXTI1_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {   
    if(Sample_Cnt==0)//先上升沿
    {
      //Test_Cnt1=10000*TIME_ISR_CNT+TIM2->CNT/2;
      Test_Cnt1[0]=TIME_ISR_CNT;//10ms
      Test_Cnt1[1]=TIM2->CNT;//us
      HC_SR04_DN();
      Sample_Cnt++;
    }
    else if(Sample_Cnt==1)//后下降沿
    {
      //Test_Cnt2=10000*TIME_ISR_CNT+TIM2->CNT/2;
      Test_Cnt2[0]=TIME_ISR_CNT;//10ms
      Test_Cnt2[1]=TIM2->CNT;//us
      HC_SR04_StartFlag=0;
      Test_Delta=10000*(Test_Cnt2[0]-Test_Cnt1[0])+(Test_Cnt2[1]-Test_Cnt1[1]);//us
      EXTI->IMR &=~EXTI_Line1;//关闭外部中断
      HCSR_04_Update_Flag=1;
    }
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
}


uint8 HC_SR04_Cnt=0;
void HC_SR04_Statemachine(void)
{
  HC_SR04_Cnt++;
  if(HC_SR04_Cnt>=20)//100ms
  {
    HC_SR04_Start();//超声波触发，接收在外部中断里面处理
    HC_SR04_Cnt=0;
  }
}

