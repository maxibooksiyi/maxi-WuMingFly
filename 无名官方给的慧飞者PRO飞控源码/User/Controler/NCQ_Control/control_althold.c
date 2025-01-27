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
#include "control_config.h"
#include "control_althold.h"


Vector3f UAV_Cushion_Stop_Point;

float Yaw_Vel_Feedforward_Output=0;//竖直速度前馈控制器输出;
float Yaw_Vel_Feedforward_Rate=1.0;//竖直速度前馈控制器，APM里面为1、0.45;
float Yaw_Vel_Feedforward_Delta=0;//竖直期望速度变化率;
float Last_Yaw_Vel_Target=0;
float Yaw_Vel_Target=0;


uint16_t  Deadband=400;//油门中位死区

/*************以下为默认值，校准遥控器后会自动赋值**********************/
uint16_t  Deadzone_Min=400;
uint16_t  Deadzone_Max=600;
uint16_t  Thr_Top=1000;//油门最大上行程
uint16_t  Thr_Buttom=0;//油门最大下行程
uint16_t  Thr_Middle=500;//油门最大下行程
uint8_t Thr_Push_Over_State=0;
uint8_t Thr_Push_Over_Deadband(void)
{
  static uint8_t thr_now=0,thr_last=0;
  static uint8_t push_flag=0;
  thr_last=thr_now;
  if(RC_NewData[0]<=Deadzone_Min) thr_now=0;
  else if(RC_NewData[0]<=Deadzone_Max) thr_now=1;
  else thr_now=2;  
  if(thr_now==0&&thr_last==1)       return push_flag=0;//向下推过中位死区
  else if(thr_now==1&&thr_last==0)  return push_flag=1;//向上推到中位死区
  else if(thr_now==2&&thr_last==1)  return push_flag=2;//向上推过中位死区
  else if(thr_now==1&&thr_last==2)  return push_flag=3;//向下推到中位死区
  else push_flag=0;
  return push_flag;
}

void Thr_Scale_Set(Vector_RC *rc_date)
{
  Thr_Buttom=0;
  Thr_Top=(uint16_t)(rc_date[2].max-rc_date[2].min);
  Thr_Middle=(uint16_t)((rc_date[2].max-rc_date[2].min)/2);
  //Deadband=rc_date[2].deadband;//油门中位死区单独设置
  Deadzone_Min=Thr_Middle-Deadband/2;
  Deadzone_Max=Thr_Middle+Deadband/2;
}

void ncq_control_althold()
{
  static uint16 High_Pos_Control_Cnt=0;//高度位置控制计数器
  static uint16 High_Vel_Control_Cnt=0;//高度速度控制计数器
  static uint8_t move_flag=0;
  float step_mapping=0,scale_mapping=0;
  Thr_Scale_Set(&RC_Calibration[2]);
  High_Hold_Throttle=Thr_Hover_Default;
  
  /*高度控制器第1步*/
  /********
      **
      **
      **
      **
      **
   ********/
  /*******************************高度控制器开始****************************************/
  /****************定高：高度位置环+速度环+加速度环，控制周期分别为8ms、4ms、4ms*******************/
  if(RC_NewData[0]>=Deadzone_Min
     &&RC_NewData[0]<=Deadzone_Max)
  {
    //高度位置环输出给定速度期望
    if(Total_Controller.High_Position_Control.Expect==0)//油门回中后，只设置一次
    {
      Total_Controller.High_Position_Control.Expect=NamelessQuad.Position[_YAW];//油门回中后，更新高度期望
    }
    High_Pos_Control_Cnt++;
    if(High_Pos_Control_Cnt>=2)//竖直高度控制周期2*5=10ms
    {
      High_Pos_Control_Cnt=0;
      Total_Controller.High_Position_Control.FeedBack=NamelessQuad.Position[_YAW];//反馈
      PID_Control(&Total_Controller.High_Position_Control);//海拔高度位置控制器
      //内环速度期望
#ifdef YAW_VEL_FEEDFORWARD  //速度前馈控制器
      Yaw_Vel_Target=Total_Controller.High_Position_Control.Control_OutPut;//本次速度期望
      Yaw_Vel_Feedforward_Delta=(Yaw_Vel_Target-Last_Yaw_Vel_Target)/(2*Delta);//速度期望变化率
      Yaw_Vel_Feedforward_Output=Yaw_Vel_Feedforward_Rate*Yaw_Vel_Feedforward_Delta;
      Last_Yaw_Vel_Target=Yaw_Vel_Target;//上次速度期望
#endif
      Total_Controller.High_Speed_Control.Expect=Total_Controller.High_Position_Control.Control_OutPut;
    }
    move_flag=0;
  }
  else if(RC_NewData[0]>Deadzone_Max)//给定上升速度期望
  {
    //油门杆上推、给定速度期望
    step_mapping=(float)(RC_NewData[0]-Deadzone_Max)/(Thr_Top-Deadzone_Max);//范围0~1
    scale_mapping=constrain_float(step_mapping*step_mapping,0,1);
    Total_Controller.High_Speed_Control.Expect=Climb_Up_Speed_Max*scale_mapping;//最大上升速度50cm/s
    Total_Controller.High_Position_Control.Expect=0;//位置环期望置0
    move_flag=1;
  }
  else if(RC_NewData[0]<Deadzone_Min)//给定下降速度期望
  {
    //油门杆下推、给定速度期望
    step_mapping=(float)(RC_NewData[0]-Deadzone_Min)/(Deadzone_Min-Thr_Buttom);//范围0~-1
    scale_mapping=constrain_float(step_mapping*step_mapping,0,1);
    Total_Controller.High_Speed_Control.Expect=-Climb_Down_Speed_Max*scale_mapping;//最大下降速度40cm/s
    Total_Controller.High_Position_Control.Expect=0;//位置环期望置0
    move_flag=2;
  }
  
  /*高度控制器第2步*/
  /********
          *
          *
    * * * *
  *
  *
  ********/
  
  /*******************************竖直速度控制器开始*******************************************************************/
  High_Vel_Control_Cnt++;
  if(High_Vel_Control_Cnt>=1)//竖直高度控制周期5*1=5ms
  {
    High_Vel_Control_Cnt=0;
    Total_Controller.High_Speed_Control.FeedBack=NamelessQuad.Speed[_YAW];//惯导速度估计给速度反馈
    PID_Control_Div_LPF(&Total_Controller.High_Speed_Control);//海拔高度速度控制
  }
  /*******************************上升下降过程中期望加速度限幅单独处理*******************************************************************/     
  if(move_flag==1)//在上下推杆时对速度控制器输出，对期望上升、下降加速度进行分别限幅，确保快速下降时姿态平稳
  {
    Total_Controller.High_Speed_Control.Control_OutPut=constrain_float(Total_Controller.High_Speed_Control.Control_OutPut,
                                                                       -Climb_Up_Acceleration_Max,Climb_Up_Acceleration_Max);                                                                
  }
  else if(move_flag==2)
  {
    Total_Controller.High_Speed_Control.Control_OutPut=constrain_float(Total_Controller.High_Speed_Control.Control_OutPut,
                                                                       -Climb_Down_Acceleration_Max,Climb_Down_Acceleration_Max);
  }  
  /*******************************竖直速度控制器结束******************************************************************/
  
  /*高度控制器第3步*/
  /********
         **
         **
      **
         **
         **
  ********/
  
  /*******************************竖直加速度控制器开始******************************************************************/
  Total_Controller.High_Acce_Control.Expect=Total_Controller.High_Speed_Control.Control_OutPut;//加速度期望
#ifdef YAW_VEL_FEEDFORWARD  //速度前馈控制器
  Total_Controller.High_Acce_Control.Expect+=Yaw_Vel_Feedforward_Output;//加上前馈控制器输出
#endif
  Total_Controller.High_Acce_Control.FeedBack=Filter_Feedback_NamelessQuad.Acceleration[_YAW];//加速度反馈
  PID_Control_Err_LPF(&Total_Controller.High_Acce_Control);//海拔高度加速度控制
  /**************************************
  加速度环前馈补偿，引用时请注明出处
  悬停油门 = 加速度环积分值 + 基准悬停油门
  此时输出力 F = mg
  当需要输出a的加速度时，输出力 F1=m(g+a)
  F1/F = 1 + a/g
  因此此时应输出：悬停油门*(1 + a/g)
  **************************************/
  Total_Controller.High_Acce_Control.Control_OutPut+=(High_Hold_Throttle+Total_Controller.High_Acce_Control.Integrate-Thr_Start)*
    (Total_Controller.High_Acce_Control.Expect/980);
  /*******************************竖直加速度控制器结束******************************************************************/
  if(YAW_Pos_Control_Accel_Disable==0)
  {
    Throttle=Int_Sort(High_Hold_Throttle+Total_Controller.High_Acce_Control.Control_OutPut);//油门来源于高度加速度控制器输出
  }
  else
  {
    Throttle=Int_Sort(High_Hold_Throttle+Total_Controller.High_Speed_Control.Control_OutPut);//油门来源于高度速度控制器输出
  }
  /*****************************************高度控制器结束，给定油门控制量***********************************************************/
}




