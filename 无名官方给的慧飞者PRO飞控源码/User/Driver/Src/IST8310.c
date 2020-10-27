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
#include "IST8310.h"
#include "HMC5883.h"


#define MAG_REVERSE_SIDE

uint8_t ts[4]={0};
void IST8310_Init(void)
{
#ifdef MAG_REVERSE_SIDE//重新映射磁力计三轴数据		
  Single_WriteI2C_Adjust(IST8310_SLAVE_ADDRESS,0x41,0x24);//开启16x内部平均
  Single_WriteI2C_Adjust(IST8310_SLAVE_ADDRESS,0x42,0xC0);//Set/Reset内部平均
#else	
	Single_WriteI2C1(IST8310_SLAVE_ADDRESS,0x41,0x24);//开启16x内部平均
	Single_WriteI2C1(IST8310_SLAVE_ADDRESS,0x42,0xC0);//Set/Reset内部平均
#endif	
}



IST8310 Mag_IST8310;
//磁力机航向角计算与补偿：https://blog.csdn.net/u013636775/article/details/72675148
void Get_Mag_IST8310(void)
{
  static uint16_t IST8310_Sample_Cnt=0;
  float MagTemp[3]={0};
  IST8310_Sample_Cnt++;
  if(IST8310_Sample_Cnt==1)
  {
#ifdef MAG_REVERSE_SIDE//重新映射磁力计三轴数据		
    Single_WriteI2C_Adjust(IST8310_SLAVE_ADDRESS,IST8310_REG_CNTRL1,0x01);//Single Measurement Mode
#else		
		Single_WriteI2C1(IST8310_SLAVE_ADDRESS,IST8310_REG_CNTRL1,0x01);//Single Measurement Mode
#endif		
  }
  else if(IST8310_Sample_Cnt==4)//至少间隔6ms,此处为8ms
  {
#ifdef MAG_REVERSE_SIDE//重新映射磁力计三轴数据
    Mag_IST8310.Buf[0]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x03);//OUT_X_L_A
    Mag_IST8310.Buf[1]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x04);//OUT_X_H_A
    Mag_IST8310.Buf[2]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x05);//OUT_Y_L_A
    Mag_IST8310.Buf[3]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x06);//OUT_Y_H_A
    Mag_IST8310.Buf[4]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x07);//OUT_Z_L_A
    Mag_IST8310.Buf[5]=Single_ReadI2C_Adjust(IST8310_SLAVE_ADDRESS,0x08);//OUT_Z_H_A
#else	
    Mag_IST8310.Buf[0]=Single_ReadI2C1(IST8310_SLAVE_ADDRESS,0x03);//OUT_X_L_A
    Mag_IST8310.Buf[1]=Single_ReadI2C1(IST8310_SLAVE_ADDRESS,0x04);//OUT_X_H_A
    Mag_IST8310.Buf[2]=Single_ReadI2C1(IST8310_SLAVE_ADDRESS,0x05);//OUT_Y_L_A
    Mag_IST8310.Buf[3]=Single_ReadI2C1(IST8310_SLAVE_ADDRESS,0x06);//OUT_Y_H_A
    Mag_IST8310.Buf[4]=Single_ReadI2C1(IST8310_SLAVE_ADDRESS,0x07);//OUT_Z_L_A
    Mag_IST8310.Buf[5]=Single_ReadI2C1(IST8310_SLAVE_ADDRESS,0x08);//OUT_Z_H_A		
#endif		
    /*****************合成三轴磁力计数据******************/
    Mag_IST8310.Mag_Data[0]=(Mag_IST8310.Buf[1]<<8)|Mag_IST8310.Buf[0];
    Mag_IST8310.Mag_Data[1]=(Mag_IST8310.Buf[3]<<8)|Mag_IST8310.Buf[2];
    Mag_IST8310.Mag_Data[2]=(Mag_IST8310.Buf[5]<<8)|Mag_IST8310.Buf[4];
    IST8310_Sample_Cnt=0;
  }
#ifdef MAG_REVERSE_SIDE//重新映射磁力计三轴数据
  Mag_IST8310.x = Mag_IST8310.Mag_Data[0];
  Mag_IST8310.y = -Mag_IST8310.Mag_Data[1];
  Mag_IST8310.z = Mag_IST8310.Mag_Data[2];
#else
  Mag_IST8310.x = Mag_IST8310.Mag_Data[0];
  Mag_IST8310.y = -Mag_IST8310.Mag_Data[1];
  Mag_IST8310.z = Mag_IST8310.Mag_Data[2];
#endif
  DataMag.x=Compass.x=Mag_IST8310.x;
  DataMag.y=Compass.y=Mag_IST8310.y;
  DataMag.z=Compass.z=Mag_IST8310.z;
  MagTemp[0]=GildeAverageValueFilter_MAG(Mag_IST8310.x-Mag_Offset[0],Data_X_MAG);//滑动窗口滤波
  MagTemp[1]=GildeAverageValueFilter_MAG(Mag_IST8310.y-Mag_Offset[1],Data_Y_MAG);
  MagTemp[2]=GildeAverageValueFilter_MAG(Mag_IST8310.z-Mag_Offset[2],Data_Z_MAG);
  
  
  
  Mag_Data[0]=Mag_IST8310.Mag_Data_Correct[0]=MagTemp[0];
  Mag_Data[1]=Mag_IST8310.Mag_Data_Correct[1]=MagTemp[1];
  Mag_Data[2]=Mag_IST8310.Mag_Data_Correct[2]=MagTemp[2];
  
  /************磁力计倾角补偿*****************/
  MagN.x=Mag_IST8310.thx = MagTemp[0] * Cos_Roll+ MagTemp[2] * Sin_Roll;
  MagN.y=Mag_IST8310.thy = MagTemp[0] * Sin_Pitch*Sin_Roll
                          +MagTemp[1] * Cos_Pitch
                          -MagTemp[2] * Cos_Roll*Sin_Pitch;
  /***********反正切得到磁力计观测角度*********/
  Mag_IST8310.Angle_Mag=atan2(Mag_IST8310.thx,Mag_IST8310.thy)*57.296;
}


float Earth_Magnetic_Field_Intensity=0;
void Compass_Tradeoff(void)
{
  if(Extern_Mag_Work_Flag==1)//只要初始化时，检测到外部磁力计HMC5883/5983，就用外部磁力计融合，注意外部磁力计轴向
  {
    HMC5883L_StateMachine();
    Earth_Magnetic_Field_Intensity=sqrt(Mag_Data[0]*Mag_Data[0]
                                        +Mag_Data[1]*Mag_Data[1]
                                          +Mag_Data[2]*Mag_Data[2])/MAG_GAIN_SCALE7;
  }
  else//使用内部磁力计IST8310
  { 
    Get_Mag_IST8310();
    Earth_Magnetic_Field_Intensity=sqrt(Mag_Data[0]*Mag_Data[0]
                                        +Mag_Data[1]*Mag_Data[1]
                                          +Mag_Data[2]*Mag_Data[2])/330;
  }
  
}


