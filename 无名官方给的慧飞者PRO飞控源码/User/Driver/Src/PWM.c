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
#include "PWM.h"


#define  MAX_PWM 2500 //400hz    周期2.5ms
//#define  MAX_PWM 5000  //200hz  周期20ms
//#define  MAX_PWM 10000 //100hz  周期20ms
//#define  MAX_PWM 20000 //50hz  周期20ms

/***************************************************
函数名: void PWM_GPIO_Init(void)
说明:	PWM输出IO初始化
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void PWM_GPIO_Init(void )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //开始TIM4 的时钟 及GPIOB时钟 和AFIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//定时器3作为PWM输出
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO ,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  
  //设置PA5、PA6、PB0、PB1 为推挽输出
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;//
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//输出速率
  GPIO_Init(GPIOA,&GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;//
  GPIO_Init(GPIOB,&GPIO_InitStructure);
  
  
  /*************TIM4******************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;//只用到两路，因为PB6、PB7已作为I2C口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //GPIO_PinRemapConfig(GPIO_Remap_TIM4 , ENABLE);//引脚复用映像  PD12 PD13 PD14 PD15
}


void PWM_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  uint16_t prescalerValue = 0, ccr1_PWMVal = 0;
  PWM_GPIO_Init();
  prescalerValue = 72-1;//10us
  //-----TIM3定时配置-----//
  TIM_TimeBaseStructure.TIM_Period = MAX_PWM;		//40000/2M=20ms-->50Hz，从0开始计数,这个值被写入到Auto-Reload Register中
  TIM_TimeBaseStructure.TIM_Prescaler = 0;	       //暂时不分频
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;	       //时钟分割
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;	//重复比较次数更新事件
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_PrescalerConfig(TIM3, prescalerValue, TIM_PSCReloadMode_Immediate);//预分频,现在计时器频率为20MHz
  
  //-----PWM配置-----//
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 		//选择定时器模式:TIM脉冲宽度调制模式1-->向上计数为有效电平
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
  TIM_OCInitStructure.TIM_Pulse = ccr1_PWMVal;					//duty cycle
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 	//输出极性:TIM输出比较极性高
  
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);  		//初始化外设TIM3 OC1-->Motor1
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);  		//初始化外设TIM3 OC2-->Motor2
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);  		//初始化外设TIM3 OC3-->Motor3
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);  		//初始化外设TIM3 OC4-->Motor4
  
  TIM_ARRPreloadConfig(TIM3, ENABLE);//自动重载寄存器使能，下一个更新事件自动更新影子寄存器
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_Cmd(TIM3, ENABLE);
  
  
  /*************TIM4**********************/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//配置时钟
  TIM_TimeBaseStructure.TIM_Period=MAX_PWM;
  TIM_TimeBaseStructure.TIM_Prescaler=71; //72  1us
  TIM_TimeBaseStructure.TIM_ClockDivision=0;
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM1;//PWM1模式1
  TIM_OCInitStructure.TIM_OutputState =TIM_OutputState_Enable;//比较输出使能
  TIM_OCInitStructure.TIM_Pulse = ccr1_PWMVal; //设置占空比
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出比较极性高
  
  //TIM_OC1Init(TIM4, &TIM_OCInitStructure);//初始化TIM4的CH1通道
  //TIM_OC2Init(TIM4, &TIM_OCInitStructure);//初始化TIM4的CH2通道
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);//初始化TIM4的CH3通道
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);//初始化TIM4的CH4通道
  
  //TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  //TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM4,ENABLE);//使能TIM4在CH3通道ARR3上的预装载寄存器
  TIM_Cmd(TIM4,ENABLE);//使能定时器4
  //PWM_Set(1000,1000,1000,1000,1000,1000,1000,1000);
}





/***********************************************************************************
函数名：void PWM_Set(const u16 pwm1, const u16 pwm2, const u16 pwm3, const u16 pwm4 ,const uint16_t pwm5, const uint16_t pwm6)
说明：PWM输出设置
入口：四个通道的值
出口：无
备注：满占空为2.5ms（20ms）
************************************************************************************/
void PWM_Set(const uint16_t pwm1, const uint16_t pwm2,
             const uint16_t pwm3, const uint16_t pwm4,
             const uint16_t pwm5, const uint16_t pwm6)
{
  TIM_SetCompare4(TIM3, pwm1);
  TIM_SetCompare3(TIM3, pwm2);
  TIM_SetCompare2(TIM3, pwm3);
  TIM_SetCompare1(TIM3, pwm4);
  
  TIM_SetCompare3(TIM4, pwm5);
  TIM_SetCompare4(TIM4, pwm6);
}





