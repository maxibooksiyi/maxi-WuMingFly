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
Sensor_Okay_Flag Sensor_Init_Flag;
/***************************************************
函数名: void HardWave_Init(void)
说明:	板载硬件资源初始化
入口:	无
出口:	无
备注:	飞控内部资源、外设初始化
注释者：无名小哥
****************************************************/
void HardWave_Init(void)
{
  SystemInit();        //系统始终初始化
  delay_init(72);      //滴答延时初始化
  USART1_Init(115200); //主串口初始化
  USB_Config();        //USB虚拟串口初始化
	OLED_Init();         //显示屏初始化
  Bling_Init();        //指示灯、测试IO初始化
  GetLockCode();       //正版校验
  Key_Init();          //按键初始化
  PPM_Init();          //PPM遥控器接收初始化
  HC_SR04_Init();      //超声波初始化
  PWM_Init();          //PWM初始化—TIM3、TIM4
  delay_ms(1000);
  /***********此部分为自制遥控器时使用，群文件有遥控器DIY方案***************
  SPI2_Configuration();
  NRF24L01_Init();
  while(NRF24L01_Check())
  {
  printf("24L01 Check Failed!\r\n");
  printf("Please Check!\r\n");
  delay_ms(100);
  }
  NRF24L01_RX_Mode();
  ***********************************************************************/
  Sensor_Init_Flag.NRF24L01_Okay=1;
  QuadInit();
  /*******************IMU初始化开始*************************/
  /***********MPU6050初始化***************/
  IIC_GPIO_Config();           //软件模拟I2C初始化
  InitMPU6050_GY86();          //MPU6050初始化、唤醒、设置采样频率、量程等
  delay_ms(500);
  IMU_Calibration();           //陀螺仪零偏标定
  Sensor_Init_Flag.MPU6050_Okay=1;
  /***********HMC5883初始化***************/
  delay_ms(100);
  QuadInit();
  /***********磁力计+气压计初始化***************/
  delay_ms(500);
  HMC5883L_Initial();
  Sensor_Init_Flag.Mag_Okay=1;
  QuadInit();
#ifdef IMU_BOARD_GY86           //GY86模块磁力计为HMC5883L
  Baro_init();
  Read_MS5611_Offset();
  Sensor_Init_Flag.Baro_Okay=1;
#endif
#ifdef IMU_BOARD_NC686          //NC686模块磁力计为IST8310、气压计为SPL01_001
  IST8310_Init();
  Sensor_Init_Flag.Mag_Okay=1;
  QuadInit();
  spl0601_init();
  Sensor_Init_Flag.Baro_Okay=1;
  QuadInit();
#endif
#ifdef IMU_BOARD_NC683          //NC686模块磁力计为IST8310、气压计为FBM320
  IST8310_Init();
  Sensor_Init_Flag.Mag_Okay=1;
  QuadInit();
  FBM320_Init();
  Sensor_Init_Flag.Baro_Okay=1;
  QuadInit();
#endif  
  /*******************IMU初始化结束*************************/
  Quad_Start_Bling();          //LED开机预显示
  delay_ms(500);
  Mag_LS_Init();	       //磁力计最小二乘法球面拟合初始化
  Parameter_Init();            //控制器参数初始化
  NCQ_Quad_Init();             //初始四元数初始化
  Butterworth_Parameter_Init();//滤波器参数初始化
  RC_Calibration_Trigger();    //遥控器标定参数初始化
  Horizontal_Calibration_Init();//水平标定参数初始化
  LCD_CLS();                   //清屏
  Set_GPS_USART();             //上电轮询自动配置GPS
  USART3_Init(115200);         //串口3、副串口连接匿名地面站
  OpticalFlow_Init();          //LC306光流传感器初始化
  USART4_Init(19200);          //串口4、解析光流数据
  SBUS_USART5_Init();          //串口5、SBUS解析
  PID_Paramter_Init_With_Flash();//PID控制器初始化，可以通过地面站修改参数//Total_PID_Init();PID控制器初始化，只能通过程序修改参数
  ADRC_Init(&ADRC_Pitch_Controller,&ADRC_Roll_Controller);//自抗扰控制器初始化
  Chip_ADC_Init();             //芯片AD检测初始化
  SDK_Init();
  TIM2_Configuration_Cnt();    //TIM2程序计时定时器
  Timer1_Configuration();      //TIM4任务调度定时器
  NVIC_Configuration();        //中断优先级设置
}


/***************************************************
函数名: void NVIC_Configuration(void)
说明:	中断优先级定义
入口:	无
出口:	无
备注:	系统中断时间在整体设计时，已全部测试过最大开销，
        确保程序运动的稳健性        
注释者：无名小哥
****************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;//定义NVIC初始化结构体
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//优先级组别2，具体参见misc.h line80
  //GPS数据接收中断
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口中断2
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   
  
  //超声波
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
  //飞控系统定时器
  NVIC_InitStructure.NVIC_IRQChannel =TIM2_IRQn ;//计数定时器
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
	//SBUS解析串口 
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn; //中断号；
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级；
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //响应优先级；
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);    
	  
  //PPM接收机
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
  //副串口
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
   
  //备用串口光流使用 
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn; //中断号；
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级；
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //响应优先级；
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
 
  
  //-----NRF24L01数据中断-----//
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//IRQ中断通道-->NRF24L01,PB12
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//抢先式优先级别
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//副优先级别
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能通道
  NVIC_Init(&NVIC_InitStructure);//初始化NVIC
  
  //飞控任务调度定时器
  NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



/***************************************************
函数名: void NCQ_Init(void)
说明:	系统初始化
入口:	无
出口:	无
备注:	首先读取特定片区标志位，决策进入
        电调校准模式还是正常初始化        
注释者：无名小哥
****************************************************/
void NCQ_Init(void)
{
  ReadFlashParameterOne(ESC_CALIBRATION_FLAG,&ESC_Calibration_Flag);
  if(ESC_Calibration_Flag==1)
  {
    ESC_HardWave_Init();//只初始化校准电调的必要资源 
  }
  else
  {
    HardWave_Init();//飞控板内部资源、相关外设初始化
  }
}

