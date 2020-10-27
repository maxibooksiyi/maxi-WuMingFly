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
Sensor_Okay_Flag Sensor_Init_Flag;
/***************************************************
������: void HardWave_Init(void)
˵��:	����Ӳ����Դ��ʼ��
���:	��
����:	��
��ע:	�ɿ��ڲ���Դ�������ʼ��
ע���ߣ�����С��
****************************************************/
void HardWave_Init(void)
{
  SystemInit();        //ϵͳʼ�ճ�ʼ��
  delay_init(72);      //�δ���ʱ��ʼ��
  USART1_Init(115200); //�����ڳ�ʼ��
  USB_Config();        //USB���⴮�ڳ�ʼ��
	OLED_Init();         //��ʾ����ʼ��
  Bling_Init();        //ָʾ�ơ�����IO��ʼ��
  GetLockCode();       //����У��
  Key_Init();          //������ʼ��
  PPM_Init();          //PPMң�������ճ�ʼ��
  HC_SR04_Init();      //��������ʼ��
  PWM_Init();          //PWM��ʼ����TIM3��TIM4
  delay_ms(1000);
  /***********�˲���Ϊ����ң����ʱʹ�ã�Ⱥ�ļ���ң����DIY����***************
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
  /*******************IMU��ʼ����ʼ*************************/
  /***********MPU6050��ʼ��***************/
  IIC_GPIO_Config();           //���ģ��I2C��ʼ��
  InitMPU6050_GY86();          //MPU6050��ʼ�������ѡ����ò���Ƶ�ʡ����̵�
  delay_ms(500);
  IMU_Calibration();           //��������ƫ�궨
  Sensor_Init_Flag.MPU6050_Okay=1;
  /***********HMC5883��ʼ��***************/
  delay_ms(100);
  QuadInit();
  /***********������+��ѹ�Ƴ�ʼ��***************/
  delay_ms(500);
  HMC5883L_Initial();
  Sensor_Init_Flag.Mag_Okay=1;
  QuadInit();
#ifdef IMU_BOARD_GY86           //GY86ģ�������ΪHMC5883L
  Baro_init();
  Read_MS5611_Offset();
  Sensor_Init_Flag.Baro_Okay=1;
#endif
#ifdef IMU_BOARD_NC686          //NC686ģ�������ΪIST8310����ѹ��ΪSPL01_001
  IST8310_Init();
  Sensor_Init_Flag.Mag_Okay=1;
  QuadInit();
  spl0601_init();
  Sensor_Init_Flag.Baro_Okay=1;
  QuadInit();
#endif
#ifdef IMU_BOARD_NC683          //NC686ģ�������ΪIST8310����ѹ��ΪFBM320
  IST8310_Init();
  Sensor_Init_Flag.Mag_Okay=1;
  QuadInit();
  FBM320_Init();
  Sensor_Init_Flag.Baro_Okay=1;
  QuadInit();
#endif  
  /*******************IMU��ʼ������*************************/
  Quad_Start_Bling();          //LED����Ԥ��ʾ
  delay_ms(500);
  Mag_LS_Init();	       //��������С���˷�������ϳ�ʼ��
  Parameter_Init();            //������������ʼ��
  NCQ_Quad_Init();             //��ʼ��Ԫ����ʼ��
  Butterworth_Parameter_Init();//�˲���������ʼ��
  RC_Calibration_Trigger();    //ң�����궨������ʼ��
  Horizontal_Calibration_Init();//ˮƽ�궨������ʼ��
  LCD_CLS();                   //����
  Set_GPS_USART();             //�ϵ���ѯ�Զ�����GPS
  USART3_Init(115200);         //����3��������������������վ
  OpticalFlow_Init();          //LC306������������ʼ��
  USART4_Init(19200);          //����4��������������
  SBUS_USART5_Init();          //����5��SBUS����
  PID_Paramter_Init_With_Flash();//PID��������ʼ��������ͨ������վ�޸Ĳ���//Total_PID_Init();PID��������ʼ����ֻ��ͨ�������޸Ĳ���
  ADRC_Init(&ADRC_Pitch_Controller,&ADRC_Roll_Controller);//�Կ��ſ�������ʼ��
  Chip_ADC_Init();             //оƬAD����ʼ��
  SDK_Init();
  TIM2_Configuration_Cnt();    //TIM2�����ʱ��ʱ��
  Timer1_Configuration();      //TIM4������ȶ�ʱ��
  NVIC_Configuration();        //�ж����ȼ�����
}


/***************************************************
������: void NVIC_Configuration(void)
˵��:	�ж����ȼ�����
���:	��
����:	��
��ע:	ϵͳ�ж�ʱ�����������ʱ����ȫ�����Թ��������
        ȷ�������˶����Ƚ���        
ע���ߣ�����С��
****************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;//����NVIC��ʼ���ṹ��
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//���ȼ����2������μ�misc.h line80
  //GPS���ݽ����ж�
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//�����ж�2
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);   
  
  //������
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
  //�ɿ�ϵͳ��ʱ��
  NVIC_InitStructure.NVIC_IRQChannel =TIM2_IRQn ;//������ʱ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
	//SBUS�������� 
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn; //�жϺţ�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ���
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //��Ӧ���ȼ���
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);    
	  
  //PPM���ջ�
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
  //������
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
   
  //���ô��ڹ���ʹ�� 
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn; //�жϺţ�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //��ռ���ȼ���
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //��Ӧ���ȼ���
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
 
  
  //-----NRF24L01�����ж�-----//
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//IRQ�ж�ͨ��-->NRF24L01,PB12
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//����ʽ���ȼ���
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//�����ȼ���
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ��ͨ��
  NVIC_Init(&NVIC_InitStructure);//��ʼ��NVIC
  
  //�ɿ�������ȶ�ʱ��
  NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
  NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



/***************************************************
������: void NCQ_Init(void)
˵��:	ϵͳ��ʼ��
���:	��
����:	��
��ע:	���ȶ�ȡ�ض�Ƭ����־λ�����߽���
        ���У׼ģʽ����������ʼ��        
ע���ߣ�����С��
****************************************************/
void NCQ_Init(void)
{
  ReadFlashParameterOne(ESC_CALIBRATION_FLAG,&ESC_Calibration_Flag);
  if(ESC_Calibration_Flag==1)
  {
    ESC_HardWave_Init();//ֻ��ʼ��У׼����ı�Ҫ��Դ 
  }
  else
  {
    HardWave_Init();//�ɿذ��ڲ���Դ����������ʼ��
  }
}

