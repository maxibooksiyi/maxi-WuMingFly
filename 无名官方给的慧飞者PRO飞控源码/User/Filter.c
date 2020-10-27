#include "Headfile.h"
#include "Filter.h"
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
//-----Butterworth变量 Matlab fdatool设计-----//
//设计方法见博客：http://blog.csdn.net/u011992534/article/details/73743955
/*
Butter_Parameter Butter_80HZ_Parameter_Acce={
//200hz---80hz
1,     1.14298050254,   0.4128015980962,
0.638945525159,    1.277891050318,    0.638945525159
};

Butter_Parameter Butter_60HZ_Parameter_Acce={
//200hz---60hz
1,   0.3695273773512,   0.1958157126558,
0.3913357725018,   0.7826715450035,   0.3913357725018
};

Butter_Parameter Butter_51HZ_Parameter_Acce={
//200hz---51hz
1,  0.03680751639284,   0.1718123812701,
0.3021549744157,   0.6043099488315,   0.3021549744157,
};

Butter_Parameter Butter_30HZ_Parameter_Acce={
//200hz---30hz
1,  -0.7477891782585,    0.272214937925,
0.1311064399166,   0.2622128798333,   0.1311064399166
};
Butter_Parameter Butter_20HZ_Parameter_Acce={
//200hz---20hz
1,    -1.14298050254,   0.4128015980962,
0.06745527388907,   0.1349105477781,  0.06745527388907
};
Butter_Parameter Butter_15HZ_Parameter_Acce={
//200hz---15hz
1,   -1.348967745253,   0.5139818942197,
0.04125353724172,  0.08250707448344,  0.04125353724172
};

Butter_Parameter Butter_10HZ_Parameter_Acce={
//200hz---10hz
1,   -1.561018075801,   0.6413515380576,
0.02008336556421,  0.04016673112842,  0.02008336556421};
Butter_Parameter Butter_5HZ_Parameter_Acce={
//200hz---5hz
1,   -1.778631777825,   0.8008026466657,
0.005542717210281,  0.01108543442056, 0.005542717210281
};

Butter_Parameter Butter_2HZ_Parameter_Acce={
//200hz---2hz
1,   -1.911197067426,   0.9149758348014,
0.0009446918438402,  0.00188938368768,0.0009446918438402
};
*/
int  Ce[10];
//-----Butterworth变量-----//
Butter_Parameter Butter_80HZ_Parameter_Acce,Butter_60HZ_Parameter_Acce,Butter_51HZ_Parameter_Acce,
Butter_30HZ_Parameter_Acce,Butter_20HZ_Parameter_Acce,Butter_15HZ_Parameter_Acce,
Butter_10HZ_Parameter_Acce,Butter_5HZ_Parameter_Acce,Butter_2HZ_Parameter_Acce;
Butter_BufferData Butter_Buffer[3];
Butter_BufferData Butter_Buffer_Feedback[3];
Butter_BufferData Butter_Buffer_SINS[3];

/****************************************
Butterworth低通滤波器参数初始化：http://blog.csdn.net/u011992534/article/details/73743955
***************************************/
void Butterworth_Parameter_Init(void)
{
  Set_Cutoff_Frequency(Sampling_Freq, 80,&Butter_80HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 60,&Butter_60HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 51,&Butter_51HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 30,&Butter_30HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 20,&Butter_20HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 15,&Butter_15HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 10,&Butter_10HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 5 ,&Butter_5HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 2 ,&Butter_2HZ_Parameter_Acce);
  pascalTriangle(10,1,Ce);
}

/*************************************************
函数名:	float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
说明:	加速度计低通滤波器
入口:	float curr_input 当前输入加速度计,滤波器参数，滤波器缓存
出口:	无
备注:	2阶Butterworth低通滤波器
*************************************************/
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  /* 加速度计Butterworth滤波 */
  /* 获取最新x(n) */
  static int LPF_Cnt=0;
  Buffer->Input_Butter[2]=curr_input;
  if(LPF_Cnt>=100)
  {
    /* Butterworth滤波 */
    Buffer->Output_Butter[2]=
      Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
          +Parameter->b[2] * Buffer->Input_Butter[0]
            -Parameter->a[1] * Buffer->Output_Butter[1]
              -Parameter->a[2] * Buffer->Output_Butter[0];
  }
  else
  {
    Buffer->Output_Butter[2]=Buffer->Input_Butter[2];
    LPF_Cnt++;
  }
  /* x(n) 序列保存 */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) 序列保存 */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
  return Buffer->Output_Butter[2];
}

Butter_Parameter Notch_Filter_Parameter_55hz={
  //200hz---55hz-5hz  采样-陷波频率-带宽
  1,   0.3008218748187,   0.9229897627825,
  0.9614948813912,   0.3008218748187,   0.9614948813912
};
Butter_Parameter Notch_Filter_Parameter_60hz={
  //200hz---60hz-5hz  采样-陷波频率-带宽
  1,   0.5942365167088,   0.9229897627825,
  0.9614948813912,   0.5942365167088,   0.9614948813912
};
Butter_Parameter Notch_Filter_Parameter_65hz={
  //200hz---65hz-5hz  采样-陷波频率-带宽
  1,   0.8730190833996,   0.9229897627825,
  0.9614948813912,   0.8730190833996,   0.9614948813912
};

Butter_Parameter Notch_Filter_Parameter_75hz={
  //200hz---75hz-50hz  采样-陷波频率-带宽
  1,   0.9372808715784,   0.3255153203391,
  0.6627576601696,   0.9372808715784,   0.6627576601696
};
Notch_Filter_BufferData  Notch_Filter_Buffer;
/************************************************/
float Notch_Filter(float curr_input,Notch_Filter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  float temp=0;
  /* 获取最新x(n) */ 
  Buffer->Output_Butter[0]= Parameter->a[0]*curr_input
    -Parameter->a[1]*Buffer->Output_Butter[1]
      -Parameter->a[2]*Buffer->Output_Butter[2];
  /* 计算陷波滤波器输出 */
  temp=Parameter->b[0]*Buffer->Output_Butter[0]
    +Parameter->b[1]*Buffer->Output_Butter[1]
      +Parameter->b[2]*Buffer->Output_Butter[2];
  /* y(n) 序列保存 */
  Buffer->Output_Butter[2]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[0]; 
  return temp;
}


Butter_Parameter Bandstop_Filter_Parameter_30_98={
  //200hz---30hz-98hz  采样-阻带
  1,   0.6270403082828,  -0.2905268567319,
  0.354736571634,   0.6270403082828,    0.354736571634
};
Butter_Parameter Bandstop_Filter_Parameter_30_94={
  //200hz---30hz-94hz  采样-阻带
  1,   0.5334540355829,  -0.2235264828971,
  0.3882367585514,   0.5334540355829,   0.3882367585514
};
Butter_BufferData  Bandstop_Filter_Buffer[2];
float BPF_Butterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  /* 加速度计Butterworth滤波 */
  /* 获取最新x(n) */
  Buffer->Input_Butter[2]=curr_input;
  /* Butterworth滤波 */
  Buffer->Output_Butter[2]=
    Parameter->b[0] * Buffer->Input_Butter[2]
      +Parameter->b[1] * Buffer->Input_Butter[1]
        +Parameter->b[2] * Buffer->Input_Butter[0]
          -Parameter->a[1] * Buffer->Output_Butter[1]
            -Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) 序列保存 */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) 序列保存 */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
  return Buffer->Output_Butter[2];
}



float Acce_Control[3]={0};
float Acce_Control_Feedback[3]={0};
float Acce_SINS[3]={0};
void Acce_Control_Filter()
{
  /**********************惯导加速度LPF_60hz**************************/
  Vector3f Body_Frame,Earth_Frame;
  
  Acce_SINS[0]=LPButterworth(X_Origion,
                             &Butter_Buffer_SINS[0],&Butter_30HZ_Parameter_Acce);
  Acce_SINS[1]=LPButterworth(Y_Origion,
                             &Butter_Buffer_SINS[1],&Butter_30HZ_Parameter_Acce);
  Acce_SINS[2]=LPButterworth(Z_Origion,
                             &Butter_Buffer_SINS[2],&Butter_30HZ_Parameter_Acce);//用作惯导融合的加速度计量
  
  Acce_Control[0]=LPButterworth(X_Origion,
                                &Butter_Buffer[0],&Butter_51HZ_Parameter_Acce);
  Acce_Control[1]=LPButterworth(Y_Origion
                                ,&Butter_Buffer[1],&Butter_51HZ_Parameter_Acce);
  Acce_Control[2]=LPButterworth(Z_Origion
                                ,&Butter_Buffer[2],&Butter_51HZ_Parameter_Acce);
  
  Body_Frame.x=Acce_Control[0];
  Body_Frame.y=Acce_Control[1];
  Body_Frame.z=Acce_Control[2];
  Vector_From_BodyFrame2EarthFrame(&Body_Frame,&Earth_Frame);
  FilterAfter_NamelessQuad.Acceleration[_YAW]=Earth_Frame.z;
  FilterAfter_NamelessQuad.Acceleration[_PITCH]=Earth_Frame.x;
  FilterAfter_NamelessQuad.Acceleration[_ROLL]=Earth_Frame.y;
  FilterAfter_NamelessQuad.Acceleration[_YAW]*=AcceGravity/AcceMax;
  FilterAfter_NamelessQuad.Acceleration[_YAW]-=AcceGravity;
  FilterAfter_NamelessQuad.Acceleration[_YAW]*=100;//加速度cm/s^2
  FilterAfter_NamelessQuad.Acceleration[_PITCH]*=AcceGravity/AcceMax;
  FilterAfter_NamelessQuad.Acceleration[_PITCH]*=100;//加速度cm/s^2
  FilterAfter_NamelessQuad.Acceleration[_ROLL]*=AcceGravity/AcceMax;
  FilterAfter_NamelessQuad.Acceleration[_ROLL]*=100;//加速度cm/s^2
  
  /**********************加速度反馈量LPF_5hz*****************************************************************************/
  Acce_Control_Feedback[0]=LPButterworth(X_Origion,
                                         &Butter_Buffer_Feedback[0],&Butter_15HZ_Parameter_Acce);//用作加速度反馈的加速度计量
  Acce_Control_Feedback[1]=LPButterworth(Y_Origion
                                         ,&Butter_Buffer_Feedback[1],&Butter_15HZ_Parameter_Acce);
  Acce_Control_Feedback[2]=LPButterworth(Z_Origion
                                         ,&Butter_Buffer_Feedback[2],&Butter_15HZ_Parameter_Acce);
  /******************如果定高时，发现飞机有时莫名抽、抖，可以把加速度反馈滤波截止频率降低，APM飞控用的Fc=2Hz******************/
  
  Body_Frame.x=Acce_Control_Feedback[0];
  Body_Frame.y=Acce_Control_Feedback[1];
  Body_Frame.z=Acce_Control_Feedback[2];
  Vector_From_BodyFrame2EarthFrame(&Body_Frame,&Earth_Frame);
  Filter_Feedback_NamelessQuad.Acceleration[_YAW]=Earth_Frame.z;
  Filter_Feedback_NamelessQuad.Acceleration[_PITCH]=Earth_Frame.x;
  Filter_Feedback_NamelessQuad.Acceleration[_ROLL]=Earth_Frame.y;
  Filter_Feedback_NamelessQuad.Acceleration[_YAW]*=AcceGravity/AcceMax;
  Filter_Feedback_NamelessQuad.Acceleration[_YAW]-=AcceGravity;
  Filter_Feedback_NamelessQuad.Acceleration[_YAW]*=100;//加速度cm/s^2
  Filter_Feedback_NamelessQuad.Acceleration[_PITCH]*=AcceGravity/AcceMax;
  Filter_Feedback_NamelessQuad.Acceleration[_PITCH]*=100;//加速度cm/s^2
  Filter_Feedback_NamelessQuad.Acceleration[_ROLL]*=AcceGravity/AcceMax;
  Filter_Feedback_NamelessQuad.Acceleration[_ROLL]*=100;//加速度cm/s^2
}

// discrete low pass filter, cuts out the
// high frequency noise that can drive the controller crazy
//derivative = _last_derivative + _d_lpf_alpha * (derivative - _last_derivative);
float set_lpf_alpha(int16_t cutoff_frequency, float time_step)
{
  // calculate alpha
  float lpf_alpha;
  float rc = 1/(2*PI*cutoff_frequency);
  lpf_alpha = time_step / (time_step + rc);
  return lpf_alpha;
}
//https://blog.csdn.net/sszhouplus/article/details/43113505
//https://blog.csdn.net/shengzhadon/article/details/46784509
//https://blog.csdn.net/shengzhadon/article/details/46791903
//https://blog.csdn.net/shengzhadon/article/details/46803401
/************************************************************************/
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
  float fr = sample_frequent / cutoff_frequent;
  float ohm = tanf(M_PI_F / fr);
  float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
  if (cutoff_frequent <= 0.0f) {
    // no filtering
    return;
  }
  LPF->b[0] = ohm * ohm / c;
  LPF->b[1] = 2.0f * LPF->b[0];
  LPF->b[2] = LPF->b[0];
  LPF->a[0]=1.0f;
  LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
  LPF->a[2] = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}

float Filter_data[4];
void Test_Filter()
{
  static uint32_t Filter_Cnt=0;
  static float Ts=0.005f;
  Filter_Cnt++;
  Filter_data[0]=100*sin(2*PI*30*Filter_Cnt*Ts)
    +100*sin(2*PI*90*Filter_Cnt*Ts)
      //+100*sin(2*PI*150*Filter_Cnt*Ts)
      +100*sin(2*PI*75*Filter_Cnt*Ts);//多种频率混合信号
  Filter_data[1]=Notch_Filter(Filter_data[0],&Notch_Filter_Buffer,&Notch_Filter_Parameter_75hz);
  Filter_data[2]=BPF_Butterworth(Filter_data[0],&Bandstop_Filter_Buffer[0],&Bandstop_Filter_Parameter_30_98);
  Filter_data[3]=BPF_Butterworth(Filter_data[0],&Bandstop_Filter_Buffer[1],&Butter_51HZ_Parameter_Acce);
}



#define SYMBOL_ADD  0
#define SYMBOL_SUB  1
/*======================================================================
 * 函数名：  pascalTriangle
 * 函数功能：计算杨辉三角的第N行的值（数组），该系列值为(x+1)^N的系数，
 *         加改进(x-1)^N的系数，最低次数在第一个
 *
 * 变量名称：
 *          N      - 杨辉三角第N行，N=0,1,...,N
 *          symbol - 运算符号，0——(x+1)^N，1——(x-1)^N
 *          vector - 返回数组，杨辉三角的第N行的值
 *
 * 返回值：  void
 *=====================================================================*/

void pascalTriangle(int N,int symbol,int *vector)
{
    vector[0] = 1;
    if(N == 0)
    {
        return;
    }
    else if (N == 1)
    {
        if(symbol == SYMBOL_ADD)
        {
            vector[1] = 1;
        }
        else
        {
            vector[0] = -1; //如果是减号，则第二项系数是-1
            vector[1] = 1;
        }
        return;
    }
    int length = N + 1; //数组长度
    int temp[11];   //定义中间变量
    
    temp[0] = 1;
    temp[1] = 1;
    
    for(int i = 2; i <= N; i++)
    {
        vector[i] = 1;
        for(int j = 1; j < i; j++)
        {
            vector[j] = temp[j - 1] + temp[j]; //x[m][n] = x[m-1][n-1] + x[m-1][n]
        }
        if(i == N) //最后一次不需要给中间变量赋值
        {
            if(symbol == SYMBOL_SUB) //运算符为减号
            {
                for(int k = 0; k < length; k++)
                {
                    vector[k] = vector[k] * pow(-1, length - 1 - k);
                }
            }
            return;
        }
        for(int j = 1; j <= i; j++)
        {
            temp[j] = vector[j];
        }
    }
}
