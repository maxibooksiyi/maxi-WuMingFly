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
#ifndef _CONTROL_CONFIG_H
#define _CONTROL_CONFIG_H

#include "control_althold.h"
#include "control_poshold.h"
#include "control_land.h"


#define Self_Balance_Mode 1//自稳、纯姿态加油门补偿
#define High_Hold_Mode    2//定高模式
#define Pos_Free_Mode 1//水平位置no fixed
#define Pos_Hold_Mode 2//定点模式
//GPS定点下打杆控速模式与直接姿态角
#define Speed_Mode 0  //GPS定点模式下，打杆控速
#define Angle_Mode 1  //GPS定点模式下，打杆直接给姿态期望角

/*****************遥控器行程设置**********************/
#define  Climb_Up_Speed_Max    300//向上最大攀爬速度，cm/s 150 250  400
#define  Climb_Down_Speed_Max  150//向下最大下降速度，cm/s  80 120  180

/****************************************************************
当油门推重比较小时，若上升下降期望加速度比较大，
会导致输出映射的里面的姿态控制量得不到充分输出，
使得快速上升下降时，姿态不平稳，若此过程持续时间长，
会导致姿态长期得不到修正，直致最后炸鸡，故推重比较
小时，可将期望加速度限小一点，或者做控制量优先级处理
*******************************************************************/
#define  Climb_Up_Acceleration_Max     300//向上最大攀爬加速度，cm/s^2   500
#define  Climb_Down_Acceleration_Max   150//向下最大下降加速度，cm/s^2   300


#define  Thr_Start  1100//起转油门量，油门倾角补偿用2d，太大会导致过补偿
#define  Thr_Fly_Start  1150//起飞油门量 1150
#define  Thr_Hover_Default 1580//默认悬停油门，直接定高起飞时用1500
#define  Thr_Min 1000
#define  Thr_Idle 1100//油门怠速，取接近起转油门值即可1150
#define  Nav_Speed_Max  500//最大期望水平速度为5m/s


/*****************一键起飞高度设置，单位为cm，比如100表示原地起飞到相对初始位置1米高的位置**********************/
#define  Auto_Launch_Target 100//一键起飞的目标高度，相对起飞高度，使用超声波时，请勿超过超过声波量程，推荐200以下



#endif

