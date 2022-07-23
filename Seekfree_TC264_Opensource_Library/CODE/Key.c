/*
 * Key.c
 *
 *  Created on: 2022年4月12日
 *      Author: yue
 */

#include "Key.h"
#include "zf_stm_systick.h"
#include "zf_gpio.h"
#include "ImageProcess.h"
#include "PID.h"            //修改PID参数
#include "LED.h"
#include "oled.h"           //OLED显示
#include "ImageSpecial.h"

extern float encoder_distance;

/*
 ** 函数功能: 初始化按键对应IO口
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void KeyInit(void)
{
    gpio_init(P33_10, GPI, 0, PULLDOWN);    //初始化按键
    gpio_init(P33_11, GPI, 0, PULLDOWN);
    gpio_init(P33_12, GPI, 0, PULLDOWN);
    gpio_init(P33_13, GPI, 0, PULLDOWN);
    gpio_init(P32_4, GPI, 0, PULLDOWN);
}

/*
 ** 函数功能: 按键扫描处理
 ** 参    数: 无
 ** 返 回 值: 对应按下按键的值，0=无按键按下，其他对应查看宏定义
 ** 作    者: WBN
 */
uint8 KeyScan(void)
{
    static uint8 key_up=1;     //按键松开标志
    if(key_up&&(KEY_S1==0||KEY_S2==0||KEY_S3==0||KEY_S4==0||KEY_S5==0))
    {
        systick_delay_ms(STM0,10);
        key_up=0;
        if(KEY_S1==0)       return KEY_UP;
        else if(KEY_S2==0)  return KEY_DOWN;
        else if(KEY_S3==0)  return KEY_LEFT;
        else if(KEY_S4==0)  return KEY_RIGHT;
        else if(KEY_S5==0)  return KEY_ENTER;
    }
    else if(KEY_S1==1||KEY_S2==1||KEY_S3==1||KEY_S4==1||KEY_S5==1)
        key_up=1;
    return 0;   //无按键按下
}

/*
 ** 函数功能: 按键PID调参的OLED参数显示
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void PIDParameterDisplay(uint8 key_num)
{
    OLED_clear();
    switch(key_num)
    {
        case 0: //舵机P
        {
            OLED_ShowStr(0, 1, "Steer.P:", 2);
            OLED_ShowFloat(0, 4, SteerK.P, 2);
            break;
        }
        case 1: //舵机D
        {
            OLED_ShowStr(0, 1, "Steer.D:", 2);
            OLED_ShowFloat(0, 4, SteerK.D, 2);
            break;
        }
        case 2: //左电机P
        {
            OLED_ShowStr(0, 1, "Motor_L.P:", 2);
            OLED_ShowFloat(0, 4, MotorK_L.P, 2);
            break;
        }
        case 3: //左电机I
        {
            OLED_ShowStr(0, 1, "Motor_L.I:", 2);
            OLED_ShowFloat(0, 4, MotorK_L.I, 2);
            break;
        }
        case 4: //右电机P
        {
            OLED_ShowStr(0, 1, "Motor_R.P:", 2);
            OLED_ShowFloat(0, 4, MotorK_R.P, 2);
            break;
        }
        case 5: //右电机I
        {
            OLED_ShowStr(0, 1, "Motor_R.I:", 2);
            OLED_ShowFloat(0, 4, MotorK_R.I, 2);
            break;
        }
    }
}

/*
 ** 函数功能: 按键PID调参
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void KeyPID(void)
{
    uint8 key_num=0;
    while(1)
    {
        switch(KeyScan())
        {
            case KEY_UP:    //增大参数值
            {
                switch(key_num)
                {
                    case 0:SteerK.P+=1;     break;
                    case 1:SteerK.D+=1;     break;
                    case 2:MotorK_L.P+=10;   break;
                    case 3:MotorK_L.I+=0.1;   break;
                    case 4:MotorK_R.P+=1;   break;
                    case 5:MotorK_R.I+=0.1;   break;
                }
                PIDParameterDisplay(key_num);
                break;
            }
            case KEY_DOWN:  //减小参数值
            {
                switch(key_num)
                {
                    case 0:SteerK.P-=1;     break;
                    case 1:SteerK.D-=1;     break;
                    case 2:MotorK_L.P-=10;   break;
                    case 3:MotorK_L.I-=0.1;   break;
                    case 4:MotorK_R.P-=10;   break;
                    case 5:MotorK_R.I-=0.1;   break;
                }
                PIDParameterDisplay(key_num);
                break;
            }
            case KEY_LEFT:  //向后切换参数
            {
                if(key_num>0)
                {
                    key_num--;
                }
                PIDParameterDisplay(key_num);
                break;
            }
            case KEY_RIGHT: //向前切换参数
            {
                if(key_num<5)
                {
                    key_num++;
                }
                PIDParameterDisplay(key_num);
                break;
            }
            case KEY_ENTER: //退出调参
            {
                OLED_clear();
                OLED_ShowFloat(0, 1, SteerK.P, 2);
                OLED_ShowFloat(0, 2, SteerK.D, 2);
                OLED_ShowFloat(0, 3, MotorK_L.P, 2);
                OLED_ShowFloat(0, 4, MotorK_L.I, 2);
                OLED_ShowFloat(0, 5, MotorK_R.P, 2);
                OLED_ShowFloat(0, 6, MotorK_R.I, 2);
                return;
            }
        }
        systick_delay_ms(STM0,100);
    }
}

/*
 ** 函数功能: 按键Process调参的OLED参数显示
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void ProcessParameterDisplay(uint8 key_num)
{
    OLED_clear();
    switch(key_num)
    {
        case 0: //状态机状态
        {
            OLED_ShowStr(0, 1, "process_flag:", 2);
            OLED_ShowNum(0, 4, process_flag, 1, 1);
            break;
        }
        case 1: //速度
        {
            OLED_ShowStr(0, 1, "base_speed:", 2);
            OLED_ShowNum(0, 4, base_speed, 3, 1);
            break;
        }
        case 2: //编码器测距
        {
            OLED_ShowStr(0, 1, "encoder_distance", 2);
            OLED_ShowFloat(0, 4, encoder_distance, 1);
            break;
        }
    }
}

/*
 ** 函数功能: 按键Process调参
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void KeyProcess(void)
{
    uint8 key_num=0;
    ProcessParameterDisplay(key_num);
    while(1)
    {
        switch(KeyScan())
        {
            case KEY_UP:    //增大参数值
            {
                switch(key_num)
                {
                    case 0: if(process_flag<7)process_flag++;break;
                    case 1: base_speed+=5;          break;
                    case 2: encoder_distance+=1;    break;
                }
                ProcessParameterDisplay(key_num);
                break;
            }
            case KEY_DOWN:  //减小参数值
            {
                switch(key_num)
                {
                    case 0: if(process_flag>0)process_flag--;break;
                    case 1: base_speed-=5;          break;
                    case 2: encoder_distance-=1;    break;
                }
                ProcessParameterDisplay(key_num);
                break;
            }
            case KEY_LEFT:  //向后切换参数
            {
                if(key_num>0)
                {
                    key_num--;
                }
                ProcessParameterDisplay(key_num);
                break;
            }
            case KEY_RIGHT: //向前切换参数
            {
                if(key_num<2)
                {
                    key_num++;
                }
                ProcessParameterDisplay(key_num);
                break;
            }
            case KEY_ENTER: //退出调参
            {
                OLED_clear();
                OLED_ShowNum(0, 1, process_flag, 1, 1);
                OLED_ShowNum(0, 2, base_speed, 3, 1);
                OLED_ShowFloat(0, 3, encoder_distance, 1);
                return;
            }
        }
        systick_delay_ms(STM0,100);
    }
}

/*
 ** 函数功能: 按键Image调参的OLED参数显示
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void ImageParameterDisplay(uint8 key_num_1,uint8 key_num_2)
{
    OLED_clear();
    //先确定key_num_1，再确定key_num_2
    switch(key_num_1)
    {
        case 0: //左十字
        {
            OLED_ShowStr(0, 1, "CrossLoop_L", 2);
            switch(key_num_2)
            {
                case 0:OLED_ShowStr(0, 4, "CrossLoopBegin_L", 1);       break;
                case 1:OLED_ShowStr(0, 4, "CrossLoopOverBegin_L", 1);   break;
                case 2:OLED_ShowStr(0, 4, "CrossLoopEnd_L", 1);         break;
            }
            break;
        }
        case 1: //右车库直行
        {
            OLED_ShowStr(0, 1, "Garage_R", 2);
            switch(key_num_2)
            {
                case 0:OLED_ShowStr(0, 4, "RNINGarageIdentify", 1);break;
                case 1:OLED_ShowStr(0, 4, "ZebraIndentify", 1);break;
            }
            break;
        }
        case 2: //第一遍三岔
        {
            OLED_ShowStr(0, 1, "ForkRoad_First", 2);
            switch(key_num_2)
            {
                case 0:OLED_ShowStr(0, 4, "ForkTurnRIdentify", 1);break;
            }
            break;
        }
        case 3: //右环岛
        {
            OLED_ShowStr(0, 1, "CircleIsland_R", 2);
            switch(key_num_2)
            {
                case 0:OLED_ShowStr(0, 4, "CircleIslandExit_R", 1);     break;
                case 1:OLED_ShowStr(0, 4, "CircleIslandMid_R", 1);      break;
                case 2:OLED_ShowStr(0, 4, "CircleIslandBegin_R", 1);    break;
                case 3:OLED_ShowStr(0, 4, "CircleIslandEnd_R", 1);      break;
                case 4:OLED_ShowStr(0, 4, "CircleIslandOverBegin_R", 1);break;
            }
            break;
        }
        case 4: //右十字
        {
            OLED_ShowStr(0, 1, "CrossLoop_R", 2);
            switch(key_num_2)
            {
                case 0:OLED_ShowStr(0, 4, "CrossLoopBegin_R", 1);       break;
                case 1:OLED_ShowStr(0, 4, "CrossLoopOverBegin_R", 1);   break;
                case 2:OLED_ShowStr(0, 4, "CrossLoopEnd_R", 1);         break;
            }
            break;
        }
        case 5: //左环岛
        {
            OLED_ShowStr(0, 1, "CircleIsland_L", 2);
            switch(key_num_2)
            {
                case 0:OLED_ShowStr(0, 4, "CircleIslandExit_L", 1);     break;
                case 1:OLED_ShowStr(0, 4, "CircleIslandMid_L", 1);      break;
                case 2:OLED_ShowStr(0, 4, "CircleIslandBegin_L", 1);    break;
                case 3:OLED_ShowStr(0, 4, "CircleIslandEnd_L", 1);      break;
                case 4:OLED_ShowStr(0, 4, "CircleIslandOverBegin_L", 1);break;
            }
            break;
        }
        case 6: //第二遍三岔
        {
            OLED_ShowStr(0, 1, "ForkRoad_Second", 2);
            switch(key_num_2)
            {
                case 0:OLED_ShowStr(0, 4, "ForkTurnRIdentify", 1);break;
            }
            break;
        }
        case 7: //左车库入库
        {
            OLED_ShowStr(0, 1, "Garage_In", 2);
            switch(key_num_2)
            {
                case 0:OLED_ShowStr(0, 4, "ZebraCrossingSearch", 1);break;
                case 1:OLED_ShowStr(0, 4, "GarageInBegin", 1);      break;
                case 2:OLED_ShowStr(0, 4, "GarageInEnd", 1);        break;
            }
            break;
        }
    }
}

/*
 ** 函数功能: 按键Image调参的实际参数处理，这个函数在Process中被调用
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void ImageParameterHandle(uint8 key_num_1,uint8 key_num_2,Point InflectionL,Point InflectionR)
{
    uint8 a=0;
    //先确定key_num_1，再确定key_num_2
    switch(key_num_1)
    {
        case 0: //左十字
        {
            switch(key_num_2)
            {
                case 0:CrossLoopBegin_L(LeftLine, RightLine, InflectionL, InflectionR);     break;
                case 1:CrossLoopOverBegin_L(LeftLine, RightLine, InflectionL, InflectionR); break;
                case 2:CrossLoopEnd_L();                                                    break;
            }
            break;
        }
        case 1: //右车库直行
        {
            switch(key_num_2)
            {
                case 0:RNINGarageIdentify(InflectionL, InflectionR);break;
                case 1:ZebraIndentify(80, 50, &a);break;
            }
            break;
        }
        case 2: //第一遍三岔
        {
            switch(key_num_2)
            {
                case 0:if(ForkTurnRIdentify(LeftLine, RightLine, InflectionL, InflectionR)==1)gpio_toggle(LED_WHITE);break;
            }
            break;
        }
        case 3: //右环岛
        {
            switch(key_num_2)
            {
                case 0:CircleIslandExit_R(InflectionR);     break;
                case 1:if(CircleIslandMid_R()==1)gpio_toggle(LED_WHITE);break;
                case 2:CircleIslandBegin_R();               break;
                case 3:CircleIslandEnd_R();                 break;
                case 4:CircleIslandOverBegin_R(RightLine);  break;
            }
            break;
        }
        case 4: //右十字
        {
            switch(key_num_2)
            {
                case 0:CrossLoopBegin_R(LeftLine, RightLine, InflectionL, InflectionR);       break;
                case 1:CrossLoopOverBegin_R(LeftLine, RightLine, InflectionL, InflectionR);   break;
                case 2:CrossLoopEnd_R();                                                      break;
            }
            break;
        }
        case 5: //左环岛
        {
            switch(key_num_2)
            {
                case 0:CircleIslandExit_L(InflectionL);     break;
                case 1:if(CircleIslandMid_L()==1)gpio_toggle(LED_WHITE);break;
                case 2:CircleIslandBegin_L();               break;
                case 3:CircleIslandEnd_L();                 break;
                case 4:CircleIslandOverBegin_L(LeftLine);   break;
            }
            break;
        }
        case 6: //第二遍三岔
        {
            switch(key_num_2)
            {
                case 0:if(ForkTurnRIdentify(LeftLine, RightLine, InflectionL, InflectionR)==1)gpio_toggle(LED_WHITE);break;
            }
            break;
        }
        case 7: //左车库入库
        {
            switch(key_num_2)
            {
                case 0:if(ZebraCrossingSearch(MT9V03X_H/2+15, MT9V03X_H/2-15)==1)gpio_toggle(LED_WHITE); break;
                case 1:GarageInBegin();                                     break;
                case 2:if(GarageInEnd()==1)gpio_toggle(LED_WHITE);          break;
            }
            break;
        }
    }
}

uint8 key_num_1=0,key_num_2=2;  //key_num_1：第一级选择（选择状态）；key_num_2：第二级选择（选择函数）

/*
 ** 函数功能: 按键Image调参
 ** 参    数: 无
 ** 返 回 值: 无
 ** 作    者: WBN
 */
void KeyImage(void)
{
    uint8 sum_num_2=0;  //根据key_num_1决定函数个数从而决定key_num_2
    uint8 flag=0;       //是否有效按下按键
    ImageParameterDisplay(key_num_1, key_num_2);
    while(1)
    {
        //按键扫描
        switch(KeyScan())
        {
            case KEY_UP:    //向前切换函数
            {
                if(key_num_2<sum_num_2) //sum_num_2+1个函数
                {
                    key_num_2++;
                    flag=1;
                }
                break;
            }
            case KEY_DOWN:  //向后切换函数
            {
                if(key_num_2>0)
                {
                    key_num_2--;
                    flag=1;
                }
                break;
            }
            case KEY_LEFT:  //向后切换状态
            {
                if(key_num_1>0)
                {
                    key_num_1--;
                    key_num_2=0;
                    flag=1;
                }
                break;
            }
            case KEY_RIGHT: //向前切换状态
            {
                if(key_num_1<7) //8个状态
                {
                    key_num_1++;
                    key_num_2=0;
                    flag=1;
                }
                break;
            }
            case KEY_ENTER: //
            {

                break;
            }
        }
        //按键数据处理
        if(flag==1) //有效按键按下
        {
            switch(key_num_1)
            {
                case 0:sum_num_2=2; break;  //左十字
                case 1:sum_num_2=1; break;  //左车库
                case 2:sum_num_2=0; break;  //三岔
                case 3:sum_num_2=4; break;  //右环岛
                case 4:sum_num_2=2; break;  //右十字
                case 5:sum_num_2=4; break;  //左环岛
                case 6:sum_num_2=0; break;  //三岔
                case 7:sum_num_2=2; break;  //入库
            }
            ImageParameterDisplay(key_num_1, key_num_2);    //按键参数显示
            flag=0;
        }

        systick_delay_ms(STM0,100);
    }
}

