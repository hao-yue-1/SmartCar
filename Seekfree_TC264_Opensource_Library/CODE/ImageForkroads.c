/*
 * Forkroads.c
 * Created on: 2022年5月25日
 * Author: 30516
 * Effect: 存放三岔路口相关的源代码
 * 三岔测试最高速度为260状态机+-15跟接的弯道同方向最好跑250比较稳
 */

#include "ImageSpecial.h"
#include "PID.h"
#include <stdlib.h> //abs函数，fabs在math.h
#include "LED.h"
#include "zf_gpio.h"
#include "Motor.h"//编码器测距
#include "ICM20602.h"//陀螺仪测角度

#define L_FINDWHIDE_THRE  10 //Y拐点中间找左边白色区域停止的阈值
#define R_FINDWHIDE_THRE  150//Y拐点中间找右边白色区域停止的阈值
#define ROW_FINDWHIDE_THRE 110//Y拐点行的下限的阈值
#define CLOUMN_FINDWHIDE_THRE   10//列左右寻找白色区域的宽度，如果找不到则进入种子生长
#define SEED_R_TRANSVERSE_GROW_THRE    5//三岔种子生长从右往左边横向生长的阈值范围，超出则默认横向生长的第一个为拐点
#define SEED_L_TRANSVERSE_GROW_THRE    10//从左往右生长的阈值
#define FORK_INFLECTION_WIDTH  110//打开三岔debug,当拐点在60行附近左右拐点的差值，补全的时候，依据没有丢失的拐点的行数做一个简单的比例关系到单边循迹思路上
#define FORK_DEBUG  0
#define FORK_LED_DEBUG  0   //状态机的LEDdebug

extern uint8 bias_startline,bias_endline;        //动态前瞻

/*********************************************************************************
 ** 函数功能: 寻找三岔左边上顶点，防止因为扫线扫到三岔另外一边使得Bias太小
 ** 参    数: uint8 row,uint8 cloumn:去找寻虚拟补线折现上点的基准点坐标
 **           Point *LeftUpPoint:左边的特殊点
 ** 返 回 值: 无
 ** 说    明: 用完这个函数之后应该再补一条左边的垂直线和右边从上拐点补到左上角拐点
 ** 作    者: LJF
 **********************************************************************************/
typedef struct SeedGrowAqueue
{
    uint8 front;//队头
    uint8 rear;//队尾
}SeedGrowAqueue;//找到拐点之后再次八领域扫线的种子生长的队列
uint8 const l_data[8] = {0,1,2,3,4,5,6,7};//左种子标号队列数据域
uint8 const r_data[8] = {4,3,2,1,0,7,6,5};//右种子标号队列数据域
void ForkSearchLineAgain(int *UpInflectionY,Point left_seed,Point right_seed)
{
    SeedGrowAqueue seed_queue;//左右线用于标号转移的循环队列
    seed_queue.front = 0; seed_queue.rear = 7;
    //左种子生长:第三个条件是代表周围八个领域不是白色
    int left_seed_grow_starcolumn=left_seed.X-1;
    for(;left_seed.X>left_seed_grow_starcolumn && left_seed.Y>bias_endline-1 && left_seed.Y<*UpInflectionY+1 && (seed_queue.front + 1) % 8 != seed_queue.rear;)
    {
        uint8 break_for_flag = 0;//用于判断是否找到黑色的区域从而不继续标号查看生长
        //判断种子是丢线还是已经到了前边的大片黑色区域
        if (left_seed.X + 1 >= MT9V03X_W && BinaryImage[left_seed.Y][left_seed.X] == IMAGE_BLACK)//左种子长到了右边，并且还是黑色说明这一段已经步入了全黑,赛道外了
        {
            LeftLine[left_seed.Y] = 0;
            left_seed.Y--;
        }
        else if (left_seed.X - 1 <= 0 && (BinaryImage[left_seed.Y][left_seed.X] == IMAGE_WHITE|| BinaryImage[left_seed.Y-1][left_seed.X] == IMAGE_WHITE))//左种子一直在左边边界，并且是白色，说明是正常丢线
        {
            LeftLine[left_seed.Y] = 0;
            left_seed.Y--;
        }
        else
        {
            for (; (seed_queue.front + 1) % 8 != seed_queue.rear; seed_queue.front++)
            {
                switch (l_data[seed_queue.front])
                {
                case 0:
                    if (BinaryImage[left_seed.Y][left_seed.X + 1] == IMAGE_BLACK)
                    {
                        left_seed.X++;
                        break_for_flag = 1;
                    }
                    break;
                case 1:
                    if (BinaryImage[left_seed.Y - 1][left_seed.X + 1] == IMAGE_BLACK)
                    {
                        left_seed.X++; left_seed.Y--;
                        break_for_flag = 1;
                    }
                    break;
                case 2:
                    if (BinaryImage[left_seed.Y - 1][left_seed.X] == IMAGE_BLACK)
                    {
                        left_seed.Y--;
                        break_for_flag = 1;
                    }
                    break;
                case 3:
                    if (BinaryImage[left_seed.Y - 1][left_seed.X - 1] == IMAGE_BLACK)
                    {
                        left_seed.X--; left_seed.Y--;
                        break_for_flag = 1;
                    }
                    break;
                case 4:
                    if (BinaryImage[left_seed.Y][left_seed.X - 1] == IMAGE_BLACK)
                    {
                        left_seed.X--;
                        break_for_flag = 1;
                    }
                    break;
                case 5:
                    if (BinaryImage[left_seed.Y + 1][left_seed.X - 1] == IMAGE_BLACK)
                    {
                        left_seed.X--; left_seed.Y++;
                        break_for_flag = 1;
                    }
                    break;
                case 6:
                    if (BinaryImage[left_seed.Y + 1][left_seed.X] == IMAGE_BLACK)
                    {
                        left_seed.Y++;
                        break_for_flag = 1;
                    }
                    break;
                case 7:
                    if (BinaryImage[left_seed.Y + 1][left_seed.X + 1] == IMAGE_BLACK)
                    {
                        left_seed.X++; left_seed.Y++;
                        break_for_flag = 1;
                    }
                    break;
                default:
                    break;
                }
                //在这里判断一下前面的switch有没有找到黑色区域
                if (break_for_flag == 1)
                {
                    char temp = seed_queue.front;
                    if (temp - 2 < 0)//判断是否-2会小于0，会的话就下一次从0号找
                    {
                        seed_queue.front = 0;
                    }
                    else
                    {
                        seed_queue.front = temp - 2;
                    }
                    seed_queue.rear = (seed_queue.front + 7) % 8;//重置队头
                    break;//跳出这次的8领域寻找，进入下一个种子8领域搜索
                }
            }
            //通过点的右边是否是白色来判断它是否是边界点，否则边界点可能会在黑色区域，但是种子还要继续往右边走，却没有被存入了
            if (BinaryImage[left_seed.Y][left_seed.X + 1] == IMAGE_WHITE)
            {
                LeftLine[left_seed.Y] = left_seed.X;
                CentreLine[left_seed.Y]=(left_seed.X+RightLine[left_seed.Y])/2;
            }
        }
    }
    //右种子生长
    seed_queue.front = 0; seed_queue.rear = 7;//重置队头和队尾，但是数据域进行改变即可
    int right_seed_grow_startline=right_seed.Y;
    for(;right_seed.Y>bias_endline-1 && right_seed.Y<right_seed_grow_startline+1 && (seed_queue.front+1)%8!=seed_queue.rear;)
    {
        uint8 break_for_flag = 0;//用于判断是否找到黑色的区域从而不继续标号查看生长
        //判断种子是丢线还是已经到了前边的大片黑色区域
        if (right_seed.X - 1 <= 0 && BinaryImage[right_seed.Y][right_seed.X] == IMAGE_BLACK)//右种子长到了左边，并且还是黑色说明这一段已经步入了全黑,赛道外了
        {
            RightLine[right_seed.Y] = MT9V03X_W-1;
            right_seed.Y--;
        }
        else if (right_seed.X + 1 >= MT9V03X_W && (BinaryImage[right_seed.Y][right_seed.X] == IMAGE_WHITE || BinaryImage[right_seed.Y - 1][right_seed.X] == IMAGE_WHITE))//右种子一直在右边边界，并且是白色，说明是正常丢线
        {
            RightLine[right_seed.Y] = MT9V03X_W - 1;
            right_seed.Y--;
        }
        else
        {
            for (; (seed_queue.front + 1) % 8 != seed_queue.rear; seed_queue.front++)
            {
                switch (r_data[seed_queue.front])
                {
                case 0:
                    if (BinaryImage[right_seed.Y][right_seed.X + 1] == IMAGE_BLACK)
                    {
                        right_seed.X++;
                        break_for_flag = 1;
                    }
                    break;
                case 1:
                    if (BinaryImage[right_seed.Y - 1][right_seed.X + 1] == IMAGE_BLACK)
                    {
                        right_seed.X++; right_seed.Y--;
                        break_for_flag = 1;
                    }
                    break;
                case 2:
                    if (BinaryImage[right_seed.Y - 1][right_seed.X] == IMAGE_BLACK)
                    {
                        right_seed.Y--;
                        break_for_flag = 1;
                    }
                    break;
                case 3:
                    if (BinaryImage[right_seed.Y - 1][right_seed.X - 1] == IMAGE_BLACK)
                    {
                        right_seed.X--; right_seed.Y--;
                        break_for_flag = 1;
                    }
                    break;
                case 4:
                    if (BinaryImage[right_seed.Y][right_seed.X - 1] == IMAGE_BLACK)
                    {
                        right_seed.X--;
                        break_for_flag = 1;
                    }
                    break;
                case 5:
                    if (BinaryImage[right_seed.Y + 1][right_seed.X - 1] == IMAGE_BLACK)
                    {
                        right_seed.X--; right_seed.Y++;
                        break_for_flag = 1;
                    }
                    break;
                case 6:
                    if (BinaryImage[right_seed.Y + 1][right_seed.X] == IMAGE_BLACK)
                    {
                        right_seed.Y++;
                        break_for_flag = 1;
                    }
                    break;
                case 7:
                    if (BinaryImage[right_seed.Y + 1][right_seed.X + 1] == IMAGE_BLACK)
                    {
                        right_seed.X++; right_seed.Y++;
                        break_for_flag = 1;
                    }
                    break;
                default:
                    break;
                }
                //在这里判断一下前面的switch有没有找到黑色区域
                if (break_for_flag == 1)
                {
                    char temp = seed_queue.front;
                    if (temp - 2 < 0)//判断是否-2会小于0，会的话就下一次从0号找
                    {
                        seed_queue.front = 0;
                    }
                    else
                    {
                        seed_queue.front = temp - 2;
                    }
                    seed_queue.rear = (seed_queue.front + 7) % 8;//重置队头
                    break;//跳出这次的8领域寻找，进入下一个种子8领域搜索
                }
            }
            //通过点的右边是否是白色来判断它是否是边界点，否则边界点可能会在黑色区域，但是种子还要继续往右边走，却没有被存入了
            if (BinaryImage[right_seed.Y][right_seed.X - 1] == IMAGE_WHITE)
            {
                RightLine[right_seed.Y] = right_seed.X;
                CentreLine[right_seed.Y]=(LeftLine[right_seed.Y]+right_seed.X)/2;
            }
        }
    }
    //如果找到了结束行停下来的那么更新上拐点的结束行便于循迹做偏差
//    LcdDrawRow(left_seed.Y, PURPLE);
//    lcd_showint32(TFT_X_MAX-50, 2, left_seed.Y, 3);
    if(left_seed.Y<*UpInflectionY) *UpInflectionY=left_seed.Y;
}
/*********************************************************************************
 ** 函数功能: 三岔种子生长生长至谷底寻找Y上拐点
 ** 参    数:char Choose：选择是在谷的左边还是右边
 **          Point Seed
 **          int endline
 **          Point *UpInflectionC
 ** 返 回 值: 无
 ** 说    明: 因为选择走三岔右边的话从左边往谷底爬是一定会爬到底的所以，左边不需要加transeverse的限制，能更好的找到那个Y拐点
 ** 作    者: LJF
 **********************************************************************************/
void SeedGrowFindUpInflection(char Choose,Point Seed,int endline,Point *UpInflectionC)
{
    char transversenum=0;//记录种子是否一直横向移动,种子横向生长的次数
    Point tempSeed;//临时的种子
    for(;Seed.Y<endline && Seed.X<MT9V03X_W-1 && Seed.X>0;)
    {
#if FORK_DEBUG
        lcd_drawpoint(Seed.X, Seed.Y, GREEN);
#endif
        switch(Choose)
        {
            case 'L':
                if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_BLACK && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_BLACK)
                {
                    Seed.Y++,Seed.X++;
                    transversenum=0;
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_BLACK && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_WHITE)
                {
                    Seed.Y++;
                    transversenum=0;
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_WHITE && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_BLACK)
                {
                    Seed.X++;
                    if(transversenum==0)//判断是否是第一次往右走
                    {
                        tempSeed=Seed;
                    }
                    transversenum++;//说明在往右边走
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_WHITE && BinaryImage[Seed.Y][Seed.X+1]==IMAGE_WHITE)
                {
                    if(transversenum!=0)//说明之前一直都是往右走找到了谷底
                    {
                        UpInflectionC->Y=tempSeed.Y,UpInflectionC->X=tempSeed.X;
                    }
                    else
                    {
                        UpInflectionC->Y=Seed.Y,UpInflectionC->X=Seed.X;
                    }
                    return;
                }
                //当种子横向生长的次数大于了阈值
                if(transversenum>SEED_L_TRANSVERSE_GROW_THRE)
                {
                    UpInflectionC->Y=tempSeed.Y,UpInflectionC->X=tempSeed.X;
                    return;
                }
                break;
            case 'R':
                if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_BLACK && BinaryImage[Seed.Y][Seed.X-1]==IMAGE_BLACK)
                {
                    Seed.Y++,Seed.X--;
                    transversenum=0;
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_BLACK && BinaryImage[Seed.Y][Seed.X-1]==IMAGE_WHITE)
                {
                    Seed.Y++;
                    transversenum=0;
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_WHITE && BinaryImage[Seed.Y][Seed.X-1]==IMAGE_BLACK)
                {
                    Seed.X--;
                    if(transversenum==0)//判断是否是第一次往右走
                    {
                        tempSeed=Seed;
                    }
                    transversenum++;;//说明在往左边走
                }
                else if(BinaryImage[Seed.Y+1][Seed.X]==IMAGE_WHITE && BinaryImage[Seed.Y][Seed.X-1]==IMAGE_WHITE)
                {
                    if(transversenum!=0)//说明之前一直都是往右走找到了谷底
                    {
                        UpInflectionC->Y=tempSeed.Y,UpInflectionC->X=tempSeed.X;
                    }
                    else
                    {
                        UpInflectionC->Y=Seed.Y,UpInflectionC->X=Seed.X;
                    }
                    return;
                }
                //当种子横向生长的次数大于了阈值
                if(transversenum>SEED_R_TRANSVERSE_GROW_THRE)
                {
                    UpInflectionC->Y=tempSeed.Y,UpInflectionC->X=tempSeed.X;
                    return;
                }
                break;
            default:break;
        }
        if(Seed.X<0 || Seed.X>MT9V03X_W-1 || Seed.Y<0 || Seed.Y>MT9V03X_H-1)
        {
            break;//如果找的线超出了屏幕则退出，避免程序卡死
        }
    }
}
/*********************************************************************************
 ** 函数功能: 根据左右下拐点搜寻出三岔上拐点
 ** 参    数: Point InflectionL: 左边拐点
 **           Point InflectionR: 右边拐点
 **           Point *UpInflectionC: 中间上拐点
 ** 返 回 值: 无
 ** 说    明: 这个函数最后面会修改，写入在找拐点里面，或者还要加一个元素类型的参数，根据类型来找不同的拐点，这里只针对三岔
 ** 作    者: LJF
 **********************************************************************************/
void GetForkUpInflection(Point DownInflectionL,Point DownInflectionR,Point *UpInflectionC)
{
    int row=0, cloumnL=0, cloumnR=0, cloumnnum=0;//cloumnnum记录从基准点偏移了多少列黑色区域
    Point Seed;
    char Choose=0,flagL=0,flagR=0;//判断是在谷的左边还是右边的函数,以及判断左右两边有没有白色区域的FLAG
    UpInflectionC->X = 0; UpInflectionC->Y = 0;//上拐点置零
    UpInflectionC->X = (DownInflectionL.X + DownInflectionR.X) / 2;//V型上拐点的列坐标为左右拐点均值
    //防止因为单边寻拐点使得溢出
    if(UpInflectionC->X<0) UpInflectionC->X=0;
    if(UpInflectionC->X>MT9V03X_W-1) UpInflectionC->X=MT9V03X_W-1;
    row = (DownInflectionL.Y + DownInflectionR.Y) / 2;//起始行为左右拐点行的均值
    if(row<0) row=0;
    if(row>MT9V03X_H-1) row=MT9V03X_H-1;//避免溢出，增强程序健壮性
    //判断一次开始的点是否为白点，否则如果开始的点为黑点，一直找白跳黑可能找到一些杂乱的地方
    if(BinaryImage[row][UpInflectionC->X]==IMAGE_BLACK)
        return;//如果开始点就是黑色，那么直接跳出
    for (; row > 20; row--)
    {
#if FORK_DEBUG
        lcd_drawpoint(UpInflectionC->X, row, PURPLE);
#endif
        if (BinaryImage[row][UpInflectionC->X] == IMAGE_WHITE && BinaryImage[row - 1][UpInflectionC->X] == IMAGE_BLACK)
        {
            //往左往右找白色区域
            for (cloumnL = UpInflectionC->X; cloumnL > L_FINDWHIDE_THRE; cloumnL--)
            {
#if FORK_DEBUG
                lcd_drawpoint(cloumnL, row-1, YELLOW);
#endif
                if(BinaryImage[row - 1][cloumnL] == IMAGE_BLACK)
                {
                    cloumnnum++;
                }
                if (BinaryImage[row - 1][cloumnL] == IMAGE_BLACK && BinaryImage[row - 1][cloumnL - 1] == IMAGE_WHITE && BinaryImage[row - 1][cloumnL - 3] == IMAGE_WHITE)
                {
                    cloumnnum=0;
                    flagL = 1;
                    break;
                }
                //条件1：循环结束了还没找到白色区域，或，条件2：黑色区域太多了大于了阈值，就算能找到白色区域补的线也不好
                if (cloumnL == L_FINDWHIDE_THRE + 1 || cloumnnum>CLOUMN_FINDWHIDE_THRE)
                {
                    cloumnnum=0;
                    Choose = 'R';//左边找不到说明在谷的右边
                    break;
                }
            }
            for (cloumnR = UpInflectionC->X; cloumnR < R_FINDWHIDE_THRE && Choose!='R'; cloumnR++)
            {
#if FORK_DEBUG
                lcd_drawpoint(cloumnR, row-1, YELLOW);
#endif
                if(BinaryImage[row - 1][cloumnR] == IMAGE_BLACK)
                {
                    cloumnnum++;
                }
                if (BinaryImage[row - 1][cloumnR] == IMAGE_BLACK && BinaryImage[row - 1][cloumnR + 1] == IMAGE_WHITE && BinaryImage[row - 1][cloumnL + 3] == IMAGE_WHITE)
                {
                    cloumnnum=0;
                    flagR = 1;
                    break;
                }
                if (cloumnR == R_FINDWHIDE_THRE - 1 || cloumnnum>CLOUMN_FINDWHIDE_THRE)
                {
                    cloumnnum=0;
                    Choose = 'L';//右边找不到说明在谷的左边
                    break;
                }
            }
            break;
        }
    }
    if (flagL == 0 || flagR == 0)//说明有一边是没有白色区域的
    {
        if(Choose!=0)
        {
            Seed.X = UpInflectionC->X, Seed.Y = row - 1;
#if FORK_DEBUG
            for(int j=0;j<MT9V03X_W-1;j++)//画出100行那条线
            {
                lcd_drawpoint(j, ROW_FINDWHIDE_THRE, PURPLE);
            }
#endif
            SeedGrowFindUpInflection(Choose, Seed, ROW_FINDWHIDE_THRE, UpInflectionC);
            //在此再次验证一次推箱子找到的拐点是不是真的拐点，防止误判
            for (cloumnL = UpInflectionC->X; cloumnL > L_FINDWHIDE_THRE; cloumnL--)
            {
                if (BinaryImage[UpInflectionC->Y][cloumnL] == IMAGE_WHITE && BinaryImage[UpInflectionC->Y][cloumnL-3]==IMAGE_WHITE)
                {
                    break;
                }
                if (cloumnL == L_FINDWHIDE_THRE + 1 )
                {
                    //横着读完了都是黑色的那么再往两边的最下面看一下是否有白
                    if(BinaryImage[UpInflectionC->Y+1][cloumnL]==IMAGE_WHITE)//下面一个点也是白色那么就也算是三岔拐点
                        break;
                    else
                    {
                        UpInflectionC->Y=0;//如果找不到白色的判定为误判
                        break;
                    }
                }
            }
            for (cloumnR = UpInflectionC->X; cloumnR < R_FINDWHIDE_THRE; cloumnR++)
            {
                if (BinaryImage[UpInflectionC->Y][cloumnR] == IMAGE_WHITE && BinaryImage[UpInflectionC->Y][cloumnR+3] == IMAGE_WHITE)
                {
                    break;
                }
                if (cloumnR == R_FINDWHIDE_THRE - 1)
                {
                    //横着读完了都是黑色的那么再往两边的最下面看一下是否有白
                    if(BinaryImage[UpInflectionC->Y+1][cloumnR]==IMAGE_WHITE)//下面一个点也是白色那么就也算是三岔拐点
                        break;
                    else
                    {
                        UpInflectionC->Y=0;//如果找不到白色的判定为误判
                        break;
                    }
                }
            }
        }
    }
    else
    {
        UpInflectionC->Y = row - 1;
    }
}

/********************************************************************************************
 ** 函数功能: 识别三岔
 ** 参    数:
 **           int *LeftLine：左线
 **           int *RightLine:右线
 **           Point *InflectionL:左边拐点
 **           Point *InflectionR:右边拐点
 ** 返 回 值:  0：没有识别到三岔
 **           1：识别到三岔
 ** 作    者: LJF
 ** 注    意：向右边进入三岔
 *********************************************************************************************/
uint8 ForkTurnRIdentify(int *LeftLine,int *RightLine,Point DownInflectionL,Point DownInflectionR)
{
    Point UpInflectionC;//上拐点，左边拐点，左边上顶点，补折线
    UpInflectionC.X=0;UpInflectionC.Y=0;
    //当左右拐点存在,并且两个拐点要在图像下半部分
    if(DownInflectionL.X!=0 && DownInflectionR.X!=0 && DownInflectionL.X<120 && DownInflectionR.X>30)
    {
        //取消这个左右拐点行数的判断，增加运算速率
        if(abs((DownInflectionL.Y-DownInflectionR.Y))<40)//左右两个拐点的行数小于30，才进行判断
        {
            GetForkUpInflection(DownInflectionL, DownInflectionR, &UpInflectionC);//去搜索上拐点
//            gpio_toggle(LED_BLUE);
        }
    }
    else if((DownInflectionL.X==0 && DownInflectionR.X==0) || (BinaryImage[MT9V03X_H-5][5]==IMAGE_WHITE && BinaryImage[MT9V03X_H-5][MT9V03X_W-5]==IMAGE_WHITE))//如果左右下拐点不存在并且下面一段出现就丢线的话的话,我们就去看存不存在正上的拐点
    {
        Point ImageDownPointL,ImageDownPointR;//以画面的左下角和右下角作为左右补线的点
        //给上拐点的列一个预测的点，而不是写死为屏幕中间
        if(LeftLine[MT9V03X_H-1]!=0)//如果最下面的一行没有丢线
            ImageDownPointL.X=LeftLine[MT9V03X_H-1]+10,ImageDownPointL.Y=MT9V03X_H-1;
        else ImageDownPointL.X=0,ImageDownPointL.Y=MT9V03X_H-1;
        if(RightLine[MT9V03X_H-1]!=MT9V03X_W-1)
            ImageDownPointR.X=RightLine[MT9V03X_H-1]-10,ImageDownPointR.Y=MT9V03X_H-1;
        else ImageDownPointR.X=MT9V03X_W-1,ImageDownPointR.Y=MT9V03X_H-1;
        //找寻上拐点
        DownInflectionL=ImageDownPointL;
        GetForkUpInflection(ImageDownPointL, ImageDownPointR, &UpInflectionC);
        if(UpInflectionC.Y<40) UpInflectionC.Y=0;
//        gpio_toggle(LED_GREEN);
    }
    //右边丢线超过60，左拐点存在，并且左拐点不能在上半平屏防止误判
    else if(LostNum_RightLine>55 && DownInflectionL.X>0 && DownInflectionL.X<90 && DownInflectionL.Y>40)
    {
        Point ImageDownPointR;//以左拐点对称的点去补线和找拐点
        //给自己设定的右拐点去找上拐点
//        ImageDownPointR.X=MT9V03X_W-1,ImageDownPointR.YDownInflectionL.Y=DownInflectionL.Y;
        //运用单边循迹法的思想给拐点，赛道宽度，左斜找上拐点右边的多往右一点
        ImageDownPointR.X=DownInflectionL.X+(145-(119-DownInflectionL.Y)*1.1);ImageDownPointR.Y=DownInflectionL.Y;
        GetForkUpInflection(DownInflectionL, ImageDownPointR, &UpInflectionC);
        DownInflectionR=ImageDownPointR;
//        gpio_toggle(LED_RED);
    }
    //左边丢线超过60,右拐点存在,并且右拐点不能在上半平屏防止误判
    else if(LostNum_LeftLine>55 && DownInflectionR.X>60 && DownInflectionR.Y>40)
    {
        Point ImageDownPointL;//以左拐点对称的点去补线和找拐点
        //与拐点行数做一个比例关系，越靠近底部了拐点宽度越大.左斜多往左一点
//        ImageDownPointL.X=DownInflectionR.X-(145-(119-DownInflectionL.Y)*1.1);ImageDownPointL.Y=DownInflectionR.Y;
        ImageDownPointL.X=DownInflectionR.X-FORK_INFLECTION_WIDTH;ImageDownPointL.Y=DownInflectionR.Y;
        GetForkUpInflection(ImageDownPointL, DownInflectionR, &UpInflectionC);
        //右斜的时候，其实左拐点会在左下屏幕的最下角，所以用于补线的时候以左下角为补线点
        ImageDownPointL.X=1;ImageDownPointL.Y=MT9V03X_H-1;
        DownInflectionL=ImageDownPointL;
//        gpio_toggle(LED_WHITE);
    }
    if(UpInflectionC.Y!=0)//直接访问Y即可，加快速度，因为X默认就会赋值了
    {
        //补线
        FillingLine('L',DownInflectionL,UpInflectionC);//三岔成立了就在返回之前补线
        if(CentreLine[UpInflectionC.Y]>CentreLine[UpInflectionC.Y-3])//如果扫线扫到了和路径规划相反方向，重新扫一次线
        {
            Point right_seed;
            right_seed.X=0;right_seed.Y=0;
            for(int column=UpInflectionC.X;column<MT9V03X_W-1;column++)
            {
                if(BinaryImage[column][UpInflectionC.Y]==IMAGE_WHITE&&BinaryImage[column+1][UpInflectionC.Y]==IMAGE_BLACK)
                {
                    right_seed.X=column+1;right_seed.Y=UpInflectionC.Y;
                }
                else if(column==MT9V03X_W-2)
                {
                    right_seed.X=MT9V03X_W-1;right_seed.Y=UpInflectionC.Y;
                }
            }
            ForkSearchLineAgain(&(UpInflectionC.Y),UpInflectionC,right_seed);
        }
        //偏差处理
//        if(UpInflectionC.Y<bias_endline)//starline<endline<Up.y，则正常循迹
//        {
//            ;
//        }
//        else if(UpInflectionC.Y<bias_startline && bias_endline<UpInflectionC.Y)//starline<UP.y<endline,则按照起始行到上拐点
//        {
//            bias_endline=UpInflectionC.Y;
//        }
//        else if(bias_startline<UpInflectionC.Y)//UP.y<starline<endline
//        {
//            bias_startline=DownInflectionL.Y;bias_endline=UpInflectionC.Y;
//        }
        return 1;
    }
    else
        return 0;
}
/********************************************************************************************
 ** 函数功能: 三岔状态跳转判断函数
 ** 参    数: Point InflectionL：左下拐点
 **           Point InflectionR：右下拐点
 ** 返 回 值:  0：三岔还未结束
 **           1：三岔已结束
 ** 作    者: LJF
 *********************************************************************************************/
uint8 ForkFStatusIdentify(Point DownInflectionL,Point DownInflectionR,uint8 *ForkFlag)
{
    static uint8 StatusChange;//状态转移变量、出口帧数延迟
    uint8 NowFlag=0;//这次的识别结果
    NowFlag=ForkTurnRIdentify(LeftLine, RightLine, DownInflectionL, DownInflectionR);
    *ForkFlag=NowFlag;//把识别结果送出去
    //状态机开始部分
    switch(StatusChange)
    {
        //入口状态
        case 0:
        {
#if FORK_LED_DEBUG
            gpio_set(LED_RED, 0);
#endif
            if(NowFlag==1)
            {
#if FORK_LED_DEBUG
                gpio_set(LED_RED, 1);
#endif
                EncoderDistance(1, 1.7, 0, 0);//避免因为误判或者三岔口中有一帧没判断到而把状态打乱
                StatusChange=1;//只要开始识别到了三岔就说明已经是入口阶段了
            }
            break;
        }
        //走完入口状态
        case 1:
        {
#if FORK_LED_DEBUG
            gpio_set(LED_YELLOW, 0);
#endif
            if(NowFlag==0)
            {
#if FORK_LED_DEBUG
                gpio_set(LED_YELLOW, 1);
#endif
                base_speed+=15;
                StatusChange=2;
            }
            break;
        }
        //中途状态
        case 2:
        {
#if FORK_LED_DEBUG
            gpio_set(LED_BLUE, 0);
#endif
            if(NowFlag==1)
            {
                if(encoder_dis_flag!=1)//测距没完成
                {
#if FORK_LED_DEBUG
                    gpio_set(LED_BLUE, 1);
#endif
                    base_speed-=15;//因为还没进入口所以速度降回去
                    EncoderDistance(1, 1.7, 0, 0);//避免因为误判或者三岔口中有一帧没判断到而把状态打乱
                    StatusChange=1;//如果还没有测完距我就认为我上一次的识别到三岔为误判，则返回三岔入口状态
                }
                else
                {
#if FORK_LED_DEBUG
                    gpio_set(LED_BLUE, 1);
#endif
                    base_speed-=15;
                    StartIntegralAngle_Z(30);//陀螺仪开启积分准备出三岔
                    StatusChange=3;
                }
            }
            break;
        }
        //出口
        case 3:
        {
#if FORK_LED_DEBUG
            gpio_set(LED_WHITE, 0);
#endif
            if(icm_angle_z_flag==1)
            {
#if FORK_LED_DEBUG
                gpio_set(LED_WHITE, 1);
#endif
                StatusChange=4;
            }
            break;
        }
        case 4:
        {
            if(NowFlag==0)
            {
                StatusChange=0;//重置状态机标志变量使得状态机能再次使用
                return 1;
            }
            break;
        }
        default:break;
    }
    return 0;
}

/********************************************************************************************
 ** 函数功能: 三岔状态跳转判断函数
 ** 参    数: Point InflectionL：左下拐点
 **           Point InflectionR：右下拐点
 ** 返 回 值:  0：三岔还未结束
 **           1：三岔已结束
 ** 作    者: LJF
 *********************************************************************************************/
uint8 ForkSStatusIdentify(Point DownInflectionL,Point DownInflectionR,uint8 *ForkFlag)
{
    static uint8 StatusChange,numentrance,fork_encooder_flag,last_speed;//三岔识别函数的临时状态变量，用来看状态是否跳转
    uint8 NowFlag=0;//这次的识别结果
    NowFlag=ForkTurnRIdentify(LeftLine, RightLine, DownInflectionL, DownInflectionR);
    *ForkFlag=NowFlag;//把识别结果送出去
    //状态机开始部分
    switch(StatusChange)
    {
        //入口状态
        case 0:
        {
#if FORK_LED_DEBUG
            gpio_set(LED_RED, 0);
#endif
            if(NowFlag==1)
            {
#if FORK_LED_DEBUG
                gpio_set(LED_RED, 1);
#endif
                last_speed=base_speed;
                StatusChange=1;//只要开始识别到了三岔就说明已经是入口阶段了
            }
            break;
        }
        //入口到坡道起点的状态
        case 1:
        {
#if FORK_LED_DEBUG
            gpio_set(LED_YELLOW, 0);
#endif
            if(numentrance<10)
            {
                numentrance++;
                break;
            }
            if(NowFlag==0)
            {
#if FORK_LED_DEBUG
                gpio_set(LED_YELLOW, 1);
#endif
                EncoderDistance(1, 0.7, 0, 0);//开启测距,测距上坡减速
                StatusChange=2;
            }
            break;
        }
        //坡道中途状态
        case 2:
        {
#if FORK_LED_DEBUG
            gpio_set(LED_WHITE, 0);
#endif
            if(encoder_dis_flag==1)
            {
#if FORK_LED_DEBUG
                gpio_set(LED_WHITE, 1);
#endif
                base_speed=150;
                EncoderDistance(1, 1.2, 0, 0);//开启测距,坡道
                StatusChange=3;
            }
            break;
        }
        //三岔检测出口状态
        case 3:
        {
#if FORK_LED_DEBUG
            gpio_set(LED_BLUE, 0);
#endif
            if(encoder_dis_flag==1)
            {
#if FORK_LED_DEBUG
                gpio_set(LED_BLUE, 1);
#endif
                base_speed=last_speed;//恢复速度
                StatusChange=4;
            }
            break;
        }
        //出三岔的状态
        case 4:
        {
#if FORK_LED_DEBUG
            gpio_set(LED_GREEN, 0);
#endif
            if(NowFlag==1)
            {
#if FORK_LED_DEBUG
            gpio_set(LED_GREEN, 1);
#endif
                StartIntegralAngle_Z(30);//陀螺仪开启积分准备出三岔
                StatusChange=5;
            }
            break;
        }
        case 5:
        {
#if FORK_LED_DEBUG
            gpio_set(LED_RED, 0);
#endif
            if(icm_angle_z_flag==1)
            {
#if FORK_LED_DEBUG
            gpio_set(LED_RED, 1);
#endif
                StatusChange=6;
            }
            break;
        }
        case 6:
        {
#if FORK_LED_DEBUG
            gpio_set(LED_YELLOW, 0);
#endif
            if(NowFlag==0)
            {
#if FORK_LED_DEBUG
            gpio_set(LED_YELLOW, 1);
#endif
                return 1;
            }
            break;
        }
        default:break;
    }
    return 0;
}

