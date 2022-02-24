/*
 * ImageSpecial.c
  *  �����ϸ�������Ԫ�ص�ʶ��
  *  ���ļ��ĺ���ֻʵ��ʶ���Ƿ��и�Ԫ�صĳ����Լ����ֵ�λ�ã����ڸ�Ԫ�س����˶��ٴ��Լ���Ӧ��ʩ�����ڸ��ļ���Χ��
 *  Created on: 2022��1��17��
 *      Author: yue
 */

#include "ImageSpecial.h"

//�����ж�flag
uint8 Flag_CircleBegin=0;   //���ֻ���
uint8 Flag_CircleIn=0;      //�������

/*
 *******************************************************************************************
 ** ��������: ʶ��������
 ** ��    ��: *LeftLine��  ��������
 **           *RightLine����������
 ** �� �� ֵ: 0��û��ʶ��������
 **           1��ʶ���������ҳ����ڳ����
 **           2��ʶ���������ҳ����ڳ��Ҳ�
 ** ��    ��: WBN
 ** ע    �⣺1 . Ĭ���ڽ���������ʶ���·�ζ���ֱ�ߣ�����ͷ�Ѿ�����
 **           2.����û��ʵ��ͼ���ο���ֻ���ȼ���һ������״̬������������ǡ�ò�������ͼ��
 ********************************************************************************************
 */
uint8 StartLineFlag(int *LeftLine,int *RightLine)
{
    /*
     ** �������ߵĵط����г��⣬��������ԭ�򣬳����п��ܳ����ڳ�����߻��ұߣ�����Ҳ�ȶ���������������жϲ��ֱ�����
     ** �Գ��������Ϊ�����ӹյ㿪ʼ��Ϊɨ�ߵ�ԭ�㣬�������ϣ���������ɨ��
     ** �̶�һ�У���������ɨ���ҵ�һ���ڵ㣬�̶����У���������ɨ����¼�������ֵĺڵ���������ߵĿ��ȣ������ÿ��ȴ�����ֵBLACK_WIDTH����
     ** ��ȷ������һ�����ߡ�����ɨ��һ�У����������㹻��������ĺ��ߣ������趨����ֵBLACK_NUM��������Ϊ��һ�з��ϰ����ߡ����ո����ۻ�
     ** ���������е�ɨ�裬���������������㹻�ࣨ�����趨����ֵBLACK_TIMES��������Ϊ��һ��ͼƬ�д��ڰ����ߣ�����·����������
     **/

    int row,cloum;          //��,��
    int Black_width=0;      //�̶��У�����ɨ���Ǽ�¼ÿ�κڵ�ĸ�������һ�����ߵĿ��ȣ�
    int Black_num=0;        //��¼�к��ߵ���������Ϊ�жϸ����Ƿ�Ϊ�����ߵ�����
    int Black_times=0;      //��¼��������ߵ�����������Ϊ�жϸ�·���Ƿ�Ϊ�����ߵ�����

    Point InflectionL, InflectionR; //�¹յ�
    InflectionL.X=0;
    InflectionL.Y=0;
    InflectionR.X=0;
    InflectionR.Y=0;
    GetDownInflection(0,MT9V03X_H,LeftLine,RightLine,&InflectionL,&InflectionR);    //��ȡ�¹յ�


    if(InflectionL.X!=0&&InflectionL.Y!=0)    //�յ㣨���⣩�����
    {
        for(row=InflectionL.X,cloum=InflectionL.Y;row<MT9V03X_H;row++)        //����յ㿪ʼ�̶��У���������ɨ
        {
            if(BinaryImage[row][cloum]==IMAGE_BLACK)    //�ҵ���һ���ڵ�
            {
                for(;cloum<MT9V03X_W;cloum++)                                 //�̶��У���������ɨ
                {
                    if(BinaryImage[row][cloum]==IMAGE_BLACK)    //ɨ���ڵ�
                    {
                        Black_width++;   //���߿���+1
                    }
                    else                                        //ɨ���׵�
                    {
                        if(Black_width>=S_BLACK_WIDTH)    //�жϺ��߿����Ƿ�������ֵ
                        {
                            Black_num++; //�к�������+1
                        }
                        Black_width=0;   //��һ�ΰ׵��жϺ����ú��߿���
                    }
                    if(Black_num>=S_BLACK_NUM)    //����������ߵ���ֵ����һ�У�
                    {
                        Black_times++;  //��������ߵ�����+1
                        break;
                    }
                }
                Black_num=0;    //��һ�е�ɨ�߽����������к�����
            }
            if(Black_times>=S_BLACK_TIMES)    //�����������·�ε���ֵ
            {
                return 1;
            }
        }
        return 0;
    }

    if(InflectionR.X!=0&&InflectionR.Y!=0)    //�յ㣨���⣩���ұ�
    {
        for(row=InflectionL.X,cloum=InflectionL.Y;row<MT9V03X_H;row++)        //���ҹյ㿪ʼ�̶��У���������ɨ
        {
            if(BinaryImage[row][cloum]==IMAGE_BLACK)    //�ҵ���һ���ڵ�
            {
                for(;cloum>0;cloum--)                                         //�̶��У���������ɨ
                {
                    if(BinaryImage[row][cloum]==IMAGE_BLACK)    //ɨ���ڵ�
                    {
                        Black_width++;   //���߿���+1
                    }
                    else                                        //ɨ���׵�
                    {
                        if(Black_width>=S_BLACK_WIDTH)    //�жϺ��߿����Ƿ�������ֵ
                        {
                            Black_num++; //�к�������+1
                        }
                        Black_width=0;   //��һ�ΰ׵��жϺ����ú��߿���
                    }
                    if(Black_num>=S_BLACK_NUM)    //����������ߵ���ֵ����һ�У�
                    {
                        Black_times++;  //��������ߵ�����+1
                        break;
                    }
                }
                Black_num=0;    //��һ�е�ɨ�߽����������к�����
            }
            if(Black_times>=S_BLACK_TIMES)    //�����������·�ε���ֵ
            {
                return 2;
            }
        }
        return 0;
    }

    return 0;
}

/*
 *******************************************************************************************
 ** ��������: ʶ�𻷵���û�е��ﻷ�����
 ** ��    ��: LeftLine����������
 **           RightLine����������
 **           InflectionL����յ�
 **           InflectionR���ҹյ�
 ** �� �� ֵ: 0��û��ʶ�𵽻���
 **           1��ʶ�𵽻������ڳ������
 **           2��ʶ�𵽻������ڳ����Ҳ�
 ** ��    ��: WBN
 ** ע    �⣺������ʶ���Ϊ�����֣�һ��ʶ�𵽻�������Ϊ������ڣ�����ʶ���˻������
 ********************************************************************************************
 */
uint8 CircleIsland_Begin(int LeftLine,int RightLine,Point InflectionL,Point InflectionR)
{
    /*
     ** ������ܵ�����Ƚ϶࣬�Ƚ��Ѵ�����������д
     ** ��ʶ��ǰ���л���ʱ����Ҫ�ǶԻ�������һ�����εĲ��ߣ����⳵������Ϊ������ս������ĳ���
     * */

    if(InflectionL.X!=0&&InflectionL.Y!=0)    //�յ㣨�����������
    {

    }
    return 0;
}

/*
 *******************************************************************************************
 ** ��������: ʶ�𻷵����
 ** ��    ��: LeftLine����������
 **           RightLine����������
 **           InflectionL����յ�
 **           InflectionR���ҹյ�
 ** �� �� ֵ: 0��û��ʶ�𵽻���
 **           1��ʶ�𵽻���������ڳ������
 **           2��ʶ�𵽻���������ڳ����Ҳ�
 ** ��    ��: WBN
 ** ע    �⣺�ú����ĵ���Ӧ����ʶ�𵽻���֮��ʼ��Flag_CircleBegin==1����
 **           ���ڽ��뻷����رգ�������ֵΪ��0����Flag_CircleBegin=0��
 ********************************************************************************************
 */
uint8 CircleIsland_In(int LeftLine,int RightLine,Point InflectionL,Point InflectionR)
{
    /*
     **     ������ֻȡһ����������Ӹպ��ڻ���Բ��������λ�ã��������Ѿ�����ͼ���У�ͼ�����е�ֻ�ǻ��������
     **     ���ڸ���������жϷ�ʽ��ͼ�������һ��ֱ�к�һ�������·����ô�ͻ����һ�������������ߣ���һ�߳��ֶ��ߣ�ת��·�������߶��ߣ�
     **     �ڶ����жϷ�ʽ��һ�߶���һ�߲����ߣ�������д�������жϷ�ʽ��ŵĴ���
     * */

    int row,cloum;              //��,��
    int upline[MT9V03X_W]={0};  //�±߽���
    //����һ
    if(InflectionL.X!=0&&InflectionL.Y!=0)    //�յ㣨������ڣ������
    {
        for(cloum=InflectionL.Y;cloum>0;cloum--)    //�ӹյ������꿪ʼ��������ɨ
        {
            for(row=InflectionL.X;row<MT9V03X_H;row++)   //�ӹյ������꿪ʼ��������ɨ
            {
                if(BinaryImage[row][cloum]==IMAGE_WHITE)    //ɨ���׵�
                {
                    upline[cloum-InflectionL.Y]=row;    //��¼�ϱ߽��ߣ������ݵ�λ��ΪX�ᣬ���ݵ�ֵΪY�ửͼ
                    break;
                }
            }
        }
        int left=0,mid=0,right=0;   //���߽��߷�Ϊ���������Σ��ֱ���������εľ�ֵ
        for(cloum=InflectionL.Y;cloum>2*InflectionL.Y/3;cloum--)    //��
        {
            right+=upline[cloum];
        }
        for(;cloum>InflectionL.Y/3;cloum--)                         //��
        {
            mid+=upline[cloum];
        }
        for(;cloum>0;cloum--)                                       //��
        {
            left+=upline[cloum];
        }
        if(left<mid&&right<mid)     //�м�����������ߣ��γ�һ�λ���
        {
            return 1;
        }
    }
    //������
    if(InflectionR.X!=0&&InflectionR.Y!=0)    //�յ㣨������ڣ����ұ�
    {
        if(LostNum_LeftLine<C_LOST1&&LostNum_RightLine>C_LOST2) //������С����ֵ���Ҷ�����������ֵ
        {
            return 2;
        }
    }
    return 0;
}

/********************************************************************************************
 ** ��������: ʶ������
 ** ��    ��: int startline:�û���������ʼ��
 **           int endline:�û������Ľ����У���ʾ��ǰ���ε�ʶ�𣬸����ٶȲ�ͬ���е�����
 **           int *LeftLine������
 **           int *RightLine:����
 **           Point *InflectionL:��߹յ�
 **           Point *InflectionR:�ұ߹յ�
 **           Point *InflectionC:�м�յ�
 ** �� �� ֵ: 0��û��ʶ�𵽻���
 **           1��ʶ������
 ** ��    ��: LJF
 ** ע    �⣺1 . Ŀǰ����������������ʱ��ĺ�������Ϊ����ǰ�涼���и�������Ի���ֳ���б���������ʱ�����ҹյ㲢��һ��������
 **           2.����ǽ������ĺ�����������ʱ��Ӧ����дһ�������ڽ����������ٿ������������ж�
 *********************************************************************************************/
uint8 ForkIdentify(int startline,int endline,int *LeftLine,int *RightLine,Point *InflectionL,Point *InflectionR,Point *InflectionC)
{
    GetDownInflection(startline, endline, LeftLine, RightLine, InflectionL, InflectionR);//��ȡ���ҹյ�
    if(InflectionL->X!=0&&InflectionR->X!=0)//�����ҹյ����
    {
        GetForkUpInflection(*InflectionL, *InflectionR, InflectionC);//ȥ�����Ϲյ�
        if(InflectionC->X!=0)
        {
            //�����������Ϲյ㻹���������������жϣ����粻����������ʲô�ģ����Ҫ��������ж���
            //��������������̫���˻�������У������ʮ�ֳ�����ʱ��Ϳ��ܻ�����ͬ���ľ���������
            //�����������Ԫ�ػ����ԭ��,��һ����ʮ�ֵı�־
            return 1;//�����յ������������
            /*����Ҫд��������ѡ���ʶ����ѡ���������ұߴӶ����в���*/
        }
    }
    return 0;
}