/*
 * ImageSpecial.h
 *
 *  Created on: 2022��1��17��
 *      Author: yue
 */

#ifndef CODE_IMAGESPECIAL_H_
#define CODE_IMAGESPECIAL_H_

#include "Binarization.h"       //��ֵ��֮���ͼ������
#include "SEEKFREE_MT9V03X.h"   //Ϊ��Ҫuint8���ֶ���,��ֵ���㷨�е�ĳЩ��ѧ����,����ͷͼ���ȫ�ֱ���
#include "ImageBasic.h"         //��ȡͼ���������֮�������

//������ʶ��
#define S_BLACK_WIDTH  3    //�����ߺ��߿�����ֵ //�����жϸ����Ƿ�Ϊ�����ߺ���
#define S_BLACK_NUM    8    //�����ߺ���������ֵ //�����жϸ����Ƿ�Ϊ������
#define S_BLACK_TIMES  3    //������������ֵ     //�����жϸ�·���Ƿ�Ϊ������
//����ʶ��
#define C_LOST1 5           //������һ�ߵĶ�������ֵ
#define C_LOST2 20          //����һ�ߵĶ�������ֵ
//�����ж�flag
extern uint8 Flag_CircleBegin;   //���ֻ���
extern uint8 Flag_CircleIn;      //�������

uint8 StartLineFlag(int *LeftLine,int *RightLine);      //������ʶ��
uint8 CircleIsland_Begin(int LeftLine,int RightLine,Point InflectionL,Point InflectionR);
uint8 CircleIsland_Begin(int LeftLine,int RightLine,Point InflectionL,Point InflectionR);
uint8 ForkIdentify(int startline,int endline,int *LeftLine,int *RightLine,Point *InflectionL,Point *InflectionR,Point *InflectionC);//����ʶ��

#endif /* CODE_IMAGESPECIAL_H_ */