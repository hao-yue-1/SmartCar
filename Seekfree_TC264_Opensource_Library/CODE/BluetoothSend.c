/*
 * BluetoothSend.c
 *
 *  Created on: 2022年3月6日
 *      Author: 30516
 */

#include "BluetoothSend.h"

//int16转为uint8
void ShortToByte(int16 x,uint8 *y)
{
    y[1]=(x&0xFF00)>>8;
    y[0]=(x&0xFF);
}

void Float_to_Byte(float f,unsigned char *byte)
{

    unsigned long longdata = 0;
    longdata = *(unsigned long*)&f;
    byte[3] = (longdata & 0xFF000000) >> 24;
    byte[2] = (longdata & 0x00FF0000) >> 16;
    byte[1] = (longdata & 0x0000FF00) >> 8;
    byte[0] = (longdata & 0x000000FF);

}

void BluetooothSendBias(float Bias)
{
    uint8 uart_tx_buf[7];                       //数据包
    uart_tx_buf[0]=0xA5;                        //包头
    Float_to_Byte(Bias,&uart_tx_buf[1]);        //原数据
    //计算校验和
    for(int cnt_tx=1;cnt_tx<=7-3;cnt_tx++)
        uart_tx_buf[7-2] += uart_tx_buf[cnt_tx];
    uart_tx_buf[7-2] = uart_tx_buf[7-2]&0xff;
//    uart_tx_buf[5]=(uint8)Bias;                 //计算校验和
    uart_tx_buf[6]=0x5A;                        //包尾

    uart_putbuff(UART_2,uart_tx_buf,7);         //通过串口2发送数据到蓝牙
}

void BluetoothSendToApp(int16 EncoderL,int16 EncoderR)
{
    uint8 uart_tx_buf[7];                       //数据包
    uart_tx_buf[0]=0xA5;                        //包头
    ShortToByte(EncoderL,&uart_tx_buf[1]);      //原数据
    ShortToByte(EncoderR,&uart_tx_buf[3]);      //原数据
    uart_tx_buf[5]=(uint8)(EncoderL+EncoderR);  //校验和
    uart_tx_buf[6]=0x5A;                        //包尾

    uart_putbuff(UART_2,uart_tx_buf,7);         //通过串口2发送数据到蓝牙
}


