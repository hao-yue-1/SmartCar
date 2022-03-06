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

void BluetoothReceiveFromApp(int P,int I)
{
    uint8 uart_rx_buf[11];                       //数据包
    uint8 err;                                   //校验和
    if(uart_query(UART_2,uart_rx_buf)==1)        //接收到数据
    {
        if(uart_rx_buf[0]==0xA5&&uart_rx_buf[10]==0x5A)
        {
            //检查校验和
            err = ((uint8)(uart_rx_buf[1]+uart_rx_buf[2]+uart_rx_buf[3]+uart_rx_buf[4]+uart_rx_buf[5]+uart_rx_buf[6]+uart_rx_buf[7]+uart_rx_buf[8])&0xFF);
            if(err!=uart_rx_buf[9])
            {
                return; //校验和错误，直接return
            }
            //校验和正确，赋值操作
            P=(int)uart_rx_buf[1];
            I=(int)uart_rx_buf[5];

        }
    }
}

