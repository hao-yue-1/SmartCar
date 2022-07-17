#ifndef _OLED_H
#define _OLED_H

//////////////////////////////////
//0.96寸的OLED驱动函数
//////////////////////////////////

void OLED_send_cmd(unsigned char o_command);	//写命令
void OLED_send_data(unsigned char o_data);		//写数据
void OLED_clear(void);			//清屏OLED
void OLED_full(void);			//填充整个OLED
void OLED_init(void);			//初始化OLED
void Picture_display(const unsigned char *ptr_pic);
void Picture_ReverseDisplay(const unsigned char *ptr_pic);
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);

void OLED_ShowNum(unsigned char x,unsigned char y,int num,unsigned char len,unsigned char TextSize);
void OLED_ShowCh(unsigned char x,unsigned char y,unsigned char ch,unsigned char TextSize);
void OLED_ShowFloat(unsigned char x,unsigned char y,double num,unsigned char N);

#endif
