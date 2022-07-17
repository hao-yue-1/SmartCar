#include "oled.h"
#include "myiic.h"
#include "oled_font.h"

const unsigned char OLED_init_cmd[25]=
{
  /*0xae,0X00,0X10,0x40,0X81,0XCF,0xff,0xa1,0xa4,
  0xA6,0xc8,0xa8,0x3F,0xd5,0x80,0xd3,0x00,0XDA,0X12,
  0x8d,0x14,0xdb,0x40,0X20,0X02,0xd9,0xf1,0xAF*/
       0xAE,//关闭显示
       0xD5,//设置时钟分频因子,震荡频率
       0x80,  //[3:0],分频因子;[7:4],震荡频率

       0xA8,//设置驱动路数
       0X3F,//默认0X3F(1/64)
       0xD3,//设置显示偏移
       0X00,//默认为0
       0x40,//设置显示开始行 [5:0],行数.                              
       0x8D,//电荷泵设置
       0x14,//bit2，开启/关闭
       0x20,//设置内存地址模式
       0x02,//[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
       0xA1,//段重定义设置,bit0:0,0->0;1,0->127;
       0xC8,//设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
       0xDA,//设置COM硬件引脚配置
       0x12,//[5:4]配置            
       0x81,//对比度设置
       0xEF,//1~255;默认0X7F (亮度设置,越大越亮)
       0xD9,//设置预充电周期
       0xf1,//[3:0],PHASE 1;[7:4],PHASE 2;
       0xDB,//设置VCOMH 电压倍率
       0x30,//[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;
       0xA4,//全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
       0xA6,//设置显示方式;bit0:1,反相显示;0,正常显示        
       0xAF,//开启显示     
};

//写命令
void OLED_send_cmd(unsigned char o_command)
{
	IIC_Start();
    IIC_Send_Byte(0x78);//OLED地址
    IIC_Wait_Ack();
    IIC_Send_Byte(0x00);//寄存器地址
    IIC_Wait_Ack();
    IIC_Send_Byte(o_command);
    IIC_Wait_Ack();
    IIC_Stop();
}
//写数据
void OLED_send_data(unsigned char o_data)
{ 
	IIC_Start();
    IIC_Send_Byte(0x78);//OLED地址
    IIC_Wait_Ack();
    IIC_Send_Byte(0x40);//寄存器地址
    IIC_Wait_Ack();
    IIC_Send_Byte(o_data);
    IIC_Wait_Ack();
    IIC_Stop();
}

void Column_set(unsigned char column)
{
	OLED_send_cmd(0x10|(column>>4));    //设置列地址高位
	OLED_send_cmd(0x00|(column&0x0f));  //设置列地址低位   
}
void Page_set(unsigned char page)
{
	OLED_send_cmd(0xb0+page);
}

/*
*********************************************************************************************************
*	函 数 名: OLED_clear
*	功能说明: 清屏整个OLED的显示
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void OLED_clear(void)
{
	unsigned char page,column;
	for(page=0;page<8;page++)             //page loop
	{ 
		Page_set(page);
		Column_set(0);	  
		for(column=0;column<128;column++)	//column loop
		{
			OLED_send_data(0x00);
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: OLED_full
*	功能说明: 填充整个OLED的显示
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void OLED_full(void)
{
	unsigned char page,column;
	for(page=0;page<8;page++)             //page loop
	{ 
		Page_set(page);
		Column_set(0);	  
		for(column=0;column<128;column++)	//column loop
		{
			OLED_send_data(0xff);
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: OLED_init
*	功能说明: 初始化OLED
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void OLED_init(void)
{
	unsigned char i;
	IIC_Init();		//初始化IIC
	for(i=0;i<25;i++)
	{
		OLED_send_cmd(OLED_init_cmd[i]);
	}
	OLED_clear();   //清屏OLED
}

/*
*********************************************************************************************************
*	函 数 名: Picture_display
*	功能说明: 显示图片（128*64）
*	形    参: *ptr_pic：图片数组的头指针
*	返 回 值: 无
*********************************************************************************************************
*/
void Picture_display(const unsigned char *ptr_pic)
{
	unsigned char page,column;
	for(page=0;page<(64/8);page++)        //page loop
	{ 
		Page_set(page);
		Column_set(0);	  
		for(column=0;column<128;column++)	//column loop
		{
			OLED_send_data(*ptr_pic++);
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: Picture_ReverseDisplay
*	功能说明: 反转的形式显示图片（128*64）
*	形    参: *ptr_pic：图片数组的头指针
*	返 回 值: 无
*********************************************************************************************************
*/
void Picture_ReverseDisplay(const unsigned char *ptr_pic)
{
	unsigned char page,column,data;
	for(page=0;page<(64/8);page++)        //page loop
	{ 
		Page_set(page);
		Column_set(0);	  
		for(column=0;column<128;column++)	//column loop
		{
			data=*ptr_pic++;
			data=~data;
			OLED_send_data(data);
		}
	}
}

//设置光标位置
void OLED_SetPos(unsigned char x, unsigned char y)
{ 
    OLED_send_cmd(0xb0+y);
    OLED_send_cmd(((x&0xf0)>>4)|0x10);
    OLED_send_cmd((x&0x0f)|0x01);
}

/*
*********************************************************************************************************
*	函 数 名: OLED_ShowStr
*	功能说明: 显示字符串（在字库中的字符）
*	形    参: 1. x,y：起始显示坐标		//y=0,2,4,6（TextSize=2）  y=0,1,2,3,4,5,6,7（TextSize=1）
			  2. ch[]：字符串    
			  3. TextSize：字符大小（1:6*8  2:8*16）
*	返 回 值: 无
*********************************************************************************************************
*/
void OLED_ShowStr(unsigned char x, unsigned char y, char ch[], unsigned char TextSize)
{
    unsigned char c = 0,i = 0,j = 0;
    switch(TextSize)
    {
        case 1:
        {
            while(ch[j] != '\0')
            {
                c = ch[j] - 32;
                if(x > 126)
                {
                    x = 0;
                    y++;
                }
                OLED_SetPos(x,y);
                for(i=0;i<6;i++)
                    OLED_send_data(F6x8[c][i]);
                x += 6;
                j++;
            }
            break;
        }
        case 2:
        {
            while(ch[j] != '\0')
            {
                c = ch[j] - 32;
                if(x > 120)
                {
                    x = 0;
                    y++;
                }
                OLED_SetPos(x,y);
                for(i=0;i<8;i++)
                    OLED_send_data(F8X16[c*16+i]);
                OLED_SetPos(x,y+1);
                for(i=0;i<8;i++)
                    OLED_send_data(F8X16[c*16+i+8]);
                x += 8;
                j++;
            }
            break;
        }
        default: break;
    }
}

/*
*********************************************************************************************************
*	函 数 名: OLED_ShowCh
*	功能说明: 显示一个字符（在字库中的字符）
*	形    参: 1. x,y：起始显示坐标		//y=0,2,4,6（TextSize=2）  y=0,1,2,3,4,5,6,7（TextSize=1）
			  2. ch：字符
			  3. TextSize：字符大小（1:6*8  2:8*16）
*	返 回 值: 无
*********************************************************************************************************
*/
void OLED_ShowCh(unsigned char x,unsigned char y,unsigned char ch,unsigned char TextSize)
{
    unsigned char c=0,i=0;
    c=ch-' ';//得到偏移后的值
    switch (TextSize)
    {
        case 1:
        {
            OLED_SetPos(x,y);
            for(i=0;i<6;i++)
            {
                OLED_send_data(F6x8[c][i]);
            }
            break;
        }
        case 2:
        {
            OLED_SetPos(x,y);
            for(i=0;i<8;i++)
                OLED_send_data(F8X16[c*16+i]);
            OLED_SetPos(x,y+1);
            for(i=0;i<8;i++)
            {
                OLED_send_data(F8X16[c*16+i+8]);
            }
            break;
        }
        default: break;
    }
}

//m^n函数
int oled_pow(unsigned char m,unsigned char n)
{
    int result=1;
    while(n--)result*=m;
    return result;
}

/*
*********************************************************************************************************
*	函 数 名: OLED_ShowNum
*	功能说明: 显示一个int型整数
*	形    参: 1. x,y：起始显示坐标		//y=0,2,4,6（TextSize=2）  y=0,1,2,3,4,5,6,7（TextSize=1）
			  2. num：整数(0~4294967295)
			  3. len：整数长度
			  4. TextSize：字符大小（1:6*8  2:8*16）
*	返 回 值: 无
*   注    意: 整数的显示方式采用右对齐的方式，即len大于实际的num的长度时，num会靠右显示，空出左边
*********************************************************************************************************
*/
void OLED_ShowNum(unsigned char x,unsigned char y,int num,unsigned char len,unsigned char TextSize)
{
    unsigned char t,temp;
    unsigned char enshow=0;
    switch (TextSize)
    {
        case 1:
        {
            for(t=0;t<len;t++)
            {
                temp=(num/oled_pow(10,len-t-1))%10;
                if(enshow==0&&t<(len-1))
                {
                    if(temp==0)
                    {
                        OLED_ShowCh(x+6*t,y,' ',TextSize);
                        continue;
                    }
                    else
                        enshow=1;

                }
                OLED_ShowCh(x+6*t,y,temp+'0',TextSize);
            }
            break;
        }
        case 2:
        {
            for(t=0;t<len;t++)
            {
                temp=(num/oled_pow(10,len-t-1))%10;
                if(enshow==0&&t<(len-1))
                {
                    if(temp==0)
                    {
                        OLED_ShowCh(x+8*t,y,' ',TextSize);
                        continue;
                    }
                    else
                        enshow=1;

                }
                OLED_ShowCh(x+8*t,y,temp+'0',TextSize);
            }
            break;
        }
        default : break;
    }
}

/*
*********************************************************************************************************
*	函 数 名: OLED_ShowFloat
*	功能说明: 显示一个double型小数
*	形    参: 1. x,y：起始显示坐标		//y=0,1,2,3,4,5,6,7（TextSize=1）
             2. num：double型小数
			 3. N：小数点后位数
*	返 回 值: 无
*   注    意: 该函数是抄来的不完美的函数，主要是不能自己设置字体的大小，只能以6*8的大小显示    //TextSize=1
*********************************************************************************************************
*/
void OLED_ShowFloat(unsigned char x,unsigned char y,double num,unsigned char N)
{
    unsigned char   i_Count=1;
    unsigned char   n[12]={0};
    long   j=1;
    int    real_int=0;
    double decimal=0;
    unsigned int   real_decimal=0;
    if(num<0)
    {
        real_int=(int)(-num);
    }
    else
    {
        real_int=(int)num;
    }
    decimal=num-real_int;
    real_decimal=decimal*1e4;
    while(real_int/10/j!=0)
    {
        j=j*10;i_Count++;
    }
    n[0]=(real_int/10000)%10;
    n[1]=(real_int/1000)%10;
    n[2]=(real_int/100)%10;
    n[3]=(real_int/10)%10;
    n[4]=(real_int/1)%10;
    n[5]='.';
    n[6]=(real_decimal/1000)%10;
    n[7]=(real_decimal/100)%10;
    n[8]=(real_decimal/10)%10;
    n[9]=real_decimal%10;
    n[6+N]='\0';
    for(j=0;j<10;j++) n[j]=n[j]+16+32;
    if(num<0)
    {
        i_Count+=1;
        n[5-i_Count]='-';
    }
    n[5]='.';
    n[6+N]='\0';
    OLED_ShowStr(x,y,&n[5-i_Count],1);
}
