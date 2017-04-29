#include<reg51.h>			//Large编译，否则xdata异常
#include<math.h>
#include<LOLI_3.h>  	 //显示需要的图形数据
#define u8 unsigned char
#define u16 unsigned int


/*****************************************************/
//STC12 系列新增寄存器
sfr IPH=0xb7;
sfr P4=0xc0;
sfr P4SW=0xbb;
sfr P1ASF=0x9d;
sfr P4M0=0xb4;
sfr P3M0=0xb2;
sfr P3M1=0xb1;
sfr P2M0=0x96;
sfr P0M0=0x94;
sfr ADC_CONTR=0xbc;
sfr ADC_RES=0xbd;
sfr ADC_RESL=0xbe;

sfr BRT=0x9c;
sfr AUXR=0x8e;
sfr WAKE_CLKO=0x8f;

sfr IAP_DATA=0xc2;
sfr IAP_ADDRH=0xc3;
sfr IAP_ADDRL=0xc4;
sfr IAP_CMD=0xc5;
sfr IAP_TRIG=0xc6;
sfr IAP_CONTR=0xc7;

sfr CCON        =   0xD8;           //PCA control register
sbit CCF0       =   CCON^0;         //PCA module-0 interrupt flag
sbit CCF1       =   CCON^1;         //PCA module-1 interrupt flag
sbit CR         =   CCON^6;         //PCA timer run control bit
sfr CMOD        =   0xD9;           //PCA mode register
sfr CL          =   0xE9;           //PCA base timer LOW
sfr CH          =   0xF9;           //PCA base timer HIGH
sfr CCAPM0      =   0xDA;           //PCA module-0 mode register
sfr CCAP0L      =   0xEA;           //PCA module-0 capture register LOW
sfr CCAP0H      =   0xFA;           //PCA module-0 capture register HIGH
sfr CCAPM1      =   0xDB;           //PCA module-1 mode registers
sfr CCAP1L      =   0xEB;           //PCA module-1 capture register LOW
sfr CCAP1H      =   0xFB;           //PCA module-1 capture register HIGH


/*************************************************/
//LCD引脚定义
sbit CS=P0^6;
sbit RST=P0^7;
sbit RS=P4^6;								  
sbit SDA=P4^5;							   
sbit CLK=P4^4;
sbit LEDA=P2^5;

/**************************************************/
//NRF24L01 引脚定义

sbit CE=P0^0;
sbit SCK=P0^1;
sbit MDO=P0^2;
sbit IRQ=P0^3;
sbit MDI=P0^4;
sbit CSN=P0^5;

/*************************************************/
//I/O定义

sbit buzzer=P3^4;
sbit PPM=P3^3;

sbit KEY=P4^7;
sbit K1=P1^1;//30uart
sbit K2=P1^2;//31
sbit K3=P3^7;
sbit K4=P3^6;
sbit K5=P3^2;
sbit K6=P3^5;

sbit SA=P2^0;
sbit SB1=P2^1;
sbit SB2=P2^2;
sbit SC1=P2^3;
sbit SC2=P2^4;
sbit SD=P2^6;
sbit SE=P2^7;


/***********************************************/
//数据、标志位
u8 bdata DATA;
sbit D_7=DATA^7;
sbit D_6=DATA^6;
sbit D_5=DATA^5;
sbit D_4=DATA^4;
sbit D_3=DATA^3;
sbit D_2=DATA^2;
sbit D_1=DATA^1;
sbit D_0=DATA^0;



//ROM存储
u8 Model_number=0,Model_current=0,Model_target=0;
bit Model_change,Model_reset,reset_action;
u8 TEMP_DATA[300];
bit ROM_enable;
bit Data_system,Data_model;
u8 ROM_time=0;
u8 COPY_DATA[60];
bit copy_ready,paste_ready;

bit calibration_action,calibration_ok;

//串口数据
unsigned char iflycount=0,flydata=0;
int adc1 =0,adc2=0,adc3=0,adc4=0,adc11=0 ,adc22=0,adc33=0,adc44=0;
//显示
u8 word[10];
u8 WW[10][6];


//时钟
bit event1,event2,event3;
u8 t1=0,t2=0,t3=0;
u8 connect_time;
bit flag_20ms;

u8 beep=0,sound=0,clock_set=0;
u8	timer,second=0,minute=0,hour=0;

//菜单模式
bit PPM_OUT,PPM_IN,RC_TX,IR_TX,RC_check;
bit config_change,star=1,RC_check_star;
u8 menu=0,menu_1=0,menu_2=0,line=0,line_1=0,line_2=0;

//按键
bit KS;
bit key1,key2,key3,key4,key5,key6;
bit s1,s2,s3,s4,s5,s6;
bit s1_long_press,s2_long_press,s3_long_press,s4_long_press,s5_long_press,s6_long_press;
u8 s1_press_time,s2_press_time,s3_press_time,s4_press_time,s5_press_time,s6_press_time;
bit key_time;
bit increase,decrease,up,down,left,right;



//硬件配置
u8 LCD_vop=42,LCD_light=1;
u8 Sound_switch=0,model;
u8 Battery_1s=1,VLD1=35,VLD2=70;
u16 battery_voltage;
u8 tt=0,t_adc=0;
u16 data adc[8];

u16 value0,value1;

bit NRF_change;
u8 TX_power=3;
char out_control[8]={0,0,-100,0,0,0,0,0};
u16 out_control_data[8];

u8 IN_mapping[8]={0,1,2,3,4,5,6,7,};

bit recover;

bit NRF_config_change,config_change;
bit first;

u16 CH1_max=600,CH2_max=600,CH3_max=600,CH4_max=600;
u16 CH1_min=400,CH2_min=400,CH3_min=400,CH4_min=400;
u8 CH1_offset=51,CH2_offset=51,CH3_offset=51,CH4_offset=51;
u16 CH_value[4];




//无线通信
bit not_get,connecting;

bit CD;
u8 data RC_TX_state=0;
u8 t_check,t_hopping,NRF_error=0;
u8 CD_data[125];
u8 NRF_mapping[8]={0,1,2,3,4,5,6,7};
u16 NRF_buff[8];
u8 timer2=0,timer3=0,receive=0;


u8 rx_sum,FUCK_sum=0;//坑爹的xdata，定义成rx不能用，定义成FUCK就能用了


u8 address[5];
u8 hopping[5];
u8 address_0[5]={'L','O','V','E','!'};//使用LOVE作为对频暗语



u8 tx_buff[11],tx_data[11],rx[11]={0,0,0};
u8 tx_order[11];


u8 TX_protocol=3;





//PPM

u8 T_l,T_h;
u8 t_ppm=0,t_PPMout=0;
int PPM_buff[8],PPM_DATA[8]={0,0,0,0,0,0,0,0};
u16  PPMout[8]={500,600,800,200,1000,0,100,300};
u8 PPM_mapping[8]={0,1,2,3,4,5,6,7};

u8 train_switch=0;
bit train_output;
u8 PPM_updata=0;


//自动控制
u8 auto_switch=0,auto_mode=2;
char auto_middle=-100,auto_star=-50,auto_end=50;
u8 auto_speed=10,auto_mapping=7;
int auto_value=0;
u16 auto_max,auto_min;
bit auto_h,auto_l=0;




//通道运算
u8 Switch_A,Switch_B,Switch_C,Switch_D;

int operation[8];
bit RC_data_ready;


char CH_middle[8]={0,0,0,0,0,0,0,0};
u8 CH_ratio1[8]={100,100,100,100,100,100,100,100};
u8 CH_ratio2[8]={100,100,100,100,100,100,100,100};
u8 CH_negative[8]={0,0,0,0,0,0,0,0};
u8 CH_middle_change=0;


u8 mixer_name;
bit mixer1,mixer2,mixer3,mixer4;
u8 mixer1_same=1,mixer1_contrary=0;
u8 mixer2_same=2,mixer2_contrary=3;
u8 mixer3_same=3,mixer3_contrary=0;
u8 mixer4_same=5,mixer4_contrary=0;




/***********************************************************/
//延时函数

void Delay1ms()		//@12.000MHz
{
	unsigned char i, j;
	i = 12;
	j = 168;
	do
	{
		while (--j);
	} while (--i);
}

void delay_ms(u16 i)
{
	while(i--)
	Delay1ms();
}

/**********************************************************/
//ROM操作函数、数据读写控制

void Data_change(u8 m)
{
	if(m==1)Data_system=1;
	if(m==2)Data_model=1;
	
	event3=1,t3=0;//刷新事件3定时器
	
}

void Model_data_reset()
{
	CH_middle[0]=0;
		CH_middle[1]=0;
		CH_middle[2]=0;
		CH_middle[3]=0;
		CH_middle[4]=0;
		CH_middle[5]=0;
		CH_middle[6]=0;
		CH_middle[7]=0;
		CH_ratio1[0]=100;
		CH_ratio1[1]=100;
		CH_ratio1[2]=100;
		CH_ratio1[3]=100;
		CH_ratio1[4]=100;
		CH_ratio1[5]=100;
		CH_ratio1[6]=100;
		CH_ratio1[7]=100;
		CH_ratio2[0]=100;
		CH_ratio2[1]=100;
		CH_ratio2[2]=100;
		CH_ratio2[3]=100;
		CH_ratio2[4]=100;
		CH_ratio2[5]=100;
		CH_ratio2[6]=100;
		CH_ratio2[7]=100;			
		CH_negative[0]=0;
		CH_negative[1]=0;
		CH_negative[2]=0;
		CH_negative[3]=0;
		CH_negative[4]=0;
		CH_negative[5]=0;
		CH_negative[6]=0;
		CH_negative[7]=0;
		mixer1=0;
		mixer2=0;
		mixer3=0;
		mixer4=0;
		mixer1_same=1;
		mixer1_contrary=0;
		mixer2_same=2;
		mixer2_contrary=3;
		mixer3_same=3;
		mixer3_contrary=0;
		mixer4_same=5;
		mixer4_contrary=0;
		
		IN_mapping[0]=0;
		IN_mapping[1]=1;
		IN_mapping[2]=2;
		IN_mapping[3]=3;
		IN_mapping[4]=4;
		IN_mapping[5]=5;
		IN_mapping[6]=6;
		IN_mapping[7]=7;
}

u8 Model_adress(u8 num)
{
	if(num==0)num=0;
	else if(num==1)num=54;
	else if(num==2)num=108;
	else if(num==3)num=162;
	else num=206;
	return num;
}

void Model_copy()
{
	u16 i,base;
	base=Model_adress(Model_number);
	
	for(i=0;i<54;i++)
	{
		COPY_DATA[i]=TEMP_DATA[i+base];
	}
	

	base=Model_adress(Model_target);
	
	for(i=0;i<54;i++)
	{
		TEMP_DATA[i+base]=COPY_DATA[i];
	}
	
}

u8 EEPROM_read(u8 address_H,u16 address_L)
{
	IAP_CMD=0x01;
	if(address_L>255)
	{
		IAP_ADDRH=address_H+1;
		IAP_ADDRL=address_L-255;	
	}
	else 
	{
		IAP_ADDRH=address_H;
		IAP_ADDRL=address_L;	
	}
	
	IAP_TRIG=0x5a;			//EEPROM读写密码，不同系列单片机不同
	IAP_TRIG=0xa5; 
	return IAP_DATA;
}

void EEPROM_write(u8 address_H,u16 address_L,u8 byte)
{
	IAP_CMD=0x02;
	IAP_DATA=byte;
	if(address_L>255)
	{
		IAP_ADDRH=address_H+1;
		IAP_ADDRL=address_L-255;	
	}
	else 
	{
		IAP_ADDRH=address_H;
		IAP_ADDRL=address_L;	
	}
	IAP_ADDRH=address_H;
	IAP_ADDRL=address_L;
	IAP_TRIG=0x5a;
	IAP_TRIG=0xa5; 
	
}

void EEPROM_clean(u8 address_H)
{
	IAP_CMD=0x03;
	IAP_ADDRH=address_H;
	IAP_ADDRL=0;
	IAP_TRIG=0x5a;
	IAP_TRIG=0xa5;
}

void DATA_read1()
{								
	IAP_CONTR=0x82;
	if(EEPROM_read(0,0)!=0xa0)	 
	{
		first=1;
		
		
		RC_TX=1;PPM_OUT=1;

	}
	else
	{
//		system=EEPROM_read(0,1);
		Battery_1s=EEPROM_read(0,2);
		LCD_vop=EEPROM_read(0,3);
		LCD_light=EEPROM_read(0,4);
		CH1_offset=EEPROM_read(0,5);
		CH2_offset=EEPROM_read(0,6);
		CH3_offset=EEPROM_read(0,7);
		CH4_offset=EEPROM_read(0,8);
		
		address[0]=EEPROM_read(0,9);
		address[1]=EEPROM_read(0,10);
		address[2]=EEPROM_read(0,11);
		address[3]=EEPROM_read(0,12);
		address[4]=EEPROM_read(0,13);
		hopping[0]=EEPROM_read(0,14);
		hopping[1]=EEPROM_read(0,15);
		hopping[2]=EEPROM_read(0,16);
		hopping[3]=EEPROM_read(0,17);
		hopping[4]=EEPROM_read(0,18);
//		
		VLD1=EEPROM_read(0,19);
		VLD2=EEPROM_read(0,20);
		Sound_switch=EEPROM_read(0,21);	
/*		Sound_switch=EEPROM_read(0,22);
		angle_y_negative=EEPROM_read(0,23);
		angle_x_ratio=EEPROM_read(0,24);
		angle_y_ratio=EEPROM_read(0,25);
		angle_x_middle=EEPROM_read(0,26);
		angle_x_middle<<=8;
		angle_x_middle+=EEPROM_read(0,27);
		angle_y_middle=EEPROM_read(0,28);
		angle_y_middle<<=8;
		angle_y_middle+=EEPROM_read(0,29);*/

		NRF_mapping[0]=EEPROM_read(0,30);
		NRF_mapping[1]=EEPROM_read(0,31);
		NRF_mapping[2]=EEPROM_read(0,32);
		NRF_mapping[3]=EEPROM_read(0,33);
		NRF_mapping[4]=EEPROM_read(0,34);
		NRF_mapping[5]=EEPROM_read(0,35);
		NRF_mapping[6]=EEPROM_read(0,36);
		NRF_mapping[7]=EEPROM_read(0,37);
		
		PPM_mapping[0]=EEPROM_read(0,38);
		PPM_mapping[1]=EEPROM_read(0,39);
		PPM_mapping[2]=EEPROM_read(0,40);
		PPM_mapping[3]=EEPROM_read(0,41);
		PPM_mapping[4]=EEPROM_read(0,42);
		PPM_mapping[5]=EEPROM_read(0,43);
		PPM_mapping[6]=EEPROM_read(0,44);
		PPM_mapping[7]=EEPROM_read(0,45);

		train_switch=EEPROM_read(0,46);
		RC_TX=EEPROM_read(0,47);
		Model_number=EEPROM_read(0,48);Model_current=Model_number;
		PPM_OUT=EEPROM_read(0,49);
		PPM_IN=EEPROM_read(0,50);
		
		auto_switch=EEPROM_read(0,51);
		auto_star=EEPROM_read(0,52);
		auto_end=EEPROM_read(0,53);
		auto_middle=EEPROM_read(0,54);
		auto_mapping=EEPROM_read(0,55);
		auto_speed=EEPROM_read(0,56);
		
	}
	

	IAP_CONTR=0;
}


void DATA_save1()
{
	IAP_CONTR=0x82;
		EEPROM_clean(0);
		EEPROM_write(0,0,0xa0);

//		EEPROM_write(0,1,system);
		EEPROM_write(0,2,Battery_1s);
		EEPROM_write(0,3,LCD_vop);
		EEPROM_write(0,4,LCD_light);
		EEPROM_write(0,5,CH1_offset);
		EEPROM_write(0,6,CH2_offset);
		EEPROM_write(0,7,CH3_offset);
		EEPROM_write(0,8,CH4_offset);
	
		EEPROM_write(0,9,address[0]);
		EEPROM_write(0,10,address[1]);
		EEPROM_write(0,11,address[2]);
		EEPROM_write(0,12,address[3]);
		EEPROM_write(0,13,address[4]);
		EEPROM_write(0,14,hopping[0]);
		EEPROM_write(0,15,hopping[1]);
		EEPROM_write(0,16,hopping[2]);
		EEPROM_write(0,17,hopping[3]);
		EEPROM_write(0,18,hopping[4]);
		
		EEPROM_write(0,19,VLD1);
		EEPROM_write(0,20,VLD2);
		EEPROM_write(0,21,Sound_switch);
/*
		

		EEPROM_write(0,22,angle_x_negative);
		EEPROM_write(0,23,angle_y_negative);
		EEPROM_write(0,24,angle_x_ratio);
		EEPROM_write(0,25,angle_y_ratio);

		EEPROM_write(0,26,angle_x_middle>>8);
		EEPROM_write(0,27,angle_x_middle);
		EEPROM_write(0,28,angle_y_middle>>8);
		EEPROM_write(0,29,angle_y_middle);*/

		EEPROM_write(0,30,NRF_mapping[0]);
		EEPROM_write(0,31,NRF_mapping[1]);
		EEPROM_write(0,32,NRF_mapping[2]);
		EEPROM_write(0,33,NRF_mapping[3]);
		EEPROM_write(0,34,NRF_mapping[4]);
		EEPROM_write(0,35,NRF_mapping[5]);
		EEPROM_write(0,36,NRF_mapping[6]);
		EEPROM_write(0,37,NRF_mapping[7]);
		
		EEPROM_write(0,38,PPM_mapping[0]);
		EEPROM_write(0,39,PPM_mapping[1]);
		EEPROM_write(0,40,PPM_mapping[2]);
		EEPROM_write(0,41,PPM_mapping[3]);
		EEPROM_write(0,42,PPM_mapping[4]);
		EEPROM_write(0,43,PPM_mapping[5]);
		EEPROM_write(0,44,PPM_mapping[6]);
		EEPROM_write(0,45,PPM_mapping[7]);
		
		EEPROM_write(0,46,train_switch);

		EEPROM_write(0,47,RC_TX);
		EEPROM_write(0,48,Model_number);
		EEPROM_write(0,49,PPM_OUT);
		EEPROM_write(0,50,PPM_IN);
		
		EEPROM_write(0,51,auto_switch);
		EEPROM_write(0,52,auto_star);
		EEPROM_write(0,53,auto_end);
		EEPROM_write(0,54,auto_middle);
		EEPROM_write(0,55,auto_mapping);
		EEPROM_write(0,56,auto_speed);
		
	IAP_CONTR=0;
}

void DATA_read2()
{
	u16 i,base;
	IAP_CONTR=0x82;
	base=Model_adress(Model_current); 	

	if(EEPROM_read(2,base)!=0xa0&&EEPROM_read(2,base+1)!=0x0a)
	{
		Model_data_reset();
	}
	else
	{
		for(i=0;i<260;i++)
		{
			TEMP_DATA[i]=EEPROM_read(2,i);	
		}
			
		
			CH_middle[0]=TEMP_DATA[2+base];
			CH_middle[1]=TEMP_DATA[3+base];
			CH_middle[2]=TEMP_DATA[4+base];
			CH_middle[3]=TEMP_DATA[5+base];
			CH_middle[4]=TEMP_DATA[6+base];
			CH_middle[5]=TEMP_DATA[7+base];
			CH_middle[6]=TEMP_DATA[8+base];
			CH_middle[7]=TEMP_DATA[9+base];
			CH_ratio1[0]=TEMP_DATA[10+base];
			CH_ratio1[1]=TEMP_DATA[11+base];
			CH_ratio1[2]=TEMP_DATA[12+base];
			CH_ratio1[3]=TEMP_DATA[13+base];
			CH_ratio1[4]=TEMP_DATA[14+base];
			CH_ratio1[5]=TEMP_DATA[15+base];
			CH_ratio1[6]=TEMP_DATA[16+base];
			CH_ratio1[7]=TEMP_DATA[17+base];
			CH_ratio2[0]=TEMP_DATA[18+base];
			CH_ratio2[1]=TEMP_DATA[19+base];
			CH_ratio2[2]=TEMP_DATA[20+base];
			CH_ratio2[3]=TEMP_DATA[21+base];
			CH_ratio2[4]=TEMP_DATA[22+base];
			CH_ratio2[5]=TEMP_DATA[23+base];
			CH_ratio2[6]=TEMP_DATA[24+base];
			CH_ratio2[7]=TEMP_DATA[25+base];			
			CH_negative[0]=TEMP_DATA[26+base];
			CH_negative[1]=TEMP_DATA[27+base];
			CH_negative[2]=TEMP_DATA[28+base];
			CH_negative[3]=TEMP_DATA[29+base];
			CH_negative[4]=TEMP_DATA[30+base];
			CH_negative[5]=TEMP_DATA[31+base];
			CH_negative[6]=TEMP_DATA[32+base];
			CH_negative[7]=TEMP_DATA[33+base];
			mixer1=TEMP_DATA[34+base];
			mixer2=TEMP_DATA[35+base];
			mixer3=TEMP_DATA[36+base];
			mixer4=TEMP_DATA[37+base];
			mixer1_same=TEMP_DATA[38+base];
			mixer1_contrary=TEMP_DATA[39+base];
			mixer2_same=TEMP_DATA[40+base];
			mixer2_contrary=TEMP_DATA[41+base];
			mixer3_same=TEMP_DATA[42+base];
			mixer3_contrary=TEMP_DATA[43+base];
			mixer4_same=TEMP_DATA[44+base];
			mixer4_contrary=TEMP_DATA[45+base];
			
			IN_mapping[0]=TEMP_DATA[46+base];
			IN_mapping[1]=TEMP_DATA[47+base];
			IN_mapping[2]=TEMP_DATA[48+base];
			IN_mapping[3]=TEMP_DATA[49+base];
			IN_mapping[4]=TEMP_DATA[50+base];
			IN_mapping[5]=TEMP_DATA[51+base];
			IN_mapping[6]=TEMP_DATA[52+base];
			IN_mapping[7]=TEMP_DATA[53+base];
		
	}

	IAP_CONTR=0;
}

void DATA_save2()
{
	u16 i,base;
	IAP_CONTR=0x82;
	base=Model_adress(Model_current);
		EEPROM_clean(2);

		
			TEMP_DATA[base]=0xa0;
			TEMP_DATA[1+base]=0x0a;
			TEMP_DATA[2+base]=CH_middle[0];
			TEMP_DATA[3+base]=CH_middle[1];
			TEMP_DATA[4+base]=CH_middle[2];
			TEMP_DATA[5+base]=CH_middle[3];
			TEMP_DATA[6+base]=CH_middle[4];
			TEMP_DATA[7+base]=CH_middle[5];
			TEMP_DATA[8+base]=CH_middle[6];
			TEMP_DATA[9+base]=CH_middle[7];
			TEMP_DATA[10+base]=CH_ratio1[0];
			TEMP_DATA[11+base]=CH_ratio1[1];
			TEMP_DATA[12+base]=CH_ratio1[2];
			TEMP_DATA[13+base]=CH_ratio1[3];
			TEMP_DATA[14+base]=CH_ratio1[4];
			TEMP_DATA[15+base]=CH_ratio1[5];
			TEMP_DATA[16+base]=CH_ratio1[6];
			TEMP_DATA[17+base]=CH_ratio1[7];
			TEMP_DATA[18+base]=CH_ratio2[0];
			TEMP_DATA[19+base]=CH_ratio2[1];
			TEMP_DATA[20+base]=CH_ratio2[2];
			TEMP_DATA[21+base]=CH_ratio2[3];
			TEMP_DATA[22+base]=CH_ratio2[4];
			TEMP_DATA[23+base]=CH_ratio2[5];
			TEMP_DATA[24+base]=CH_ratio2[6];
			TEMP_DATA[25+base]=CH_ratio2[7];			
			TEMP_DATA[26+base]=CH_negative[0];
			TEMP_DATA[27+base]=CH_negative[1];
			TEMP_DATA[28+base]=CH_negative[2];
			TEMP_DATA[29+base]=CH_negative[3];
			TEMP_DATA[30+base]=CH_negative[4];
			TEMP_DATA[31+base]=CH_negative[5];
			TEMP_DATA[32+base]=CH_negative[6];
			TEMP_DATA[33+base]=CH_negative[7];
			TEMP_DATA[34+base]=mixer1;
			TEMP_DATA[35+base]=mixer2;
			TEMP_DATA[36+base]=mixer3;
			TEMP_DATA[37+base]=mixer4;
			TEMP_DATA[38+base]=mixer1_same;
			TEMP_DATA[39+base]=mixer1_contrary;
			TEMP_DATA[40+base]=mixer2_same;
			TEMP_DATA[41+base]=mixer2_contrary;
			TEMP_DATA[42+base]=mixer3_same;
			TEMP_DATA[43+base]=mixer3_contrary;
			TEMP_DATA[44+base]=mixer4_same;
			TEMP_DATA[45+base]=mixer4_contrary;
			
			TEMP_DATA[46+base]=IN_mapping[0];
			TEMP_DATA[47+base]=IN_mapping[1];
			TEMP_DATA[48+base]=IN_mapping[2];
			TEMP_DATA[49+base]=IN_mapping[3];
			TEMP_DATA[50+base]=IN_mapping[4];
			TEMP_DATA[51+base]=IN_mapping[5];
			TEMP_DATA[52+base]=IN_mapping[6];
			TEMP_DATA[53+base]=IN_mapping[7];

	for(i=0;i<260;i++)
	{
		EEPROM_write(2,i,TEMP_DATA[i]);	
	}
	IAP_CONTR=0;
}

/**************************************************************************/
//NRF24L01+
u8 SPI(u8 byte)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		MDI=(byte&0x80);
		SCK=1;
		byte<<=1;
		byte|=MDO;
		SCK=0;
	}
	return byte;
}

void REG_write(u8 address,u8 command)
{
	CSN=0;
	SPI(0x20+address);
	SPI(command);
	CSN=1;
}

u8 REG_read(u8 address)
{
	u8 m;
	CSN=0;
	SPI(address);
	m=SPI(0);
	CSN=1;
	return m;
}

void FIFO_write(u8 DATA_OUT[],u8 lengh)
{
	u8 i;
	CSN=0;
	SPI(0xa0);
	for(i=0;i<lengh;i++)
	SPI(DATA_OUT[i]);
	CSN=1;
}
void FIFO_write2(u8 DATA_OUT[],u8 lengh)
{
	u8 i;
	CSN=0;
	SPI(0xa0);
	for(i=0;i<lengh;i++)
	SPI(DATA_OUT[i]);
	CSN=1;
}
void FIFO_read(u8 DATA_IN[],u8 lengh)		//读取接收数据缓冲区
{
	u8 i;
	CSN=0;
	SPI(0x61);	//读取命令
	for(i=0;i<lengh;i++)
	DATA_IN[i]=SPI(0);	   
	CSN=1;
}
void FIFO_read2(u8 DATA_IN[],u8 lengh)		//读取接收数据缓冲区
{
	u8 i;
	CSN=0;
	SPI(0x61);	//读取命令
	for(i=0;i<lengh;i++)
	DATA_IN[i]=SPI(0);	   
	CSN=1;
}
void TX_address(u8 DATA_IN[])
{
	CSN=0;		 
	SPI(0x20+0x10);
	SPI(DATA_IN[0]);
	SPI(DATA_IN[1]);
	SPI(DATA_IN[2]);
	SPI(DATA_IN[3]);
	SPI(DATA_IN[4]);
	CSN=1;  
}  
void RX_address(u8 DATA_IN[])
{
	CSN=0;		 
	SPI(0x20+0x0a);
	SPI(DATA_IN[0]);
	SPI(DATA_IN[1]);
	SPI(DATA_IN[2]);
	SPI(DATA_IN[3]);
	SPI(DATA_IN[4]);
	CSN=1;  
}
void RX_mode()				 
{
	CE=0;
	REG_write(0x00,0x3b); //CRC,8 bit,Power on,RX
	CE=1;
} 				   
	
void TX_mode()				 
{
	CE=0;
	REG_write(0x00,0x0a);
  CE=1;
}

void  NRF_power(u8 P)				//发射功率设置
{
	CE=0;
	if(P==3)REG_write(0x06,0x27);		  //0db 修正之前注释错误
	else if(P==2)REG_write(0x06,0x25);	  //-6db
	else if(P==1)REG_write(0x06,0x23);	  //-12db
	else if(P==0)REG_write(0x06,0x21);    //-18db
	CE=1;
}

void NRF_size(u8 l)
{
	CE=0;
	REG_write(0x11,l);  
	CE=1;
}

void NRF_channel(u8 c)
{
	CE=0;
	REG_write(0x05,c);  
	CE=1;
}


void NRF_init()
{	
	CE=0;
	SCK=0;
	REG_write(0x01,0x00); //禁止 自动应答
	REG_write(0x02,0x01); //允许 P0信道
	REG_write(0x04,0x00); //禁止 自动重发
	TX_mode(); 			 
	NRF_channel(66);
	NRF_power(TX_power);
	NRF_size(11);
	TX_address(address);
	RX_address(address);
}

void NRF_test()	//无线模块终极测试
{	
	u8 reset_err=0;
	CE=0;
	SCK=0;
	CSN=0;
	
	if(SPI(0x20)!=0x0e){reset_err=1;}
	SPI(0x0a);

	CSN=1;

	CSN=0;
	SPI(0x00);

	if(SPI(0x00)!=0x0a){NRF_error|=0x02;}//MOSI bad
	CSN=1;

	REG_write(0x01,0x00);
	REG_write(0x04,0x00);
	REG_write(0x11,1);
	
	FIFO_write(tx_buff,1);
	CE=1;

	delay_ms(2);
	
	CSN=0;

	if(SPI(0x00)!=0x2e){NRF_error|=0x04;}//CE bad
	CSN=1;
		
	if(IRQ)NRF_error|=0x18;	//IRQ bad
	else 
	{
		if(NRF_error&0x04==0)NRF_error|=0x10;		//MISO bad
	}
	CE=1;
	
	if(reset_err&&NRF_error>1)NRF_error|=0x01;//CSN,CLK bad
	
	REG_write(0x07,0x20);	//清除TX中断信号
}

void device_connect()
{
	u8 n;
	
	RC_TX=0;
	 
	tx_order[0]=0xa0;
	tx_order[1]=hopping[0];
	tx_order[2]=hopping[1];
	tx_order[3]=hopping[2];
	tx_order[4]=hopping[3];
	tx_order[5]=hopping[4];
	tx_order[6]=address[0];
	tx_order[7]=address[1];
	tx_order[8]=address[2];
	tx_order[9]=address[3];
	tx_order[10]=address[4];	
		 
	NRF_channel(33);
	NRF_power(0);
	NRF_size(11);
	TX_address(address_0);
	RX_address(address_0);
	
	not_get=1;connecting=1;
	while(not_get&connecting)					 //把对频信息发给接收机，若收到回复表面通信成功，
	{								 //收不到继续发送
		TX_mode();
		FIFO_write(tx_order,11);
		Delay1ms();
		RX_mode();
		
		while(1)
		{		
			Delay1ms();
			if(IRQ==0)
			{
				FIFO_read(rx,11);		//读取接收数据
				CE=0;
				REG_write(0x07,0x40);	//清除RX中断信号
				CE=1;	
				if(rx[0]=='O'&&rx[1]=='K')
				{
					not_get=0;break;
				}
			}
				
			n++;if(n>50){n=0;break;}
		}
	}
	
	
	event2=0,t2=0;

	RC_TX=1;
	NRF_power(TX_power);//恢复发射状态
	NRF_size(11);
	TX_address(address);
	RX_address(address); 
	 		 
}






/***********************************************/
//12864液晶操作函数
void LCD(u8 D)
{
 	DATA=D;
	CLK=0;SDA=D_7;CLK=1;
 	CLK=0;SDA=D_6;CLK=1;
 	CLK=0;SDA=D_5;CLK=1;
	CLK=0;SDA=D_4;CLK=1;
	CLK=0;SDA=D_3;CLK=1;
	CLK=0;SDA=D_2;CLK=1;
	CLK=0;SDA=D_1;CLK=1;
	CLK=0;SDA=D_0;CLK=1;
	CLK=0;
}


void location(u8 x,u8 y)
{
	RS=0;
	LCD(0xb0+y);
	LCD(((x>>4)&0x0f)+0x10);
	LCD(x&0x0f);
	RS=1;
}

void LCD_ON()
{CS=0;}

void LCD_OFF()
{CS=1;}

void clean()
{
	u8 i,j;
	for(j=0;j<8;j++)
	{
		location(0,j);
		for(i=0;i<128;i++)
   		LCD(0);
	}
}

void LCD_init()
{
  LCD_ON();
  RST=0;
  delay_ms(2);
  RST=1;
  delay_ms(10);
  RS=0;
  LCD(0xe2);
  LCD(0x2c);  
  LCD(0x2e);
  LCD(0x2f);
  LCD(0x23);
  LCD(0x81);
  LCD(LCD_vop);
  LCD(0xa2);  
  LCD(0xc8);
  LCD(0xa0);
  LCD(0x40);
  LCD(0xaf);
  RS=1;
  clean();
  LCD_OFF();
//  light=LED;  
}

void send(u8 k)
{
	u8 j;
	for(j=0;j<6;j++)
   	LCD(font[k-32][j]);		
}

void send2(u8 k,u8 x,u8 y)		 //写大字母
{
	u8 i;
	k-=32;
	location(x,y);
	for(i=0;i<8;i++)
	{
		LCD(font_big[k][i]);	
	}
	location(x,y+1);
	for(i=8;i<16;i++)
	{
		LCD(font_big[k][i]);	
	}
}

void send2_hex(u8 k,u8 x,u8 y)
{
	u8 i;
	if(k>9)
	{
		k+=23;
		location(x,y);
		for(i=0;i<8;i++)
		{
			LCD(font_big[k][i]);	
		}
		location(x,y+1);
		for(i=8;i<16;i++)
		{
			LCD(font_big[k][i]);	
		}
	}
	else
	{
		k+=16;
		location(x,y);
		for(i=0;i<8;i++)
		{
			LCD(font_big[k][i]);	
		}
		location(x,y+1);
		for(i=8;i<16;i++)
		{
			LCD(font_big[k][i]);	
		}
	}
	
}

void send3(u8 *p,u8 x,u8 y)		 //写汉字
{
	u8 i;
	location(x,y);
	for(i=0;i<12;i++)
	{
		LCD(*p),p++;	
	}
	location(x,y+1);
	for(i=0;i<12;i++)
	{
		LCD(*p),p++;	
	}
}

void write(u8 x,u8 y)			//显示汉字
{
	u8 *d;
	d=word;
	while(*d)
	{		
		send3(hzk[*d],x,y);
		d++;x+=12;
	}

}
void write0(u8 *d,u8 x,u8 y)			//显示汉字串
{

	while(*d)
	{		
		send3(hzk[*d],x,y);
		d++;x+=12;
	}

}
/*
void write2(u8 x,u8 y)			//显示大字母
{
	u8 *d;
	d=word;
	while(*d)
	{		
		send2(*d,x,y);
		d++;x+=8;
	}

}*/


void photo(u8 *p,u8 x,u8 y,u8 length,u8 width)
{
  u8 i,j;
  for(i=0;i<width;i++)
  {
		location(x,i+y);
		for(j=0;j<length;j++)
	  LCD(*p),p++;
  }  	
}
void photo2(u8 *p,u8 x,u8 y,u8 length,u8 width)
{
  u8 i,j;
  for(i=0;i<width;i++)
  {
		location(x,i+y);
		for(j=0;j<length;j++)
	  LCD(*p),p++;
  }  	
}
void CH13_trim(u8 v,u8 base)
{
	u8 i;unsigned long L1,L2=0,L3,L4=0;
	L1=0x03e00000;v/=4;
	for(i=0;i<v;i++)
	{
		if(L1&1)
		{
			L1>>=1,L2>>=1;L2|=0x80000000;
		}
		else L1>>=1,L2>>=1;
	}
	L2|=0x40000000;

	if(v==0)L3=0x00100000,L4=0;
	else if(v==50)L3=0,L4=0x00000100;
	else
	{
		L3=0x02080000;L4=0;
		for(i=1;i<v;i++)
		{
			if(L3&1)
			{
				L3>>=1,L4>>=1;L4|=0x80000000;
			}
			else L3>>=1,L4>>=1;
		}
	}
	L4|=0x40000000;

	location(base,7);LCD(L1>>24);LCD(L3>>24);LCD(0x03);LCD(L3>>24);LCD(L1>>24);
	location(base,6);LCD(L1>>16);LCD(L3>>16);LCD(0xff);LCD(L3>>16);LCD(L1>>16);
	location(base,5);LCD(L1>>8); LCD(L3>>8); LCD(0xff);LCD(L3>>8); LCD(L1>>8);
	location(base,4);LCD(L1);    LCD(L3);	 LCD(0xff);LCD(L3);    LCD(L1);
	location(base,3);LCD(L2>>24);LCD(L4>>24);LCD(0xff);LCD(L4>>24);LCD(L2>>24);
	location(base,2);LCD(L2>>16);LCD(L4>>16);LCD(0xff);LCD(L4>>16);LCD(L2>>16);
	location(base,1);LCD(L2>>8); LCD(L4>>8); LCD(0xff);LCD(L4>>8); LCD(L2>>8);  
	location(base,0);LCD(L2);	 LCD(L4);	 LCD(0xf8);LCD(L4);    LCD(L2);	   	 
}

void CH24_trim(u8 v)
{
	u8 i;char m;
	v=v/4+2;		
	for(i=0;i<55;i++)
	{		
		if(i==27)LCD(0x7c);
		else
		{
			m=v-i;
			if(m==3)LCD(0x38);
			else if(m==2)LCD(0x54);	
			else if(m==1)LCD(0x54);
			else if(m==0)LCD(0x54);
			else if(m==-1)LCD(0x54);
			else if(m==-2)LCD(0x54);
			else if(m==-3)LCD(0x38);
			else LCD(0x10);
		}
		
	}
}

void throttle(u8 x)
{
	u8 i;long L=0;
	x=x/8;
	for(i=0;i<x;i++)
	{
		L>>=1;
		L|=0x80000000;
	}
	L++;
	location(9,6);LCD(L>>24);LCD(L>>24);
	location(9,5);LCD(L>>16);LCD(L>>16);
	location(9,4);LCD(L>>8);LCD(L>>8);
	location(9,3);LCD(L);LCD(L);
}
/*
void warning(u8 x,u8 y)
{
	location(x,y);
	LCD(0x40);LCD(0x60);LCD(0x50);LCD(0x48);LCD(0x44);LCD(0x42);LCD(0x6d);
	LCD(0x42);LCD(0x44);LCD(0x48);LCD(0x50);LCD(0x60);LCD(0x40);
}
*/
void write_num100(int num,u8 x,u8 y,u8 c)//写1000内数字，c为类型选择
{
	if(c=='-'||c=='/')
	{
		if(num>0)send2('+',x-8,y);
		else if(num<0)num=-num,send2('-',x-8,y);
	}
	if(num>99)send2(num/100+48,x,y);
	else send2(' ',x,y);
	send2(num%100/10+48,x+8,y);
	if(c=='.')
	{
		send2('.',x+16,y);
		send2(num%10+48,x+24,y);
	}
	else
	{
		send2(num%10+48,x+16,y);
		if(c=='%'||c=='/')send2('%',x+24,y);
	}
	
}

void write_num1000(int num)//写10000内数字
{
	
	if(num>999)send(num/1000+48);
	else send(' ');
	if(num>99)send(num%1000/100+48);
	else send(' ');
	send(num%100/10+48);
	send(num%10+48);
	
}

u8 move_cursor(u8 line,u8 line_max)//光标移动
{
	if(up)
	{
		up=0;
		if(line>0)line--;
		else line=line_max;
	}
	if(down)
	{
		down=0;
		if(line<line_max)line++;
		else line=0;
	}
	return line;
}

void display_menu(u8 line,u8 line_max)//显示菜单
{
	if(line_max<4)send3(hzk[1],0,line+line);
	else 
	{
		if(line<3)send3(hzk[1],0,line+line);
		else if(line==line_max)send3(hzk[1],0,6);
		else send3(hzk[1],0,4);
	}
	
	if(line<3)
	{
		if(line_max==2)
		{
			write0(WW[0],12,0);
			write0(WW[1],12,2);
			write0(WW[2],12,4);
		}
		else
		{
			write0(WW[0],12,0);
			write0(WW[1],12,2);
			write0(WW[2],12,4);
			write0(WW[3],12,6);	
		}
		
	}
	else if(line==line_max)
	{
		write0(WW[line-3],12,0);
		write0(WW[line-2],12,2);
		write0(WW[line-1],12,4);
		write0(WW[line],12,6);
	}
	else if(line<line_max)
	{
		write0(WW[line-2],12,0);
		write0(WW[line-1],12,2);
		write0(WW[line],12,4);
		write0(WW[line+1],12,6);
	}
	if(line_max==9)
	{
		location(97,7);
		if(line==9)send('1'),send('0');
		else send(' '),send(line+49);
		send('/');send('1');send('0');
	}
	else
	{
		location(109,7);send(line+49);send('/');send(line_max+49);
	}
		
}

void display_mapping(u8 mode,u8 x,u8 y)//显示映射对象
{
	switch(mode)
	{
		case 0:send3(hzk[96],x,y);send3(hzk[97],x+12,y);send2('1',x+24,y);break;
		case 1:send3(hzk[96],x,y);send3(hzk[97],x+12,y);send2('2',x+24,y);break;
		case 2:send3(hzk[96],x,y);send3(hzk[97],x+12,y);send2('3',x+24,y);break;
		case 3:send3(hzk[96],x,y);send3(hzk[97],x+12,y);send2('4',x+24,y);break;
		case 4:send3(hzk[100],x,y);send3(hzk[101],x+12,y);send2('1',x+24,y);break;
		case 5:send3(hzk[100],x,y);send3(hzk[101],x+12,y);send2('2',x+24,y);break;
		case 6:send3(hzk[58],x,y);send3(hzk[59],x+12,y);send2('A',x+24,y);break;
		case 7:send3(hzk[58],x,y);send3(hzk[59],x+12,y);send2('B',x+24,y);break;
		case 8:send3(hzk[58],x,y);send3(hzk[59],x+12,y);send2('C',x+24,y);break;
		case 9:send3(hzk[58],x,y);send3(hzk[59],x+12,y);send2('D',x+24,y);break;
/*		case 10:send3(hzk[102],x,y);send3(hzk[103],x+12,y);send2('X',x+24,y);break;
		case 11:send3(hzk[102],x,y);send3(hzk[103],x+12,y);send2('Y',x+24,y);break;
		case 12:send3(hzk[102],x,y);send3(hzk[103],x+12,y);send2('Z',x+24,y);break;
		case 13:send3(hzk[104],x,y);send3(hzk[105],x+12,y);send2('1',x+24,y);break;
		case 14:send3(hzk[104],x,y);send3(hzk[105],x+12,y);send2('2',x+24,y);break;*/
		case 10:send2('P',x,y);send2('P',x+8,y);send2('M',x+16,y);send2('1',x+24,y);break;
		case 11:send2('P',x,y);send2('P',x+8,y);send2('M',x+16,y);send2('2',x+24,y);break;
		case 12:send2('P',x,y);send2('P',x+8,y);send2('M',x+16,y);send2('3',x+24,y);break;
		case 13:send2('P',x,y);send2('P',x+8,y);send2('M',x+16,y);send2('4',x+24,y);break;
		case 14:send2('P',x,y);send2('P',x+8,y);send2('M',x+16,y);send2('5',x+24,y);break;
		case 15:send2('P',x,y);send2('P',x+8,y);send2('M',x+16,y);send2('6',x+24,y);break;
		case 16:send2('P',x,y);send2('P',x+8,y);send2('M',x+16,y);send2('7',x+24,y);break;
		case 17:send2('P',x,y);send2('P',x+8,y);send2('M',x+16,y);send2('8',x+24,y);break;
	}
	
}

void display_switch(u8 sw,u8 x,u8 y)
{
	switch(sw)
	{
		case 0:send2('-',x+4,y);send3(hzk[10],x+12,y);send2('-',x+24,y);break;
		case 1:send3(hzk[58],x,y);send3(hzk[59],x+12,y);send2('A',x+24,y);break;
		case 2:send3(hzk[58],x,y);send3(hzk[59],x+12,y);send2('B',x+24,y);break;
		case 3:send3(hzk[58],x,y);send3(hzk[59],x+12,y);send2('C',x+24,y);break;
		case 4:send3(hzk[58],x,y);send3(hzk[59],x+12,y);send2('D',x+24,y);break;
	}
}



/************************************************************************/
//数据运算函数
/*
void data_check(int x,int max,int min)
{
	
	if(x>max)x=max;
	if(x<min)x=min;

}
*/
void Xdata_check(int *p)//坑爹的STC Xdata，只能用指针来调用，否则会出错
{
	u8 i;
	for(i=0;i<8;i++)
	{
		if(*p>1023)*p=1023;
		if(*p<0)*p=0; 
		p++;
	}
	
}



void function_size(u8 num)//舵量函数
{

	if(operation[num]>0)
	{
		operation[num]=(long)operation[num]*CH_ratio2[num]/100;
	}
	else
	{
		operation[num]=(long)operation[num]*CH_ratio1[num]/100;
	}
	
}

void function_negative(u8 num)//反转函数
{
	
	if(CH_negative[num]==1)operation[num]=-operation[num]+CH_middle[num];
	else operation[num]+=CH_middle[num];
}


void function_mix(u8 same,u8 contrary)//混控函数 
{
	int y1,y2;
	
	y1=operation[same]+operation[contrary];
	y2=operation[same]-operation[contrary];
	
	operation[same]=y1;
	operation[contrary]=y2;
			
}

int CH_calib(int x,u8 offset)				//修正摇杆行程
{

	x=512+(x-512)*51/offset;

	if(x>1023)x=1023;
	if(x<0)x=0;
	return x;		
}

void function_mapping(u8 num)//输入映射
{

	switch(IN_mapping[num])
	{
		case 0:operation[num]=CH_value[0]*5/6-426;break;
		case 1:operation[num]=CH_value[1]*5/6-426;break;
		case 2:operation[num]=CH_value[2]*5/6-426;break;
		case 3:operation[num]=CH_value[3]*5/6-426;break;
		case 4:operation[num]=adc[2]*5/6-426;break;
		case 5:operation[num]=adc[3]*5/6-426;break;
		
		case 6:if(Switch_A==1)operation[num]=-426;
						else if(Switch_A==2)operation[num]=0;
						else operation[num]=426;break;
		case 7:if(Switch_B==1)operation[num]=-426;
						else if(Switch_B==2)operation[num]=0;
						else operation[num]=426;break;
		case 8:if(Switch_C==1)operation[num]=-426;
						else if(Switch_C==2)operation[num]=0;
						else operation[num]=426;break;
		case 9:if(Switch_D==1)operation[num]=-426;
						else if(Switch_D==2)operation[num]=0;
						else operation[num]=426;break;		
		case 10:operation[num]=PPM_DATA[0];break;
		case 11:operation[num]=PPM_DATA[1];break;
		case 12:operation[num]=PPM_DATA[2];break;
		case 13:operation[num]=PPM_DATA[3];break;
		case 14:operation[num]=PPM_DATA[4];break;
		case 15:operation[num]=PPM_DATA[5];break;
		case 16:operation[num]=PPM_DATA[6];break;
		case 17:operation[num]=PPM_DATA[7];break;
	
	}
}

void initial()			 //硬件初始化
{
	ADC_CONTR=0x80;		 //ADC配置
	P1ASF=0xff;
	
	LEDA=0;
	P4SW=0x70;			 //允许P4 IO
	P2M0=0x20;			 //推挽点亮背光
	
	


 	TMOD=0x11;
	TCON=0x04;           
                
  CCAPM0=0x49;		   //PCA定时器初始化
	
	value0 += 3000;	   //3ms
	CCAP0L = value0;
    CCAP0H = value0>> 8;
	value0 += 3000;	   //3ms
	
	value1 += 20000;	   //20ms
	CCAP1L = value1;
    CCAP1H = value1>> 8;  
		value1 += 3000;	   //3ms
	
	IP=0x80;
	IPH=0x0c;			 //优先级非常重要
	
	CCON=0x40;//打开PCA定时器
	IE=0x82;
  
}	
	

main()
{
	int temp;u8 boot_time;
	
	delay_ms(200);
	initial();
	
	DATA_read1();//数据验证
	DATA_read2();
	LCD_init();
	LEDA=LCD_light;

	LCD_ON();
	//*****************串口******************//
	PCON &= 0x7F;		//??????
	SCON = 0x50;		//8???,?????
	AUXR |= 0x04;		//???????????Fosc,?1T
	BRT = 0xB8;		//?????????????波特率2400
	AUXR |= 0x01;		//??1?????????????????
	AUXR |= 0x10;		//??????????
    ES    = 1;                  //打开串口中断
	//****************************************//
	if(KEY==0)//强制恢复出厂
	{
		delay_ms(3000);
		if(KEY==0)
		{
			IAP_CONTR=0x82;		//擦除所有数据
			EEPROM_clean(0);
		 	EEPROM_clean(2);
			IAP_CONTR=0;
			word[0]=106,word[1]=107,word[2]=32,word[3]=136,word[4]=0,write(40,2);
			word[0]=138,word[1]=139,word[2]=140,word[3]=57,word[4]=0,write(40,4);
			while(1);
		}	
	}
	
	if(first)//首次开机
	{
		while(1)//设置
		{
			word[0]=130,word[1]=81,word[2]=131,word[3]=58,word[4]=132,word[5]=0,write(30,0);
			word[0]=141,word[1]=142,word[2]=18,word[3]=13,word[4]=0,write(0,2);
			word[0]=149,word[1]=150,word[2]=137,word[3]=109,word[4]=0,write(64,2);
			if(adc[4]>510)
			{
				send3(hzk[1],0,4);send2('1',12,4);send2('2',20,4);send2('L',28,4);send2('E',36,4);
				send3(hzk[2],0,6);send2('1',12,6);send2('2',20,6);send2('C',28,6);send2(' ',36,6);
				if(KEY==0)
				{
					Battery_1s=1;
					while(!KEY);break;
				}
				
			}
			else 
			{
				send3(hzk[2],0,4);send2('1',12,4);send2('2',20,4);send2('L',28,4);send2('E',36,4);
				send3(hzk[1],0,6);send2('1',12,6);send2('2',20,6);send2('C',28,6);send2(' ',36,6);
				if(KEY==0)
				{
					Battery_1s=0;
					while(!KEY);break;
				}
			}
			delay_ms(100);

		}
		

		if(hopping[0]>125)hopping[0]=(hopping[0]-5)/2;
		if(hopping[1]>125)hopping[1]=(hopping[1]-5)/2;
		if(hopping[2]>125)hopping[2]=(hopping[2]-5)/2;
		if(hopping[3]>125)hopping[3]=(hopping[3]-5)/2;
		if(hopping[4]>125)hopping[4]=(hopping[4]-5)/2;
		
		DATA_save1();
		
		word[0]=157,word[1]=132,word[2]=12,word[3]=13,word[4]=52,word[5]=53;word[6]=0,write(0,4);
		delay_ms(500);
		send3(hzk[1],0,6);delay_ms(300);
		send3(hzk[1],12,6);delay_ms(300);
		send3(hzk[1],24,6);delay_ms(300);
		send3(hzk[1],36,6);delay_ms(300);
		send3(hzk[1],48,6);delay_ms(300);
		send3(hzk[1],60,6);delay_ms(300);

		
	}
	else//开机安全油门
	{
		delay_ms(100);
		CH_value[2]=CH_calib(adc[4],CH3_offset);
		if(CH_value[2]>80)	
			{
				while(1)
				{
					photo2(bmp5,40,2,50,4);
				
					word[0]=147,word[1]=148,word[2]=149,word[3]=150,word[4]=151,
					word[5]=52,word[6]=153,word[7]=0,write(20,6); 
			
					delay_ms(100);
					if(KEY==0)break;
					CH_value[2]=CH_calib(adc[4],CH3_offset);
					if(CH_value[2]<80)break;
				}  
			}	
		
	}
	
	
	
	
	NRF_test();
	NRF_init();
	if(NRF_error)
	{
		photo2(screen2,0,0,128,8);
		if(NRF_error&0x01)location(39,5),send(133),location(106,5),send(133);
		if(NRF_error&0x02)location(83,5),send(133);
		if(NRF_error&0x04)location(29,5),send(133);
		if(NRF_error&0x08)location(64,5),send(133);
		if(NRF_error&0x10)location(49,5),send(133);
		delay_ms(2000);
	}
	else
	{
		
		photo2(screen3,0,0,128,8);
		if(boot_time<100)
		{
			delay_ms(500);
			photo2(bmp4,80,4,46,4);
			location(107,6);LCD(0);LCD(0x20);LCD(0xf0);LCD(0);
			location(107,7);LCD(0xc0);LCD(0xc8);LCD(0xcf);LCD(0xc8);
			delay_ms(500);
		}
		else if(boot_time<200)
		{
			delay_ms(1000);
			photo2(bmp4,80,4,46,4);
			location(107,6);LCD(0x20);LCD(0x10);LCD(0x10);LCD(0xe0);
			location(107,7);LCD(0xcc);LCD(0xca);LCD(0xc9);LCD(0xc8);
			delay_ms(1000);
		}
		else 
		{
			delay_ms(2000);
			photo2(bmp4,80,4,46,4);
			delay_ms(1500);
		}

	}

//	device_connect();

	



	if(PPM_OUT)
	{
		TMOD=0x11;
		EX1=0;ET1=1;
	}
	if(PPM_IN)
	{
		TMOD=0x91;
		EX1=1;ET1=0;
	}

	auto_max=auto_end*4.25+511;
	auto_min=auto_star*4.25+511;			 
		
	
	TR1=1;
	TR0=1;
	delay_ms(50);
	CCAPM1=0x49;

//photo(name1,87,3,36,2);

	
//	photo(bmp2,0,0,128,8);
//	while(1);
	
	

	while(1)	//主函数做刷新显示、菜单处理
	{
		
		if(flag_20ms)
		{
			flag_20ms=0,tt++;
/*************************************************************/
//按键检测	
			if(KEY)
			{
				if(KS)
				{
					KS=0;beep=1;
					if(menu)
					{
						star=1,menu=0;
						
						
						if(menu_2==19)
						{
							RC_check=0;
						}
						
					}
					else 
					{
						star=1;menu=1,menu_1=0,menu_2=0;
					}
				}
			}
			else KS=1;
			
			if(K1)					
			{
				s1_press_time=0;
				s1_long_press=0;
				if(s1)				   
				{					   
					s1=0;			  
					key1=1;
				} 
			}
			else 
			{
				s1=1;
				s1_press_time++;
				if(s1_press_time>50)
				{
					s1_press_time=0;s1=0;
					s1_long_press=1;
				}
			}				  
	
			if(K2)					
			{
				s2_press_time=0;
				s2_long_press=0;
				if(s2)				   
				{					   
					s2=0;			  
					key2=1;
				} 
			}
			else 
			{
				s2=1;
				s2_press_time++;
				if(s2_press_time>50)
				{
					s2_press_time=0;s2=0;
					s2_long_press=1;
				}
			}
			
			if(K3)					
			{
				s3_press_time=0;
				s3_long_press=0;
				if(s3)				   
				{					   
					s3=0;			  
					key3=1;
				} 
			}
			else 
			{
				s3=1;
				s3_press_time++;
				if(s3_press_time>50)
				{
					s3_press_time=0;s3=0;
					s3_long_press=1;
				}
			}
			
			if(K4)					
			{
				s4_press_time=0;
				s4_long_press=0;
				if(s4)				   
				{					   
					s4=0;			  
					key4=1;
				} 
			}
			else 
			{
				s4=1;
				s4_press_time++;
				if(s4_press_time>50)
				{
					s4_press_time=0;s4=0;
					s4_long_press=1;
				}
			}
			
			if(K5)					
			{
				s5_press_time=0;
				s5_long_press=0;
				if(s5)				   
				{					   
					s5=0;			  
					key5=1;
				} 
			}
			else 
			{
				s5=1;
				s5_press_time++;
				if(s5_press_time>50)
				{
					s5_press_time=0;s5=0;
					s5_long_press=1;
				}
			}
			
			if(K6)					
			{
				s6_press_time=0;
				s6_long_press=0;
				if(s6)				   
				{					   
					s6=0;			  
					key6=1;
				} 
			}
			else 
			{
				s6=1;
				s6_press_time++;
				if(s6_press_time>50)
				{
					s6_press_time=0;s6=0;
					s6_long_press=1;
				}
			}
				
		
			
		}
		else
		{
			while(!flag_20ms);
		}
		
			
		if(tt==1)//按键处理
		{
			
			if(s1_long_press)
			{
				key_time=!key_time;
				if(key_time)key1=1;
			}
			if(s2_long_press)
			{
				key_time=!key_time;
				if(key_time)key2=1;
			}
						
			if(menu==0)
			{
				if(s3_long_press)
				{
					key_time=!key_time;
					if(key_time)key3=1;
				}
				if(s4_long_press)
				{
					key_time=!key_time;
					if(key_time)key4=1;
				}		
			}
			
			if(s5_long_press)
			{
				key_time=!key_time;
				if(key_time)key5=1;
			}
			if(s6_long_press)
			{
				key_time=!key_time;
				if(key_time)key6=1;
			}

			if(key1)
			{
				key1=0;beep=1;
				if(menu)decrease=1;
				else 
				{
					if(CH_negative[3]==0)
					{
						if(CH_middle[3]>-100)CH_middle[3]--,Data_change(2);
					}
					else
					{
						if(CH_middle[3]<100)CH_middle[3]++,Data_change(2);
					}
					if(CH_middle[3]==0)beep=2;
					CH_middle_change=4,event1=1,t1=0;
				}
			}
			if(key2)
			{
				key2=0;beep=1;
				if(menu)increase=1;
				else 
				{
					if(CH_negative[3]==0)
					{
						if(CH_middle[3]<100)CH_middle[3]++,Data_change(2);
					}
					else
					{
						if(CH_middle[3]>-100)CH_middle[3]--,Data_change(2);
					}
					if(CH_middle[3]==0)beep=2;
					CH_middle_change=4,event1=1,t1=0;
				}
			}
			if(key3)
			{
				key3=0;beep=1;
				if(menu)
					left=1;
				else 
				{
					if(CH_negative[1]==0)
					{
						if(CH_middle[1]>-100)CH_middle[1]--,Data_change(2);
					}
					else
					{
						if(CH_middle[1]<100)CH_middle[1]++,Data_change(2);
					}
					if(CH_middle[1]==0)beep=2;
					CH_middle_change=2,event1=1,t1=0;
					
				}
			}
			if(key4)
			{
				key4=0;beep=1;
				if(menu)right=1;
				else 
				{
					if(CH_negative[1]==0)
					{
						if(CH_middle[1]<100)CH_middle[1]++,Data_change(2);
					}
					else
					{
						if(CH_middle[1]>-100)CH_middle[1]--,Data_change(2);
					}
					if(CH_middle[1]==0)beep=2;
					CH_middle_change=2,event1=1,t1=0;
					
				}
			}
			if(key5)
			{
				key5=0;beep=1;
				if(menu)down=1;
				else 
				{
					if(CH_negative[0]==0)
					{
						if(CH_middle[0]>-100)CH_middle[0]--,Data_change(2);
					}
					else
					{
						if(CH_middle[0]<100)CH_middle[0]++,Data_change(2);
					}
					if(CH_middle[0]==0)beep=2;
					CH_middle_change=1,event1=1,t1=0;
				}
			}
			if(key6)
			{
				key6=0;beep=1;
				if(menu)up=1;
				else 
				{
					if(CH_negative[0]==0)
					{
						if(CH_middle[0]<100)CH_middle[0]++,Data_change(2);
					}
					else
					{
						if(CH_middle[0]>-100)CH_middle[0]--,Data_change(2);
					}
					if(CH_middle[0]==0)beep=2;
					CH_middle_change=1,event1=1,t1=0;
				}
			}

		}
		else if(tt==2)
		{
			LCD_ON();
			if(menu)	//主菜单			 
			{			
				if(star)
				{
					up=0,down=0,left=0,right=0,increase=0,decrease=0;
				}
				clean();
				if(menu_1)//一级菜单
				{
					if(menu_2)//二级菜单
					{
						
						
						if(menu_2==1)//正反
						{
							line_2=move_cursor(line_2,7);
							
							WW[0][0]=94,WW[0][1]=95,WW[0][2]=81,WW[0][3]=0;
							WW[1][0]=94,WW[1][1]=95,WW[1][2]=82,WW[1][3]=0;
							WW[2][0]=94,WW[2][1]=95,WW[2][2]=83,WW[2][3]=0;
							WW[3][0]=94,WW[3][1]=95,WW[3][2]=84,WW[3][3]=0;
							WW[4][0]=94,WW[4][1]=95,WW[4][2]=85,WW[4][3]=0;
							WW[5][0]=94,WW[5][1]=95,WW[5][2]=86,WW[5][3]=0;
							WW[6][0]=94,WW[6][1]=95,WW[6][2]=87,WW[6][3]=0;
							WW[7][0]=94,WW[7][1]=95,WW[7][2]=88,WW[7][3]=0;
							display_menu(line_2,7);
							
							if(line_2<3)
							{
								if(CH_negative[0])send3(hzk[35],60,0);
								else send3(hzk[34],60,0);
								if(CH_negative[1])send3(hzk[35],60,2);
								else send3(hzk[34],60,2);
								if(CH_negative[2])send3(hzk[35],60,4);
								else send3(hzk[34],60,4);
								if(CH_negative[3])send3(hzk[35],60,6);
								else send3(hzk[34],60,6);
							}
							else if(line_2==7)
							{
								if(CH_negative[4])send3(hzk[35],60,0);
								else send3(hzk[34],60,0);
								if(CH_negative[5])send3(hzk[35],60,2);
								else send3(hzk[34],60,2);
								if(CH_negative[6])send3(hzk[35],60,4);
								else send3(hzk[34],60,4);
								if(CH_negative[7])send3(hzk[35],60,6);
								else send3(hzk[34],60,6);
							}
							else
							{
								if(CH_negative[line_2-2])send3(hzk[35],60,0);
								else send3(hzk[34],60,0);
								if(CH_negative[line_2-1])send3(hzk[35],60,2);
								else send3(hzk[34],60,2);
								if(CH_negative[line_2])send3(hzk[35],60,4);
								else send3(hzk[34],60,4);
								if(CH_negative[line_2+1])send3(hzk[35],60,6);
								else send3(hzk[34],60,6);
							}
								
								
							if(increase|decrease)
							{
								increase=0;decrease=0;
								if(CH_negative[line_2])
								{
									CH_negative[line_2]=0;Data_change(2);
								}
								else
								{
									CH_negative[line_2]=1;Data_change(2);
								}
								
							}						
							
							
						}
						else if(menu_2==2)//负舵量
						{
							line_2=move_cursor(line_2,7);
							
							WW[0][0]=94,WW[0][1]=95,WW[0][2]=81,WW[0][3]=0;
							WW[1][0]=94,WW[1][1]=95,WW[1][2]=82,WW[1][3]=0;
							WW[2][0]=94,WW[2][1]=95,WW[2][2]=83,WW[2][3]=0;
							WW[3][0]=94,WW[3][1]=95,WW[3][2]=84,WW[3][3]=0;
							WW[4][0]=94,WW[4][1]=95,WW[4][2]=85,WW[4][3]=0;
							WW[5][0]=94,WW[5][1]=95,WW[5][2]=86,WW[5][3]=0;
							WW[6][0]=94,WW[6][1]=95,WW[6][2]=87,WW[6][3]=0;
							WW[7][0]=94,WW[7][1]=95,WW[7][2]=88,WW[7][3]=0;
							display_menu(line_2,7);
							
							if(increase)
							{
								increase=0;
								if(CH_ratio1[line_2]<120)
								{
									CH_ratio1[line_2]++;Data_change(2);
								}
								
							}
							if(decrease)
							{
								decrease=0;
								if(CH_ratio1[line_2]>0)
								{
									CH_ratio1[line_2]--;Data_change(2);
								}
								
							}
							
							if(line_2<3)
							{
								write_num100(CH_ratio1[0],70,0,'%');
								write_num100(CH_ratio1[1],70,2,'%');
								write_num100(CH_ratio1[2],70,4,'%');
								write_num100(CH_ratio1[3],70,6,'%');
							}
							else if(line_2==7)
							{
								write_num100(CH_ratio1[4],70,0,'%');
								write_num100(CH_ratio1[5],70,2,'%');
								write_num100(CH_ratio1[6],70,4,'%');
								write_num100(CH_ratio1[7],70,6,'%');
							}
							else
							{
								write_num100(CH_ratio1[line_2-2],70,0,'%');
								write_num100(CH_ratio1[line_2-1],70,2,'%');
								write_num100(CH_ratio1[line_2],70,4,'%');
								write_num100(CH_ratio1[line_2+1],70,6,'%');
							}
							send2('-',62,0);
							send2('-',62,2);
							send2('-',62,4);
							send2('-',62,6);
							
						}
						else if(menu_2==3)//输入映射
						{
							line_2=move_cursor(line_2,7);
							
							WW[0][0]=94,WW[0][1]=95,WW[0][2]=81,WW[0][3]=0;
							WW[1][0]=94,WW[1][1]=95,WW[1][2]=82,WW[1][3]=0;
							WW[2][0]=94,WW[2][1]=95,WW[2][2]=83,WW[2][3]=0;
							WW[3][0]=94,WW[3][1]=95,WW[3][2]=84,WW[3][3]=0;
							WW[4][0]=94,WW[4][1]=95,WW[4][2]=85,WW[4][3]=0;
							WW[5][0]=94,WW[5][1]=95,WW[5][2]=86,WW[5][3]=0;
							WW[6][0]=94,WW[6][1]=95,WW[6][2]=87,WW[6][3]=0;
							WW[7][0]=94,WW[7][1]=95,WW[7][2]=88,WW[7][3]=0;
							display_menu(line_2,7);
							
							send3(hzk[4],60,0);
							send3(hzk[4],60,2);
							send3(hzk[4],60,4);
							send3(hzk[4],60,6);
							
							if(increase)
							{
								increase=0;
								if(IN_mapping[line_2]<17)IN_mapping[line_2]++,Data_change(2);
							}
							if(decrease)
							{
								decrease=0;
								if(IN_mapping[line_2]>0)IN_mapping[line_2]--,Data_change(2);
							}
							
							if(line_2<3)
							{
								display_mapping(IN_mapping[0],72,0);
								display_mapping(IN_mapping[1],72,2);
								display_mapping(IN_mapping[2],72,4);
								display_mapping(IN_mapping[3],72,6);
							}
							else if(line_2<4)
							{
								display_mapping(IN_mapping[1],72,0);
								display_mapping(IN_mapping[2],72,2);
								display_mapping(IN_mapping[3],72,4);
								display_mapping(IN_mapping[4],72,6);
							}
							else if(line_2<5)
							{
								display_mapping(IN_mapping[2],72,0);
								display_mapping(IN_mapping[3],72,2);
								display_mapping(IN_mapping[4],72,4);
								display_mapping(IN_mapping[5],72,6);
							}
							else if(line_2<6)
							{
								display_mapping(IN_mapping[3],72,0);
								display_mapping(IN_mapping[4],72,2);
								display_mapping(IN_mapping[5],72,4);
								display_mapping(IN_mapping[6],72,6);
							}
							else 
							{
								display_mapping(IN_mapping[4],72,0);
								display_mapping(IN_mapping[5],72,2);
								display_mapping(IN_mapping[6],72,4);
								display_mapping(IN_mapping[7],72,6);
							}
							
							
							
							
						}
						else if(menu_2==4)//微调
						{
							line_2=move_cursor(line_2,7);
							
							WW[0][0]=94,WW[0][1]=95,WW[0][2]=81,WW[0][3]=0;
							WW[1][0]=94,WW[1][1]=95,WW[1][2]=82,WW[1][3]=0;
							WW[2][0]=94,WW[2][1]=95,WW[2][2]=83,WW[2][3]=0;
							WW[3][0]=94,WW[3][1]=95,WW[3][2]=84,WW[3][3]=0;
							WW[4][0]=94,WW[4][1]=95,WW[4][2]=85,WW[4][3]=0;
							WW[5][0]=94,WW[5][1]=95,WW[5][2]=86,WW[5][3]=0;
							WW[6][0]=94,WW[6][1]=95,WW[6][2]=87,WW[6][3]=0;
							WW[7][0]=94,WW[7][1]=95,WW[7][2]=88,WW[7][3]=0;
							display_menu(line_2,7);
							
							if(increase)
							{
								increase=0;
								if(CH_middle[line_2]<100)
								{
									CH_middle[line_2]++;Data_change(2);
								}
								
							}
							if(decrease)
							{
								decrease=0;
								if(CH_middle[line_2]>-100)
								{
									CH_middle[line_2]--;Data_change(2);
								}
								
							}
							
							if(line_2<3)
							{
								write_num100(CH_middle[0],70,0,'-');
								write_num100(CH_middle[1],70,2,'-');
								write_num100(CH_middle[2],70,4,'-');
								write_num100(CH_middle[3],70,6,'-');
							}
							else if(line_2==7)
							{
								write_num100(CH_middle[4],70,0,'-');
								write_num100(CH_middle[5],70,2,'-');
								write_num100(CH_middle[6],70,4,'-');
								write_num100(CH_middle[7],70,6,'-');
							}
							else
							{
								write_num100(CH_middle[line_2-2],70,0,'-');
								write_num100(CH_middle[line_2-1],70,2,'-');
								write_num100(CH_middle[line_2],70,4,'-');
								write_num100(CH_middle[line_2+1],70,6,'-');
							}
		
						}
						else if(menu_2==5)//混控
						{
							line_2=move_cursor(line_2,2);
							
							WW[0][0]=91,WW[0][1]=79,WW[0][2]=92,WW[0][3]=0;
							WW[1][0]=93,WW[1][1]=79,WW[1][2]=92,WW[1][3]=0;
							WW[2][0]=106,WW[2][1]=107,WW[2][2]=108,WW[2][3]=109;WW[2][4]=0;
							display_menu(line_2,2);
							
							if(mixer_name==1)
							{
								word[0]=110;word[1]=111;word[2]=112;word[3]=0;write(64,4);
							}
							else if(mixer_name==2)
							{
								word[0]=113;word[1]=114;word[2]=0;write(64,4);
							}
							else if(mixer_name==3)
							{
								send2('V',64,4);send3(hzk[115],72,4);
							}
							else if(mixer_name==4)
							{
								word[0]=116;word[1]=117;word[2]=112;word[3]=0;write(64,4);
							}
							
							
							
							if(line_2<1)
							{
								
								if(increase)
								{
									increase=0;
									if(mixer_name==1)
									{
										if(mixer1_same<7)mixer1_same++,Data_change(2);
									}
									else if(mixer_name==2)
									{
										if(mixer2_same<7)mixer2_same++,Data_change(2);
									}
									else if(mixer_name==3)
									{
										if(mixer3_same<7)mixer3_same++,Data_change(2);
									}
									else if(mixer_name==4)
									{
										if(mixer4_same<7)mixer4_same++,Data_change(2);
									}
								}
								if(decrease)
								{
									decrease=0;
									if(mixer_name==1)
									{
										if(mixer1_same>0)mixer1_same--,Data_change(2);
									}
									else if(mixer_name==2)
									{
										if(mixer2_same>0)mixer2_same--,Data_change(2);
									}
									else if(mixer_name==3)
									{
										if(mixer3_same>0)mixer3_same--,Data_change(2);
									}
									else if(mixer_name==4)
									{
										if(mixer4_same>0)mixer4_same--,Data_change(2);
									}
								}
								
							}
							else if(line_2<2)
							{
							if(increase)
								{
									increase=0;
									if(mixer_name==1)
									{
										if(mixer1_contrary<7)mixer1_contrary++,Data_change(2);
									}
									else if(mixer_name==2)
									{
										if(mixer2_contrary<7)mixer2_contrary++,Data_change(2);
									}
									else if(mixer_name==3)
									{
										if(mixer3_contrary<7)mixer3_contrary++,Data_change(2);
									}
									else if(mixer_name==4)
									{
										if(mixer4_contrary<7)mixer4_contrary++,Data_change(2);
									}
								}
								if(decrease)
								{
									decrease=0;
									if(mixer_name==1)
									{
										if(mixer1_contrary>0)mixer1_contrary--,Data_change(2);
									}
									else if(mixer_name==2)
									{
										if(mixer2_contrary>0)mixer2_contrary--,Data_change(2);
									}
									else if(mixer_name==3)
									{
										if(mixer3_contrary>0)mixer3_contrary--,Data_change(2);
									}
									else if(mixer_name==4)
									{
										if(mixer4_contrary>0)mixer4_contrary--,Data_change(2);
									}
								}													
								
							}
							else if(line_2<3)
							{
							if(right)
								{
									right=0;
									if(mixer_name==1)
									{
										mixer1_same=1;mixer1_contrary=0;Data_change(2);
									}
									else if(mixer_name==2)
									{
										mixer2_same=2;mixer2_contrary=3;Data_change(2);
									}
									else if(mixer_name==3)
									{
										mixer3_same=0;mixer3_contrary=3;Data_change(2);
									}
									else if(mixer_name==4)
									{
										mixer4_same=5;mixer4_contrary=0;Data_change(2);
									}
								}
							}	
								
									
							
							if(mixer_name==1)
							{
								send2(mixer1_same+49,60,0);
								send2(mixer1_contrary+49,60,2);
							}
							else if(mixer_name==2)
							{
								send2(mixer2_same+49,60,0);
								send2(mixer2_contrary+49,60,2);
							}
							else if(mixer_name==3)
							{
								send2(mixer3_same+49,60,0);
								send2(mixer3_contrary+49,60,2);
							}
							else if(mixer_name==4)
							{
								send2(mixer4_same+49,60,0);
								send2(mixer4_contrary+49,60,2);
							}
							
						}
						else if(menu_2==6)//失控舵量
						{
							line_2=move_cursor(line_2,7);
							
							WW[0][0]=29,WW[0][1]=30,WW[0][2]=81,WW[0][3]=0;
							WW[1][0]=29,WW[1][1]=30,WW[1][2]=82,WW[1][3]=0;
							WW[2][0]=29,WW[2][1]=30,WW[2][2]=83,WW[2][3]=0;
							WW[3][0]=29,WW[3][1]=30,WW[3][2]=84,WW[3][3]=0;
							WW[4][0]=29,WW[4][1]=30,WW[4][2]=85,WW[4][3]=0;
							WW[5][0]=29,WW[5][1]=30,WW[5][2]=86,WW[5][3]=0;
							WW[6][0]=29,WW[6][1]=30,WW[6][2]=87,WW[6][3]=0;
							WW[7][0]=29,WW[7][1]=30,WW[7][2]=88,WW[7][3]=0;
							display_menu(line_2,7);
							
							if(increase)
							{
								increase=0;
								if(out_control[line_2]<120)
								{
									out_control[line_2]++;Data_change(1);
									out_control_data[0]=(int)out_control[0]*4.25+511;
									out_control_data[1]=(int)out_control[1]*4.25+511;
									out_control_data[2]=(int)out_control[2]*4.25+511;
									out_control_data[3]=(int)out_control[3]*4.25+511;
									out_control_data[4]=(int)out_control[4]*4.25+511;
									out_control_data[5]=(int)out_control[5]*4.25+511;
									out_control_data[6]=(int)out_control[6]*4.25+511;
									out_control_data[7]=(int)out_control[7]*4.25+511;
									tx_order[1]=out_control_data[0]>>2;
									tx_order[2]=out_control_data[0]<<6;
									tx_order[2]+=out_control_data[1]>>4;
									tx_order[3]=out_control_data[1]<<4;
									tx_order[3]+=out_control_data[2]>>6;
									tx_order[4]=out_control_data[2]<<2;
									tx_order[4]+=out_control_data[3]>>8;
									tx_order[5]=out_control_data[3];

									tx_order[6]=out_control_data[4]>>2;
									tx_order[7]=out_control_data[4]<<6;
									tx_order[7]+=out_control_data[5]>>4;
									tx_order[8]=out_control_data[5]<<4;
									tx_order[8]+=out_control_data[6]>>6;
									tx_order[9]=out_control_data[6]<<2;
									tx_order[9]+=out_control_data[7]>>8;
									tx_order[10]=out_control_data[7];
									
									
									NRF_config_change=1;
								}
								
							}
							if(decrease)
							{
								decrease=0;
								if(out_control[line_2]>-120)
								{
									out_control[line_2]--;Data_change(1);
									out_control_data[0]=(int)out_control[0]*4.25+511;
									out_control_data[1]=(int)out_control[1]*4.25+511;
									out_control_data[2]=(int)out_control[2]*4.25+511;
									out_control_data[3]=(int)out_control[3]*4.25+511;
									out_control_data[4]=(int)out_control[4]*4.25+511;
									out_control_data[5]=(int)out_control[5]*4.25+511;
									out_control_data[6]=(int)out_control[6]*4.25+511;
									out_control_data[7]=(int)out_control[7]*4.25+511;
									tx_order[1]=out_control_data[0]>>2;
									tx_order[2]=out_control_data[0]<<6;
									tx_order[2]+=out_control_data[1]>>4;
									tx_order[3]=out_control_data[1]<<4;
									tx_order[3]+=out_control_data[2]>>6;
									tx_order[4]=out_control_data[2]<<2;
									tx_order[4]+=out_control_data[3]>>8;
									tx_order[5]=out_control_data[3];

									tx_order[6]=out_control_data[4]>>2;
									tx_order[7]=out_control_data[4]<<6;
									tx_order[7]+=out_control_data[5]>>4;
									tx_order[8]=out_control_data[5]<<4;
									tx_order[8]+=out_control_data[6]>>6;
									tx_order[9]=out_control_data[6]<<2;
									tx_order[9]+=out_control_data[7]>>8;
									tx_order[10]=out_control_data[7];
									
									
									NRF_config_change=1;
								}
								
							}
							
							if(line_2<3)
							{
								write_num100(out_control[0],70,0,'/');
								write_num100(out_control[1],70,2,'/');
								write_num100(out_control[2],70,4,'/');
								write_num100(out_control[3],70,6,'/');
							}
							else if(line_2==7)
							{
								write_num100(out_control[4],70,0,'/');
								write_num100(out_control[5],70,2,'/');
								write_num100(out_control[6],70,4,'/');
								write_num100(out_control[7],70,6,'/');
							}
							else
							{
								write_num100(out_control[line_2-2],70,0,'/');
								write_num100(out_control[line_2-1],70,2,'/');
								write_num100(out_control[line_2],70,4,'/');
								write_num100(out_control[line_2+1],70,6,'/');
							}
						}
						else if(menu_2==7)//通信密码
						{
							line_2=move_cursor(line_2,4);
							
							WW[0][0]=48,WW[0][1]=49,WW[0][2]=81,WW[0][3]=0;
							WW[1][0]=48,WW[1][1]=49,WW[1][2]=82,WW[1][3]=0;
							WW[2][0]=48,WW[2][1]=49,WW[2][2]=83,WW[2][3]=0;
							WW[3][0]=48,WW[3][1]=49,WW[3][2]=84,WW[3][3]=0;
							WW[4][0]=48,WW[4][1]=49,WW[4][2]=85,WW[4][3]=0;

							display_menu(line_2,4);
							if(line_2<4)
							{
								send2_hex((u8)(address[0]>>4),60,0);send2_hex((u8)(address[0]&0x0f),68,0);
								send2_hex((u8)(address[1]>>4),60,2);send2_hex((u8)(address[1]&0x0f),68,2);
								send2_hex((u8)(address[2]>>4),60,4);send2_hex((u8)(address[2]&0x0f),68,4);
								send2_hex((u8)(address[3]>>4),60,6);send2_hex((u8)(address[3]&0x0f),68,6);
							}
							else 
							{
								send2_hex((u8)(address[1]>>4),60,0);send2_hex((u8)(address[1]&0x0f),68,0);
								send2_hex((u8)(address[2]>>4),60,2);send2_hex((u8)(address[2]&0x0f),68,2);
								send2_hex((u8)(address[3]>>4),60,4);send2_hex((u8)(address[3]&0x0f),68,4);
								send2_hex((u8)(address[4]>>4),60,6);send2_hex((u8)(address[4]&0x0f),68,6);

							}
							
							if(increase)
							{
								increase=0;
								address[line_2]++;Data_change(1),NRF_change=1;
							}
							if(decrease)
							{
								decrease=0;
								address[line_2]--;Data_change(1),NRF_change=1;
							}
							
						}
						else if(menu_2==8)//跳频
						{
							line_2=move_cursor(line_2,4);
							
							WW[0][0]=53,WW[0][1]=51,WW[0][2]=81,WW[0][3]=0;
							WW[1][0]=53,WW[1][1]=51,WW[1][2]=82,WW[1][3]=0;
							WW[2][0]=53,WW[2][1]=51,WW[2][2]=83,WW[2][3]=0;
							WW[3][0]=53,WW[3][1]=51,WW[3][2]=84,WW[3][3]=0;
							WW[4][0]=53,WW[4][1]=51,WW[4][2]=85,WW[4][3]=0;
							display_menu(line_2,4);
							
							if(line_2<3)
							{
								write_num100(hopping[0],60,0,0);
								write_num100(hopping[1],60,2,0);
								write_num100(hopping[2],60,4,0);
								write_num100(hopping[3],60,6,0);
							}
							else 
							{
								write_num100(hopping[1],60,0,0);
								write_num100(hopping[2],60,2,0);
								write_num100(hopping[3],60,4,0);
								write_num100(hopping[4],60,6,0);
							}
							
							if(increase)
							{
								increase=0;
								if(hopping[line_2]<125)hopping[line_2]++;Data_change(1);
							}
							if(decrease)
							{
								decrease=0;
								if(hopping[line_2]>0)hopping[line_2]--;Data_change(1);
							}
							
						}
						else if(menu_2==9)//NRF发射映射
						{
							line_2=move_cursor(line_2,7);
							
							WW[0][0]=29,WW[0][1]=30,WW[0][2]=81,WW[0][3]=0;
							WW[1][0]=29,WW[1][1]=30,WW[1][2]=82,WW[1][3]=0;
							WW[2][0]=29,WW[2][1]=30,WW[2][2]=83,WW[2][3]=0;
							WW[3][0]=29,WW[3][1]=30,WW[3][2]=84,WW[3][3]=0;
							WW[4][0]=29,WW[4][1]=30,WW[4][2]=85,WW[4][3]=0;
							WW[5][0]=29,WW[5][1]=30,WW[5][2]=86,WW[5][3]=0;
							WW[6][0]=29,WW[6][1]=30,WW[6][2]=87,WW[6][3]=0;
							WW[7][0]=29,WW[7][1]=30,WW[7][2]=88,WW[7][3]=0;
							display_menu(line_2,7);
							
							word[0]=4;word[1]=94;word[2]=95;word[3]=0;write(60,0);
							write(60,2);write(60,4);write(60,6);
							
							if(increase)
							{
								increase=0;
								if(NRF_mapping[line_2]<7)NRF_mapping[line_2]++;Data_change(1);
							}
							if(decrease)
							{
								decrease=0;
								if(NRF_mapping[line_2]>0)NRF_mapping[line_2]--;Data_change(1);
							}
							
							if(line_2<3)
							{
								send2(NRF_mapping[0]+49,96,0);
								send2(NRF_mapping[1]+49,96,2);
								send2(NRF_mapping[2]+49,96,4);
								send2(NRF_mapping[3]+49,96,6);
							}
							else if(line_2<4)
							{
								send2(NRF_mapping[1]+49,96,0);
								send2(NRF_mapping[2]+49,96,2);
								send2(NRF_mapping[3]+49,96,4);
								send2(NRF_mapping[4]+49,96,6);
							}
							else if(line_2<5)
							{
								send2(NRF_mapping[2]+49,96,0);
								send2(NRF_mapping[3]+49,96,2);
								send2(NRF_mapping[4]+49,96,4);
								send2(NRF_mapping[5]+49,96,6);
							}
							else if(line_2<6)
							{
								send2(NRF_mapping[3]+49,96,0);
								send2(NRF_mapping[4]+49,96,2);
								send2(NRF_mapping[5]+49,96,4);
								send2(NRF_mapping[6]+49,96,6);
							}
							else 
							{
								send2(NRF_mapping[4]+49,96,0);
								send2(NRF_mapping[5]+49,96,2);
								send2(NRF_mapping[6]+49,96,4);
								send2(NRF_mapping[7]+49,96,6);
							}

						}
						else if(menu_2==10)//版本信息
						{
							photo(bmp0,4,0,120,3);
							send3(hzk[73],70,6);send3(hzk[74],82,6);
							send2('V',94,6);send2('3',102,6);send2('.',110,6);send2('1',118,6);
							
						}
						else if(menu_2==11)//对频
						{
							word[0]=121;word[1]=122;word[2]=43;word[3]=44;word[4]=123;word[5]=0;write(24,0);
							send2('.',84,0);send2('.',92,0);send2('.',100,0);
							send2('1',56,2),send2('5',64,2);
														
							device_connect();

							if(not_get)
							{
								word[0]=16;word[1]=47;word[2]=126;word[3]=127;word[4]=0;write(40,6);
								menu_2=0;
							}
							else
							{
								word[0]=16;word[1]=47;word[2]=124;word[3]=125;word[4]=0;write(40,6);
								menu_2=0;menu=0;star=1;
							}	
							delay_ms(500);

						}
						else if(menu_2==12)//恢复出厂
						{
							word[0]=106,word[1]=107,word[2]=32,word[3]=136,word[4]=0,write(40,2);
							word[0]=137,word[1]=109,word[2]=0,write(52,4);send2('?',80,4);
							if(right)
							{								
								IAP_CONTR=0x82;		//擦除所有数据
								EEPROM_clean(0);
								EEPROM_clean(2);
								IAP_CONTR=0;
								send3(hzk[3],80,4);
								word[0]=138,word[1]=139,word[2]=140,word[3]=57,word[4]=0,write(40,6);
								TR0=0,TR1=0,CCON=0;while(1);
							}
						}
						else if(menu_2==13)//正舵量
						{
							line_2=move_cursor(line_2,7);
							
							WW[0][0]=94,WW[0][1]=95,WW[0][2]=81,WW[0][3]=0;
							WW[1][0]=94,WW[1][1]=95,WW[1][2]=82,WW[1][3]=0;
							WW[2][0]=94,WW[2][1]=95,WW[2][2]=83,WW[2][3]=0;
							WW[3][0]=94,WW[3][1]=95,WW[3][2]=84,WW[3][3]=0;
							WW[4][0]=94,WW[4][1]=95,WW[4][2]=85,WW[4][3]=0;
							WW[5][0]=94,WW[5][1]=95,WW[5][2]=86,WW[5][3]=0;
							WW[6][0]=94,WW[6][1]=95,WW[6][2]=87,WW[6][3]=0;
							WW[7][0]=94,WW[7][1]=95,WW[7][2]=88,WW[7][3]=0;
							display_menu(line_2,7);
							
							if(increase)
							{
								increase=0;
								if(CH_ratio2[line_2]<120)
								{
									CH_ratio2[line_2]++;Data_change(2);
								}
								
							}
							if(decrease)
							{
								decrease=0;
								if(CH_ratio2[line_2]>0)
								{
									CH_ratio2[line_2]--;Data_change(2);
								}
								
							}
							
							if(line_2<3)
							{
								write_num100(CH_ratio2[0],70,0,'%');
								write_num100(CH_ratio2[1],70,2,'%');
								write_num100(CH_ratio2[2],70,4,'%');
								write_num100(CH_ratio2[3],70,6,'%');
							}
							else if(line_2==7)
							{
								write_num100(CH_ratio2[4],70,0,'%');
								write_num100(CH_ratio2[5],70,2,'%');
								write_num100(CH_ratio2[6],70,4,'%');
								write_num100(CH_ratio2[7],70,6,'%');
							}
							else
							{
								write_num100(CH_ratio2[line_2-2],70,0,'%');
								write_num100(CH_ratio2[line_2-1],70,2,'%');
								write_num100(CH_ratio2[line_2],70,4,'%');
								write_num100(CH_ratio2[line_2+1],70,6,'%');
							}
							send2('+',62,0);
							send2('+',62,2);
							send2('+',62,4);
							send2('+',62,6);
						}
						else if(menu_2==14)//偏好设置
						{
							line_2=move_cursor(line_2,2);
							
							WW[0][0]=62,WW[0][1]=63,WW[0][2]=64,WW[0][3]=13,WW[0][4]=0;
							WW[1][0]=60,WW[1][1]=61,WW[1][2]=58,WW[1][3]=59,WW[1][4]=0;
							WW[2][0]=56,WW[2][1]=57,WW[2][2]=58,WW[2][3]=59,WW[2][4]=0;
							
	//						WW[3][0]=69,WW[3][1]=70,WW[3][2]=71,WW[3][3]=72,WW[3][4]=0;
							
							
							
							display_menu(line_2,2);
							
							write_num100(LCD_vop,60,0,0);
							if(LCD_light)send3(hzk[58],68,2);
							else send3(hzk[59],68,2);
							if(Sound_switch)send3(hzk[58],68,4);
							else send3(hzk[59],68,4);
							
							if(line_2<1)
							{
								if(increase)
								{
									increase=0;
									if(LCD_vop<60)
									{
										LCD_vop++;Data_change(1);
										
										RS=0;
										LCD(0x81);
										LCD(LCD_vop);
										RS=1;
									}
								}
								
								if(decrease)
								{
									decrease=0;
									if(LCD_vop>10)
									{
										LCD_vop--;Data_change(1);
										
										RS=0;
										LCD(0x81);
										LCD(LCD_vop);
										RS=1;
									}
								}
								
								
							}
							else if(line_2<2)
							{
								if(increase|decrease)
								{
									increase=0;decrease=0;Data_change(1);

										LCD_light=!LCD_light;LEDA=LCD_light;
								

								}	
									
									
								
							}
							else if(line_2<3)
							{
								if(increase|decrease)
								{
									increase=0;decrease=0;
									
										Sound_switch=!Sound_switch;Data_change(1);
									
								}	
									
									
								
							}
							
//							else if(line_2<5)
//							{
//						
//							}
						
						}
						else if(menu_2==15)//PPM输出
						{
							line_2=move_cursor(line_2,7);
							
							WW[0][0]=31,WW[0][1]=32,WW[0][2]=81,WW[0][3]=0;
							WW[1][0]=31,WW[1][1]=32,WW[1][2]=82,WW[1][3]=0;
							WW[2][0]=31,WW[2][1]=32,WW[2][2]=83,WW[2][3]=0;
							WW[3][0]=31,WW[3][1]=32,WW[3][2]=84,WW[3][3]=0;
							WW[4][0]=31,WW[4][1]=32,WW[4][2]=85,WW[4][3]=0;
							WW[5][0]=31,WW[5][1]=32,WW[5][2]=86,WW[5][3]=0;
							WW[6][0]=31,WW[6][1]=32,WW[6][2]=87,WW[6][3]=0;
							WW[7][0]=31,WW[7][1]=32,WW[7][2]=88,WW[7][3]=0;
							display_menu(line_2,7);
							
							word[0]=4;word[1]=94;word[2]=95;word[3]=0;write(60,0);
							write(60,2);write(60,4);write(60,6);
							
							if(increase)
							{
								increase=0;
								if(PPM_mapping[line_2]<7)PPM_mapping[line_2]++;Data_change(1);
							}
							if(decrease)
							{
								decrease=0;
								if(PPM_mapping[line_2]>0)PPM_mapping[line_2]--;Data_change(1);
							}
							
							if(line_2<3)
							{
								send2(PPM_mapping[0]+49,96,0);
								send2(PPM_mapping[1]+49,96,2);
								send2(PPM_mapping[2]+49,96,4);
								send2(PPM_mapping[3]+49,96,6);
							}
							else if(line_2<4)
							{
								send2(PPM_mapping[1]+49,96,0);
								send2(PPM_mapping[2]+49,96,2);
								send2(PPM_mapping[3]+49,96,4);
								send2(PPM_mapping[4]+49,96,6);
							}
							else if(line_2<5)
							{
								send2(PPM_mapping[2]+49,96,0);
								send2(PPM_mapping[3]+49,96,2);
								send2(PPM_mapping[4]+49,96,4);
								send2(PPM_mapping[5]+49,96,6);
							}
							else if(line_2<6)
							{
								send2(PPM_mapping[3]+49,96,0);
								send2(PPM_mapping[4]+49,96,2);
								send2(PPM_mapping[5]+49,96,4);
								send2(PPM_mapping[6]+49,96,6);
							}
							else 
							{
								send2(PPM_mapping[4]+49,96,0);
								send2(PPM_mapping[5]+49,96,2);
								send2(PPM_mapping[6]+49,96,4);
								send2(PPM_mapping[7]+49,96,6);
							}
						}
						else if(menu_2==16)//IR输出
						{
/*							line_2=move_cursor(line_2,3);
							
							WW[0][0]=31,WW[0][1]=32,WW[0][2]=81,WW[0][3]=0;
							WW[1][0]=31,WW[1][1]=32,WW[1][2]=82,WW[1][3]=0;
							WW[2][0]=31,WW[2][1]=32,WW[2][2]=83,WW[2][3]=0;
							WW[3][0]=31,WW[3][1]=32,WW[3][2]=84,WW[3][3]=0;
							
							display_menu(line_2,3);
							*/
					
						}
						else if(menu_2==17)//原始数据
						{
							
							location(0,0);send('A');send('D');send('1');send(':');write_num1000(adc[7]);

							
							location(0,1);send('A');send('D');send('2');send(':');write_num1000(adc[6]);

							
							location(0,2);send('A');send('D');send('3');send(':');write_num1000(adc[4]);

							
							location(0,3);send('A');send('D');send('4');send(':');write_num1000(adc[5]);

							
							location(0,4);send('A');send('D');send('5');send(':');write_num1000(adc[3]);

							
							location(0,5);send('A');send('D');send('6');send(':');write_num1000(adc[2]);

							
							location(0,6);send('A');send('D');send('7');send(':');write_num1000(adc[1]);

							
							location(0,7);send('A');send('D');send('8');send(':');write_num1000(adc[0]);

							
														
						}
						else if(menu_2==18)//无线协议
						{
							line_2=move_cursor(line_2,2);
							
							WW[0][0]=65,WW[0][1]=79,WW[0][2]=50,WW[0][3]=53,WW[0][4]=51,WW[0][5]=0;
							WW[1][0]=66,WW[1][1]=67,WW[1][2]=65,WW[1][3]=53,WW[1][4]=51,WW[1][5]=0;
							WW[2][0]=66,WW[2][1]=67,WW[2][2]=50,WW[2][3]=53,WW[2][4]=51,WW[2][5]=0;
							
//							WW[3][0]=69,WW[3][1]=70,WW[3][2]=71,WW[3][3]=72,WW[3][4]=0,WW[3][5]=0;
							
							display_menu(line_2,2);
							
							if(right)
							{
								right=0;
								TX_protocol=line_2+1;Data_change(1);
							}
							
							if(TX_protocol==1)send3(hzk[3],80,0);
							if(TX_protocol==2)send3(hzk[3],80,2);
							if(TX_protocol==3)send3(hzk[3],80,4);
						}
						else if(menu_2==19)//全频扫描
						{
							
							word[0]=76;word[1]=53;word[2]=77;word[3]=78;word[4]=0;write(0,0);
							location(54,0);send('2');send('4');send('0');send('0');send('-');
							send('2');send('5');send('2');send('5');send('M');send('h');send('z');
							
							location(0,2);LCD(0xff);
							location(0,4);LCD(0xff);
							location(0,6);LCD(0xff);
							location(127,2);LCD(0xff);
							location(127,4);LCD(0xff);
							location(127,6);LCD(0xff);
							
							location(63,3);LCD(0x3f);LCD(0x3f);
							location(63,5);LCD(0x3f);LCD(0x3f);
							location(63,7);LCD(0x3f);LCD(0x3f);
							location(31,3);LCD(0x0f);
							location(31,5);LCD(0x0f);
							location(31,7);LCD(0x0f);
							location(95,3);LCD(0x0f);
							location(95,5);LCD(0x0f);
							location(95,7);LCD(0x0f);
							location(0,3);send('0');
							location(116,3);send('4');send('1');
							location(0,5);send('4');send('2');
							location(116,5);send('8');send('3');
										location(0,7);send('8');send('4');
							location(110,7);send('1');send('2');send('5');
						
							
							location(1,2);
							for(temp=0;temp<42;temp++)
							{
								
								if(CD_data[temp])LCD(0x81),LCD(0x81),LCD(0x81);
								else LCD(0xbd),LCD(0xbd),LCD(0xbd);
								
							}
							location(1,4);
							for(temp=42;temp<84;temp++)
							{
								
								if(CD_data[temp])LCD(0x81),LCD(0x81),LCD(0x81);
								else LCD(0xbd),LCD(0xbd),LCD(0xbd);
								
							}	
							location(1,6);
							for(temp=84;temp<126;temp++)
							{
								
								if(CD_data[temp])LCD(0x81),LCD(0x81),LCD(0x81);
								else LCD(0xbd),LCD(0xbd),LCD(0xbd);
								
							}
							
							if(left)left=0,menu_2=0,RC_check=0;
						}
						else if(menu_2==20)//摇杆校准
						{
							
							
							if(CH1_max<adc[6])CH1_max=adc[6];		 //获取最大值
							if(CH2_max<adc[7])CH2_max=adc[7];
							if(CH3_max<adc[4])CH3_max=adc[4];
							if(CH4_max<adc[5])CH4_max=adc[5];
							
							if(CH1_min>adc[6])CH1_min=adc[6];		 //获取最小值
							if(CH2_min>adc[7])CH2_min=adc[7];
							if(CH3_min>adc[4])CH3_min=adc[4];
							if(CH4_min>adc[5])CH4_min=adc[5];

							location(0,0);						  //显示数值
							send('C');send('H');send('1');send(' ');send('<');write_num1000(CH1_min);
							send(' ');send('-');send(' ');write_num1000(CH1_max);send('>');
							location(0,1);
							send('C');send('H');send('2');send(' ');send('<');write_num1000(CH2_min);
							send(' ');send('-');send(' ');write_num1000(CH2_max);send('>');
							location(0,2);
							send('C');send('H');send('3');send(' ');send('<');write_num1000(CH3_min);
							send(' ');send('-');send(' ');write_num1000(CH3_max);send('>');
							location(0,3);
							send('C');send('H');send('4');send(' ');send('<');write_num1000(CH4_min);
							send(' ');send('-');send(' ');write_num1000(CH4_max);send('>');

							if(right)
							{
								right=0;
								if(calibration_action)
								{
									calibration_ok=1;
								}
								else
								{
									calibration_action=1;
								}
							}
							
							if(calibration_action)
							{
								if(left)
								{
									left=0;
									calibration_action=0;calibration_ok=0;
								}	
								
								send3(hzk[137],60,6);send3(hzk[109],72,6);
								if(calibration_ok)
								{
									send3(hzk[3],84,6);
									ROM_time++;
									if(ROM_time>4)
									{
										ROM_time=0;
										
										if(512-CH1_min<CH1_max-512)CH1_offset=(512-CH1_min)/10;//获取最小偏差
										else CH1_offset=(CH1_max-512)/10;
										if(512-CH2_min<CH2_max-512)CH2_offset=(512-CH2_min)/10;
										else CH2_offset=(CH2_max-512)/10;
										if(512-CH3_min<CH3_max-512)CH3_offset=(512-CH3_min)/10;
										else CH3_offset=(CH3_max-512)/10;
										if(512-CH4_min<CH4_max-512)CH4_offset=(512-CH4_min)/10;
										else CH4_offset=(CH4_max-512)/10;
										
										calibration_action=0;calibration_ok=0;Data_system=1,ROM_enable=1;	
									}
									
								}
								else send2('?',84,6);
							}
								
							
																		
							
							
					

						}
						else if(menu_2==21)//PPM输入显示
						{
							location(0,0);send('I');send('N');send('1');
							location(0,1);send('I');send('N');send('2');
							location(0,2);send('I');send('N');send('3');
							location(0,3);send('I');send('N');send('4');
							location(0,4);send('I');send('N');send('5');
							location(0,5);send('I');send('N');send('6');
							location(0,6);send('I');send('N');send('7');
							location(0,7);send('I');send('N');send('8');
							
							location(20,0);LCD(0x3e);LCD(0x41);
							location(20,1);LCD(0x3e);LCD(0x41);
							location(20,2);LCD(0x3e);LCD(0x41);
							location(20,3);LCD(0x3e);LCD(0x41);
							location(20,4);LCD(0x3e);LCD(0x41);
							location(20,5);LCD(0x3e);LCD(0x41);
							location(20,6);LCD(0x3e);LCD(0x41);
							location(20,7);LCD(0x3e);LCD(0x41);
							
							location(124,0);LCD(0x41);LCD(0x3e);
							location(124,1);LCD(0x41);LCD(0x3e);
							location(124,2);LCD(0x41);LCD(0x3e);
							location(124,3);LCD(0x41);LCD(0x3e);
							location(124,4);LCD(0x41);LCD(0x3e);
							location(124,5);LCD(0x41);LCD(0x3e);
							location(124,6);LCD(0x41);LCD(0x3e);
							location(124,7);LCD(0x41);LCD(0x3e);
							
							location(22,0);
							for(temp=0;temp<102;temp++)
							{
								if(temp>PPM_DATA[0]/10)LCD(0x41);
								else LCD(0x5d);
							}
							location(22,1);
							for(temp=0;temp<102;temp++)
							{
								if(temp>PPM_DATA[1]/10)LCD(0x41);
								else LCD(0x5d);
							}
							location(22,2);
							for(temp=0;temp<102;temp++)
							{
								if(temp>PPM_DATA[2]/10)LCD(0x41);
								else LCD(0x5d);
							}
							location(22,3);
							for(temp=0;temp<102;temp++)
							{
								if(temp>PPM_DATA[3]/10)LCD(0x41);
								else LCD(0x5d);
							}
							location(22,4);
							for(temp=0;temp<102;temp++)
							{
								if(temp>PPM_DATA[4]/10)LCD(0x41);
								else LCD(0x5d);
							}
							location(22,5);
							for(temp=0;temp<102;temp++)
							{
								if(temp>PPM_DATA[5]/10)LCD(0x41);
								else LCD(0x5d);
							}
							location(22,6);
							for(temp=0;temp<102;temp++)
							{
								if(temp>PPM_DATA[6]/10)LCD(0x41);
								else LCD(0x5d);
							}
							location(22,7);
							for(temp=0;temp<102;temp++)
							{
								if(temp>PPM_DATA[7]/10)LCD(0x41);
								else LCD(0x5d);
							}
						}
						else if(menu_2==22)//舵量监视
						{
							location(0,0);send('C');send('H');send('1');
							location(0,1);send('C');send('H');send('2');
							location(0,2);send('C');send('H');send('3');
							location(0,3);send('C');send('H');send('4');
							location(0,4);send('C');send('H');send('5');
							location(0,5);send('C');send('H');send('6');
							location(0,6);send('C');send('H');send('7');
							location(0,7);send('C');send('H');send('8');
							
							location(20,0);LCD(0x3e);LCD(0x41);
							location(20,1);LCD(0x3e);LCD(0x41);
							location(20,2);LCD(0x3e);LCD(0x41);
							location(20,3);LCD(0x3e);LCD(0x41);
							location(20,4);LCD(0x3e);LCD(0x41);
							location(20,5);LCD(0x3e);LCD(0x41);
							location(20,6);LCD(0x3e);LCD(0x41);
							location(20,7);LCD(0x3e);LCD(0x41);
							
							location(124,0);LCD(0x41);LCD(0x3e);
							location(124,1);LCD(0x41);LCD(0x3e);
							location(124,2);LCD(0x41);LCD(0x3e);
							location(124,3);LCD(0x41);LCD(0x3e);
							location(124,4);LCD(0x41);LCD(0x3e);
							location(124,5);LCD(0x41);LCD(0x3e);
							location(124,6);LCD(0x41);LCD(0x3e);
							location(124,7);LCD(0x41);LCD(0x3e);
							
							location(22,0);
							for(temp=0;temp<102;temp++)
							{
								if(temp>operation[0]/10)LCD(0x41);
								else LCD(0x5d);
							}
							location(22,1);
							for(temp=0;temp<102;temp++)
							{
								if(temp>operation[1]/10)LCD(0x41);
								else LCD(0x5d);
							}
							location(22,2);
							for(temp=0;temp<102;temp++)
							{
								if(temp>operation[2]/10)LCD(0x41);
								else LCD(0x5d);
							}
							location(22,3);
							for(temp=0;temp<102;temp++)
							{
								if(temp>operation[3]/10)LCD(0x41);
								else LCD(0x5d);
							}
							location(22,4);
							for(temp=0;temp<102;temp++)
							{
								if(temp>operation[4]/10)LCD(0x41);
								else LCD(0x5d);
							}
							location(22,5);
							for(temp=0;temp<102;temp++)
							{
								if(temp>operation[5]/10)LCD(0x41);
								else LCD(0x5d);
							}
							location(22,6);
							for(temp=0;temp<102;temp++)
							{
								if(temp>operation[6]/10)LCD(0x41);
								else LCD(0x5d);
							}
							location(22,7);
							for(temp=0;temp<102;temp++)
							{
								if(temp>operation[7]/10)LCD(0x41);
								else LCD(0x5d);
							}
						}
						else if(menu_2==23)//自动扫描
						{
							line_2=move_cursor(line_2,2);
							
							WW[0][0]=172,WW[0][1]=51,WW[0][2]=0;
							WW[1][0]=173,WW[1][1]=51,WW[1][2]=0;
							WW[2][0]=114,WW[2][1]=103,WW[2][2]=0;
							display_menu(line_2,2);
							
							if(line_2<1)
							{
								if(increase)
								{
									increase=0;
									if(auto_star<120)auto_star++,Data_change(1);
									auto_min=auto_star*4.25+511;
								}
								if(decrease)
								{
									decrease=0;
									if(auto_star>-120)auto_star--,Data_change(1);
									auto_min=auto_star*4.25+511;
								}
							}
							else if(line_2<2)
							{
								if(increase)
								{
									increase=0;
									if(auto_end<120)auto_end++,Data_change(1);
									auto_max=auto_end*4.25+511;
	
								}
								if(decrease)
								{
									decrease=0;
									if(auto_end>-120)auto_end--,Data_change(1);
									auto_max=auto_end*4.25+511;
								}
							}
							else if(line_2<3)
							{
								if(increase)
								{
									increase=0;
									if(auto_speed<100)auto_speed++,Data_change(1);
								}
								if(decrease)
								{
									decrease=0;
									if(auto_speed>0)auto_speed--,Data_change(1);
								}
							}
							write_num100(auto_star,66,0,'/');
							write_num100(auto_end,66,2,'/');
							write_num100(auto_speed,66,4,0);
							
							
						}
						else if(menu_2==24)//电压设置
						{
							line_2=move_cursor(line_2,3);
							
							WW[0][0]=175,WW[0][1]=70,WW[0][2]=0;
							WW[1][0]=71,WW[1][1]=72,WW[1][2]=0;
							WW[2][0]=174,WW[2][1]=70,WW[2][2]=0;							
							WW[3][0]=39,WW[3][1]=40,WW[3][2]=0;
						
							display_menu(line_2,3);
							
							write_num100(battery_voltage/10,60,0,'.');send2('V',92,0);
							if(Battery_1s==1)
							{
								write_num100(VLD1,60,2,'.');send2('V',92,2);
							}
							else if(Battery_1s==0)
							{
								write_num100(VLD2,60,2,'.');send2('V',92,2);
							}
							
							
							
							if(line_2<1)
							{
								
							}
							else if(line_2<2)
							{

								if(increase)
								{
									increase=0;
									if(Battery_1s==1)
									{
										if(VLD1<40)VLD1++;Data_change(1);
									}
									else if(Battery_1s==0)
									{
										if(VLD2<120)VLD2++;Data_change(1);
									}
								}
								
								if(decrease)
								{
									decrease=0;
									if(Battery_1s==1)
									{
										if(VLD1>33)VLD1--;Data_change(1);
									}
									else if(Battery_1s==0)
									{
										if(VLD2>66)VLD2--;Data_change(1);
									}
								}
								
								
									
								
							}
							else if(line_2<3)
							{
									
								
							}
							else if(line_2<4)
							{
								if(increase)
								{
									increase=0;

								}

									
							}
						}
						else if(menu_2==25)//
						{
							
						}
						else if(menu_2==26)//
						{
							
						}
						
						if(left)left=0,menu_2=0;
						increase=0;decrease=0;right=0;
					}
					else 
					{

						if(menu_1==1)//工作模式
						{
							line_1=move_cursor(line_1,2);
							
							WW[0][0]=10,WW[0][1]=11,WW[0][2]=29,WW[0][3]=30,WW[0][4]=0;
							WW[1][0]=2,WW[1][1]=2,WW[1][2]=31,WW[1][3]=32,WW[1][4]=0;
							WW[2][0]=2,WW[2][1]=2,WW[2][2]=31,WW[2][3]=33,WW[2][4]=0;
							
							display_menu(line_1,2);
							send2('P',12,4);send2('P',20,4);send2('M',28,4);
							send2('P',12,2);send2('P',20,2);send2('M',28,2);
							
							if(line_1<1)//无线
							{								
								if(right)
								{
									right=0;
									if(RC_TX)
									{
										RC_TX=0;Data_change(1);
									}
									else
									{
										IR_TX=0,RC_TX=1;Data_change(1);
										
									}
								}
							}
							else if(line_1<2)//PPM
							{
								if(right)
								{
									right=0;
									if(PPM_OUT)
									{
										PPM_OUT=0;Data_change(1);
										ET1=0;
									}
									else
									{
										PPM_OUT=1,PPM_IN=0;config_change=1;Data_change(1);
										EX1=0;ET1=1;TMOD=0x11;
									}
								}
								 
							}
							else 
							{
								if(right)
								{
									right=0;
									if(PPM_IN)
									{
										PPM_IN=0;EX1=0;Data_change(1);
									}
									else
									{
										PPM_IN=1,PPM_OUT=0;config_change=1;Data_change(1);
										TMOD=0x91;
										EX1=1;ET1=0;
								//		P3M0=0x60;			 
										
									}
								}
								
							}
							if(RC_TX)send3(hzk[3],62,0);
							if(PPM_IN)send3(hzk[3],62,4);
							if(PPM_OUT)send3(hzk[3],62,2);
						}
						else if(menu_1==2)//快捷功能
						{
							line_1=move_cursor(line_1,5);
							if(line_2<1)
							{

								if(right)right=0,menu_2=1;
							}
							else if(line_2<2)
							{

								if(right)right=0,menu_2=1;
							}
							else if(line_2<3)
							{

								if(right)right=0,menu_2=1;
							}
							else if(line_2<4)
							{

								if(right)right=0,menu_2=1;
							}
							else if(line_2<5)
							{

								if(right)right=0,menu_2=1;
							}
							else if(line_2<6)
							{

								if(right)right=0,menu_2=1;
							}
							
						}
						else if(menu_1==3)//通道设置
						{
							line_1=move_cursor(line_1,5);
							
							WW[0][0]=34,WW[0][1]=35,WW[0][2]=0;
							WW[1][0]=39,WW[1][1]=40,WW[1][2]=0;
							WW[2][0]=34,WW[2][1]=36,WW[2][2]=37,WW[2][3]=0;
							WW[3][0]=134,WW[3][1]=36,WW[3][2]=37,WW[3][3]=0;
							WW[4][0]=31,WW[4][1]=33,WW[4][2]=38,WW[4][3]=30,WW[4][4]=0;
							WW[5][0]=143,WW[5][1]=144,WW[5][2]=118,WW[5][3]=11,WW[5][4]=0;
							display_menu(line_1,5);
							
							
							if(line_1<1)
							{
								if(right)right=0,menu_2=1,line_2=0;
							}
							else if(line_1<2)
							{
								if(right)right=0,menu_2=4,line_2=0;
							}
							else if(line_1<3)
							{
								if(right)right=0,menu_2=13,line_2=0;
							}
							else if(line_1<4)
							{
								if(right)right=0,menu_2=2,line_2=0;
							}
							else if(line_1<5)
							{
								if(right)right=0,menu_2=3,line_2=0;
							}
							else 
							{
								if(right)right=0,menu_2=0,line_2=0;
							}
						}
						else if(menu_1==4)//高级混控
						{
							line_1=move_cursor(line_1,3);
							
							WW[0][0]=21,WW[0][1]=22,WW[0][2]=80,WW[0][3]=81,WW[0][4]=0;
							WW[1][0]=21,WW[1][1]=22,WW[1][2]=80,WW[1][3]=82,WW[1][4]=0;
							WW[2][0]=21,WW[2][1]=22,WW[2][2]=80,WW[2][3]=83,WW[2][4]=0;
							WW[3][0]=21,WW[3][1]=22,WW[3][2]=80,WW[3][3]=84,WW[3][4]=0;
							display_menu(line_1,3);
							
							if(increase|decrease)
							{
								increase=0;decrease=0;
								if(line_1<1)mixer1=!mixer1,Data_change(2);
								else if(line_1<2)mixer2=!mixer2,Data_change(2);
								else if(line_1<3)mixer3=!mixer3,Data_change(2);
								else if(line_1<4)mixer4=!mixer4,Data_change(2);
							}
							
							if(mixer1)send3(hzk[58],75,0);
							else send3(hzk[59],75,0);
							if(mixer2)send3(hzk[58],75,2);
							else send3(hzk[59],75,2);
							if(mixer3)send3(hzk[58],75,4);
							else send3(hzk[59],75,4);
							if(mixer4)send3(hzk[58],75,6);
							else send3(hzk[59],75,6);
							if(right)right=0,menu_2=5,line_2=0,mixer_name=line_1+1;						
							
						}
						else if(menu_1==5)//无线配置
						{
							line_1=move_cursor(line_1,7);
							
							WW[0][0]=41,WW[0][1]=42,WW[0][2]=43,WW[0][3]=44,WW[0][4]=0;
							WW[1][0]=46,WW[1][1]=22,WW[1][2]=36,WW[1][3]=37,WW[1][4]=0;						
							WW[2][0]=16,WW[2][1]=47,WW[2][2]=48,WW[2][3]=49,WW[2][4]=0;
							WW[3][0]=50,WW[3][1]=51,WW[3][2]=52,WW[3][3]=53,WW[3][4]=0;
							WW[4][0]=76,WW[4][1]=53,WW[4][2]=77,WW[4][3]=78,WW[4][4]=0;
							WW[5][0]=29,WW[5][1]=30,WW[5][2]=38,WW[5][3]=30,WW[5][4]=0;
							WW[6][0]=16,WW[6][1]=47,WW[6][2]=119,WW[6][3]=120,WW[6][4]=0;
							WW[7][0]=29,WW[7][1]=30,WW[7][2]=27,WW[7][3]=45,WW[7][4]=0;
							display_menu(line_1,7);
							
							if(line_1<1)
							{

								if(right)right=0,menu_2=11,event2=1,connect_time=15;
							}
							else if(line_1<2)
							{

								if(right)right=0,menu_2=6,line_2=0;
							}
							else if(line_1<3)
							{

								if(right)right=0,menu_2=7,line_2=0;
							}
							else if(line_1<4)
							{

								if(right)right=0,menu_2=8,line_2=0;
							}
							else if(line_1<5)
							{

								if(right)
								{
									right=0;
									menu_2=19,line_2=0;
									RC_check=1,RC_check_star=1;
									
								}
								
							}
							else if(line_1<6)
							{
								if(right)right=0,menu_2=9,line_2=0;
							}
							else if(line_1<7)
							{
								if(right)right=0,menu_2=18,line_2=0;
							}
							else if(line_1<8)
							{
								if(increase)
								{
									increase=0;	   
									if(TX_power<3)TX_power++,Data_change(1),NRF_change=1;	
								}
								if(decrease)
								{
									decrease=0;
									if(TX_power>0)TX_power--,Data_change(1),NRF_change=1;
								}
								
								if(TX_power==3)
								{
									send2('1',60,6);send2('0',68,6);send2('0',76,6);send2('m',84,6);send2('W',92,6);
								}
								else if(TX_power==2)
								{
									send2('2',68,6);send2('5',76,6);send2('m',84,6);send2('W',92,6);
								}
								else if(TX_power==1)
								{
									send2('6',76,6);send2('m',84,6);send2('W',92,6);
								}
								else if(TX_power==0)
								{
									send2('2',76,6);send2('m',84,6);send2('W',92,6);
								}
								else 
								{
									send2('E',60,6);send2('R',68,6);send2('R',76,6);send2('O',84,6);send2('R',92,6);

								}
								
							}						
							
						}
						else if(menu_1==6)//红外配置
						{

						}
						else if(menu_1==7)//PPM配置
						{		
							
							line_1=move_cursor(line_1,3);
							
							WW[0][0]=31,WW[0][1]=32,WW[0][2]=38,WW[0][3]=30,WW[0][4]=0;
							WW[1][0]=31,WW[1][1]=33,WW[1][2]=167,WW[1][3]=168,WW[1][4]=0;
							WW[2][0]=54,WW[2][1]=55,WW[2][2]=160,WW[2][3]=161,WW[2][4]=0;
							WW[3][0]=54,WW[3][1]=55,WW[3][2]=58,WW[3][3]=59,WW[3][4]=0;
							display_menu(line_1,3);
							
							if(line_1<1)
							{
								if(right)right=0,menu_2=15,line_2=0;
							}
							else if(line_1<2)
							{
							if(right)right=0,menu_2=21,line_2=0;
							}
							else if(line_1<3)
							{
									
							}
							else 
							{
								if(increase)
								{
									increase=0;
									if(train_switch<1)train_switch=4,Data_change(1);
								
									
								}
								if(decrease)
								{
									decrease=0;
									if(train_switch>3)train_switch=0,Data_change(1);

								}

			
							}
							if(PPM_updata>0)
							{
								if(train_switch&&train_output)send3(hzk[58],72,4);
								else send3(hzk[59],72,4);	
							}
							else send3(hzk[10],72,4),send3(hzk[47],84,4),send3(hzk[68],96,4);
							
							display_switch(train_switch,72,6);

						}
						else if(menu_1==8)//系统设置
						{
							line_1=move_cursor(line_1,7);
							
							WW[0][0]=64,WW[0][1]=152,WW[0][2]=18,WW[0][3]=13,WW[0][4]=0;
							WW[1][0]=96,WW[1][1]=97,WW[1][2]=98,WW[1][3]=99,WW[1][4]=0;
							WW[2][0]=175,WW[2][1]=70,WW[2][2]=18,WW[2][3]=13,WW[2][4]=0;
							WW[3][0]=58,WW[3][1]=132,WW[3][2]=145,WW[3][3]=146,WW[3][4]=0;
							WW[4][0]=154,WW[4][1]=155,WW[4][2]=144,WW[4][3]=156,WW[4][4]=0;
							WW[5][0]=36,WW[5][1]=37,WW[5][2]=167,WW[5][3]=168,WW[5][4]=0;
							WW[6][0]=73,WW[6][1]=74,WW[6][2]=47,WW[6][3]=75,WW[6][4]=0;
							
							WW[7][0]=106,WW[7][1]=107,WW[7][2]=32,WW[7][3]=136,WW[7][4]=0;
							display_menu(line_1,7);
							
							if(line_1<1)
							{
								if(right)right=0,menu_2=14,line_2=0;
							}
							else if(line_1<2)
							{
								if(right)right=0,menu_2=20,line_2=0;
							}
							else if(line_1<3)
							{
								if(right)right=0,menu_2=24,line_2=0;
								
							}
							else if(line_1<4)
							{
								
							}
							else if(line_1<5)
							{
								if(right)right=0,menu_2=17,line_2=0;
									
							}
							else if(line_1<6)
							{
								if(right)right=0,menu_2=22,line_2=0;	
							}
							else if(line_1<7)
							{
								if(right)right=0,menu_2=10,line_2=0;
							}
							else if(line_1<8)
							{
								if(right)right=0,menu_2=12,line_2=0;

							}
							
						}
						else if(menu_1==9)//角度传感
						{
							
						}
						else if(menu_1==10)//自动控制
						{
							line_1=move_cursor(line_1,4);
							
							WW[0][0]=160,WW[0][1]=161,WW[0][2]=0;
							WW[1][0]=58,WW[1][1]=59,WW[1][2]=0;
							WW[2][0]=38,WW[2][1]=30,WW[2][2]=0;
							WW[3][0]=170,WW[3][1]=171,WW[3][2]=0;
							WW[4][0]=77,WW[4][1]=78,WW[4][2]=0;
							display_menu(line_1,4);
							
							if(line_1<3)
							{
								if(auto_switch)
								{
									if(auto_mode==1)send3(hzk[170],60,0),send3(hzk[171],72,0);
									else if(auto_mode==2)send3(hzk[59],60,0);	
									else if(auto_mode==3)send3(hzk[77],60,0),send3(hzk[78],72,0);
								}
								else send3(hzk[59],60,0);
							
								display_switch(auto_switch,60,2);
							}
							else display_switch(auto_switch,60,0);
							
							if(line_1<1)
							{
								
							}
							else if(line_1<2)
							{
								if(increase)
								{
									increase=0;
									if(auto_switch<1)auto_switch=3,Data_change(1);
									
								}
								if(decrease)
								{
									decrease=0;
									if(auto_switch==3)auto_switch=0,Data_change(1);

								}
							}
							else if(line_1<3)
							{
								if(increase)
								{
									increase=0;
									if(auto_mapping<7)auto_mapping++,Data_change(1);
								}
								if(decrease)
								{
									decrease=0;
									if(auto_mapping>0)auto_mapping--,Data_change(1);
								}
								send3(hzk[94],60,4);send3(hzk[95],72,4);send2(auto_mapping+49,84,4);
							}
							else if(line_1<4)
							{
								if(increase)
								{
									increase=0;
									if(auto_middle<120)auto_middle++,Data_change(1);
								}
								if(decrease)
								{
									decrease=0;
									if(auto_middle>-120)auto_middle--,Data_change(1);
								}
								write_num100(auto_middle,66,4,'/');
							}
							else 
							{
								if(right)right=0,menu_2=23,line_2=0;
							}
							
							
						}
						else if(menu_1==11)//模型管理
						{
							line_1=move_cursor(line_1,3);
							
							WW[0][0]=162,WW[0][1]=163,WW[0][2]=0;
							WW[1][0]=164,WW[1][1]=165,WW[1][2]=0;
							WW[2][0]=166,WW[2][1]=169,WW[2][2]=0;
							WW[3][0]=166,WW[3][1]=171,WW[3][2]=0;
							display_menu(line_1,3);
							
							
							send3(hzk[Model_number+81],60,0);send3(hzk[68],72,0);
							
							
							if(line_1<1)
							{
								if(increase)
								{
									increase=0;
									if(Model_number<4)
									{
										Model_current=Model_number;
										Model_number++;
										ROM_enable=1,Data_system=1,Model_change=1;
										
									}
								}
								if(decrease)
								{
									decrease=0;
									if(Model_number>0)
									{
										Model_current=Model_number;
										Model_number--;
										ROM_enable=1,Data_system=1,Model_change=1;
										
									}
								}
							}
							else if(line_1<2)
							{
								
							}
							else if(line_1<3)
							{
								if(increase)
								{
									increase=0;
									if(Model_target<4)Model_target++;
								}
								if(decrease)
								{
									decrease=0;
									if(Model_target>0)Model_target--;
								}
								
								if(right)
								{
									right=0;
									if(copy_ready)
									{
										paste_ready=1;
											
									}
									else 
									{
										copy_ready=1;
									}								
								}
								
								
								
								if(copy_ready)
								{
									if(left)
									{
										left=0;
										copy_ready=0;paste_ready=0;
									}


									send3(hzk[176],60,4);send3(hzk[Model_target+81],72,4);send3(hzk[68],84,4);
									if(paste_ready)
									{
										send3(hzk[3],96,4);
										ROM_time++;
										if(ROM_time>4)
										{
											ROM_time=0;
											Model_copy();
											Data_model=1,ROM_enable=1;	
											copy_ready=0;paste_ready=0;
										}
									}
									else send2('?',96,4);
								}
							}
							else if(line_1<4)
							{
								if(right)
								{
									right=0;
									if(reset_action)
									{
										Model_reset=1;
										
									}
									else
									{
										reset_action=1;
										
									}
									
								}
								
								
								
								
								if(reset_action)
								{
									if(left)
									{
										left=0;
										reset_action=0;Model_reset=0;
									}	
									
									send3(hzk[137],60,6);send3(hzk[109],72,6);
									if(Model_reset)
									{
										send3(hzk[3],84,6);
										ROM_time++;
										if(ROM_time>4)
										{
											ROM_time=0;
											Model_data_reset();
											reset_action=0;Model_reset=0;Data_model=1,ROM_enable=1;	
										}
										
									}
									else send2('?',84,6);
								}
							}
							
						}

						if(left)left=0,menu_1=0;
						increase=0;decrease=0;right=0;	
					}
				}
				else						 //主菜单
				{	

					if(star)			 
					{
						star=0;line=0;clean();
						up=0,down=0;
						
					}
					line=move_cursor(line,8);
					WW[0][0]=6,WW[0][1]=7,WW[0][2]=8,WW[0][3]=9,WW[0][4]=0;
					WW[1][0]=25,WW[1][1]=26,WW[1][2]=27,WW[1][3]=28,WW[1][4]=0;
					WW[2][0]=16,WW[2][1]=17,WW[2][2]=18,WW[2][3]=13,WW[2][4]=0;							
					WW[3][0]=19,WW[3][1]=20,WW[3][2]=21,WW[3][3]=22,WW[3][4]=0;
					WW[4][0]=10,WW[4][1]=11,WW[4][2]=12,WW[4][3]=13,WW[4][4]=0;					
					WW[5][0]=2,WW[5][1]=2,WW[5][2]=12,WW[5][3]=13,WW[5][4]=0;
					WW[6][0]=104,WW[6][1]=105,WW[6][2]=22,WW[6][3]=169,WW[6][4]=0;
					WW[7][0]=8,WW[7][1]=159,WW[7][2]=14,WW[7][3]=15,WW[7][4]=0;
					WW[8][0]=23,WW[8][1]=24,WW[8][2]=18,WW[8][3]=13,WW[8][4]=0;


					display_menu(line,8);
					
					if(line<1)		  
					{
						
						if(right)right=0,menu_1=1,line_1=0;
					}
					else if(line<2)
					{

						if(right)right=0,menu_1=0,line_1=0;
					}
					else if(line<3)
					{
					if(right)right=0,menu_1=3,line_1=0;	
					}
					else if(line<4)
					{
							if(right)right=0,menu_1=4,line_1=0;
					}
					else if(line<5)
					{
						if(right)right=0,menu_1=5,line_1=0;	
						send2('P',12,6);send2('P',20,6);send2('M',28,6);
					}		  
					else if(line<6) 
					{
						send2('P',12,4);send2('P',20,4);send2('M',28,4);
						if(right)right=0,menu_1=7,line_1=0;	
						
					}
					else if(line<7) 
					{
						send2('P',12,2);send2('P',20,2);send2('M',28,2);
						if(right)right=0,menu_1=10,line_1=0;
					}
					else if(line<8) 
					{
						send2('P',12,0);send2('P',20,0);send2('M',28,0);
						if(right)right=0,menu_1=11,line_1=0;
					}
					else  
					{
						send2('P',12,0);send2('P',20,0);send2('M',28,0);
						if(right)right=0,menu_1=8,line_1=0;
					}
					
					if(left)left=0,menu=0,line=0,star=1;
					if(increase)increase=0;
					if(decrease)decrease=0;

				}
			}
			else 
			{

				if(star)			 //star标志位：页面第一次载入
				{
					star=0;
					photo(bmp2,0,0,128,8);

				}	
				
				photo(name6,60,3,36,2);
				

				
				
	//			send2(':',72,0);
	//			send2(':',96,0);
				send2(hour/10+48,64,0);send2(hour%10+48,72,0);
				
				send2(minute/10+48,85,0);send2(minute%10+48,93,0);
				
				send2(second/10+48,106,0);send2(second%10+48,114,0);
				
				if(SA)send2(123,64,5);
				else send2(124,64,5);
				
				if(SB1&SB2)send2(126,72,5);
				else if(SB1&!SB2)send2(125,72,5);
				else send2(127,72,5);
				
				if(SC1&SC2)send2(126,80,5);
				else if(SC1&!SC2)send2(125,80,5);
				else send2(127,80,5);
				
				if(SD)send2(123,88,5);
				else send2(124,88,5);
/*				location(54,6);send('A');LCD(0);send('B');LCD(0);LCD(0);
				send('C');LCD(0);LCD(0);send('D');
				
				if(SA)send2(123,54,4);
				else send2(124,54,4);
				
				if(SB1&SB2)send2(126,62,4);
				else if(SB1&!SB2)send2(125,62,4);
				else send2(127,62,4);
				
				if(SC1&SC2)send2(126,70,4);
				else if(SC1&!SC2)send2(125,70,4);
				else send2(127,70,4);
				
				if(SD)send2(123,78,4);
				else send2(124,78,4);*/
				
/*				
				location(55,5);
				temp=operation[1];
	//			temp=angle_x;
				if(temp>0)send('+');
				else if(temp<0)temp=-temp,send('-');
				else send(' ');
				send(temp/1000+48);
				send(temp%1000/100+48);
				send(temp%100/10+48);
				send(temp%10+48);
				
				location(55,6);
				temp=PPM_DATA[2];
	//			temp=angle_y;
				if(temp>0)send('+');
				else if(temp<0)temp=-temp,send('-');
				else send(' ');
				send(temp/1000+48);
				send(temp%1000/100+48);
				send(temp%100/10+48);
				send(temp%10+48);
				

				
				location(95,5);temp=test;
				send(temp/1000+48);
				send(temp%1000/100+48);
				send(temp%100/10+48);
				send(temp%10+48);
				
				location(95,6);temp=boot_time;
				send(temp/1000+48);
				send(temp%1000/100+48);
				send(temp%100/10+48);
				send(temp%10+48);
*/
				
				
				
				location(78,2);temp=battery_voltage;
				if(temp>999)
				{
					temp/=10;
					send(temp%1000/100+48);				
					send(temp%100/10+48);LCD(0x00);LCD(0x40);
					send(temp%10+48);
				}
				else
				{
					send(temp%1000/100+48);LCD(0x00);LCD(0x40);				
					send(temp%100/10+48);
					send(temp%10+48);	
				}

				
				location(23,4);
				if(rx[1])
				{
					temp=rx[1];
					temp<<=8;
					temp+=rx[2];
				}
				else temp=0;
				send(temp%1000/100+48);LCD(0x00);LCD(0x40);				
				send(temp%100/10+48);
				send(temp%10+48);
				
				location(23,3);
				if(rx[3])
				{
					temp=rx[3];
					temp<<=8;
					temp+=rx[4];
				}
				else temp=0;
				if(temp>999)
				{
					temp/=10;
					send(temp%1000/100+48);				
					send(temp%100/10+48);LCD(0x00);LCD(0x40);
					send(temp%10+48);
				}
				else
				{
					send(temp%1000/100+48);LCD(0x00);LCD(0x40);				
					send(temp%100/10+48);
					send(temp%10+48);	
				}
				
				
				
				location(23,5);temp=rx[0]*2;
/*				send(temp%1000/100+48);				
				send(temp%100/10+48);
				send(temp%10+48);*/
				if(temp>99)send('1'),send('0'),send('0');
				else send(' '),send(temp/10+48),send(temp%10+48);
				
//				if(FUCK_sum==0)warning(50,6);
				
				location(23,6);temp=FUCK_sum*2;
				if(temp>99)send('1'),send('0'),send('0');
				else send(' '),send(temp/10+48),send(temp%10+48);
	//			send3(hzk[10],80,3);send3(hzk[11],92,3);send3(hzk[12],104,3);
	//			warning(60,3);
	//photo(name1,80,3,36,16);
				
				
				if(CH_negative[2])
				{
					throttle(255-CH_value[2]/4);
					location(6,2);
					temp=100-CH_value[2]*10/102;
				}
				else
				{
					throttle(CH_value[2]/4);
					location(6,2);
					temp=CH_value[2]*10/102;
				}
				
				
					if(temp==100)
					{
						send('A');send('L');send('L');
					}
					else
					{
						send(temp/10+48);send(temp%10+48);send('%');
					}
					
				location(6,7);
				if(CH_negative[3]==1)CH24_trim(100-CH_middle[3]);
				else CH24_trim(CH_middle[3]+100);
				location(67,7);
				if(CH_negative[1]==1)CH24_trim(100-CH_middle[1]);
				else CH24_trim(CH_middle[1]+100);

				if(CH_negative[2]==1)CH13_trim(100-CH_middle[2],0);
				else CH13_trim(CH_middle[2]+100,0);
				if(CH_negative[0]==1)CH13_trim(100-CH_middle[0],123);
				else CH13_trim(CH_middle[0]+100,123);
					
				if(event1)//微调更新
				{
										
						if(CH_middle_change==1)
						{
							
							photo(bmp3,90,2,32,3);
							location(93,3);
							temp=CH_middle[0];		
							
						}
						else if(CH_middle_change==2)
						{
							
							photo(bmp3,77,4,32,3);
							location(80,5);
							temp=CH_middle[1];	
						}
						else if(CH_middle_change==3)
						{
							
							photo(bmp3,5,2,32,3);
							location(8,3);
							temp=CH_middle[2];	
						}
						else if(CH_middle_change==4)
						{
							
							photo(bmp3,17,4,32,3);
							location(20,5);
							temp=CH_middle[3];	
						}					
						
						if(temp>0)send('+');
						else if(temp<0)temp=-temp,send('-');
						else send(' ');
						send(temp/100+48);
						send(temp%100/10+48);
						send(temp%10+48);						
					
				}
				
				LCD_OFF();
			}
		}
		else if(tt==3) //保留
		{
			
		}
		else if(tt==4)
		{
			if(ROM_enable)	//避免数据重复保存
			{
				ROM_enable=0;
				if(Data_system)		//系统参数
				{
					Data_system=0;
					DATA_save1();
				}
				if(Data_model) 	//机型参数
				{
					Data_model=0;
					DATA_save2();
				}
				
			}
			
			if(Model_change)	//读取另一个机型数据
			{
				Model_change=0;
				Model_current=Model_number;
				DATA_read2();
				
			}
		}
		else if(tt==5)//保留
		{
			tt=0;
		}
	

	}

}


void ET0_isr()interrupt 1	  //数据处理定时器	 低优先级
{

	TL0=0xe0,TH0=0xb1;		 //20ms
	flag_20ms=1;
//SA=!SA;SC2=!SC2;
	timer++;				   //闹钟处理
	if(timer>49)timer=0,second++;
	if(second>59)second=0,minute++;
	if(minute>59)minute=0,hour++;
	if(hour>99)hour=99;
	
	
	

	
	if(event1)		//1秒清0
	{
		t1++;
		if(t1>49)t1=0,event1=0,star=1;
	}
	
	if(event2)		//15秒清0
	{
		t2++;
		if(t2>49)
		{
			t2=0,connect_time--;

			if(connect_time>9)send2('1',56,2),send2(connect_time%10+48,64,2);
			else send2(' ',56,2),send2(connect_time+48,64,2);
		}
		if(connect_time==0)event2=0,connecting=0;
	}
	
	if(event3)		//1.5秒清0
	{
		t3++;
		if(t3>75)t3=0,event3=0,ROM_enable=1;//避免频繁擦写ROM
	}
	
/*	if(event3)		//0.5秒清0
	{
		t3++;
		if(t3>25)t3=0,event3=0,star=1;
	}*/
	
	if(Battery_1s)//低压报警处理
	{
		battery_voltage=adc[0]*0.654;		   
		if(battery_voltage<VLD1*10)beep=3;
		else{if(beep==3)beep=0;}
	}
	else
	{
		battery_voltage=adc[0]*1.46;
		if(battery_voltage<VLD2*10)beep=3;
		else{if(beep==3)beep=0;}
	}
	

	if(Sound_switch)//蜂鸣器处理
	{
		sound++;				   
		if(beep)
		{																				  
			if(beep==1)
			{ buzzer=0;if(sound>3)beep=0,buzzer=1;}
			if(beep==2)
			{buzzer=0;if(sound>30)beep=0,buzzer=1;}
			if(beep==3)
			{
				if(sound%40>25)buzzer=0;
				else buzzer=1;
			}
		}
		else sound=0,buzzer=1;	
	}
	else sound=0,buzzer=1;
	

/**************************************************************************/
//通道运算开关处理


//	operation[0]=(angle_x+511)*5/6-426+CH_middle[0];
//	operation[1]=(angle_y+511)*5/6-426+CH_middle[1];

	if(SA)Switch_A=1;
	else Switch_A=3;
	
	if(SB1&SB2)Switch_B=2;
	else if(SB1&!SB2)Switch_B=1;
	else Switch_B=3;

	if(SC1&SC2)Switch_C=2;
	else if(SC1&!SC2)Switch_C=1;
	else Switch_C=3;
	
	if(SD)Switch_D=1;
	else Switch_D=3;
	
	if(auto_switch)
	{
		auto_mode=Switch_C;
	}
	
	
	
	
	if(auto_mode==1)
	{
		auto_value=auto_middle*4.25+511;
	}
	else if(auto_mode==3)
	{
		if(auto_value>auto_max)auto_value=auto_max,auto_h=1,auto_l=0;
		if(auto_value<auto_min)auto_value=auto_min,auto_h=0,auto_l=1;
		if(auto_l)
		{
			auto_value+=auto_speed;
		}
		else if(auto_h)
		{
			auto_value-=auto_speed;
		}	
	}
	
	
	
	if(train_switch)
	{
		if(Switch_D==3)train_output=1;
		else train_output=0;	
	}
	
	CH_value[0]=CH_calib(adc11,CH1_offset);//俯仰adc1
	CH_value[1]=CH_calib(adc22,CH2_offset);//横滚adc2
	CH_value[2]=CH_calib(adc[4],CH3_offset);//油门adc[4]
	CH_value[3]=CH_calib(adc33,CH4_offset);//航向adc[5]
	
	function_mapping(0);
	function_mapping(1);
	function_mapping(2);
	function_mapping(3);
	function_mapping(4);
	function_mapping(5);
	function_mapping(6);
	function_mapping(7);
	
	
	
	function_size(0);
	function_size(1);
	function_size(2);
	function_size(3);
	function_size(4);
	function_size(5);
	function_size(6);
	function_size(7);
	
	
	
	function_negative(0);
	function_negative(1);
	function_negative(2);
	function_negative(3);
	function_negative(4);
	function_negative(5);
	function_negative(6);
	function_negative(7);
	

	
	if(mixer1)
	{
		function_mix(mixer1_same,mixer1_contrary);
	}
	if(mixer2)
	{
		function_mix(mixer2_same,mixer2_contrary);
	}
	if(mixer3)
	{
		function_mix(mixer3_same,mixer3_contrary);
	}
	if(mixer4)
	{
		function_mix(mixer4_same,mixer4_contrary);
	}
	
	
	
	
	operation[0]+=511;
	operation[1]+=511;
	operation[2]+=511;
	operation[3]+=511;
	operation[4]+=511;
	operation[5]+=511;
	operation[6]+=511;
	operation[7]+=511;
	
	if(auto_switch)
	{
		operation[auto_mapping]=auto_value;
	}
	
	Xdata_check(operation);
	
	
	NRF_buff[0]=operation[NRF_mapping[0]];
	NRF_buff[1]=operation[NRF_mapping[1]];
	NRF_buff[2]=operation[NRF_mapping[2]];
	NRF_buff[3]=operation[NRF_mapping[3]];
	NRF_buff[4]=operation[NRF_mapping[4]];
	NRF_buff[5]=operation[NRF_mapping[5]];
	NRF_buff[6]=operation[NRF_mapping[6]];
	NRF_buff[7]=operation[NRF_mapping[7]];
	
	if(PPM_updata>0)
	{
		PPM_updata--;
		if(train_output)
		{
			NRF_buff[0]=PPM_DATA[NRF_mapping[0]];
			NRF_buff[1]=PPM_DATA[NRF_mapping[1]];
			NRF_buff[2]=PPM_DATA[NRF_mapping[2]];
			NRF_buff[3]=PPM_DATA[NRF_mapping[3]];
			NRF_buff[4]=PPM_DATA[NRF_mapping[4]];
			NRF_buff[5]=PPM_DATA[NRF_mapping[5]];
			NRF_buff[6]=PPM_DATA[NRF_mapping[6]];
			NRF_buff[7]=PPM_DATA[NRF_mapping[7]];	
		}
		
	}
	
	PPMout[0]=operation[PPM_mapping[0]];
	PPMout[1]=operation[PPM_mapping[1]];
	PPMout[2]=operation[PPM_mapping[2]];
	PPMout[3]=operation[PPM_mapping[3]];
	PPMout[4]=operation[PPM_mapping[4]];
	PPMout[5]=operation[PPM_mapping[5]];
	PPMout[6]=operation[PPM_mapping[6]];
	PPMout[7]=operation[PPM_mapping[7]];

	
//operation[0]=adc[4];
//operation[1]=adc[5];	
//operation[2]=adc[6];
//operation[3]=adc[7];
/*
NRF_buff[0]=512;
NRF_buff[1]=512;
NRF_buff[2]=512;
NRF_buff[3]=512;
NRF_buff[4]=512;
NRF_buff[5]=512;
NRF_buff[6]=512;
NRF_buff[7]=512;*/


		tx_buff[0]=0xa1;
		tx_buff[1]=NRF_buff[0]>>2;
		tx_buff[2]=NRF_buff[0]<<6;
  	tx_buff[2]+=NRF_buff[1]>>4;
		tx_buff[3]=NRF_buff[1]<<4;
		tx_buff[3]+=NRF_buff[2]>>6;
		tx_buff[4]=NRF_buff[2]<<2;
		tx_buff[4]+=NRF_buff[3]>>8;
		tx_buff[5]=NRF_buff[3];

		tx_buff[6]=NRF_buff[4]>>2;
		tx_buff[7]=NRF_buff[4]<<6;
  	tx_buff[7]+=NRF_buff[5]>>4;
		tx_buff[8]=NRF_buff[5]<<4;
		tx_buff[8]+=NRF_buff[6]>>6;
		tx_buff[9]=NRF_buff[6]<<2;
		tx_buff[9]+=NRF_buff[7]>>8;
		tx_buff[10]=NRF_buff[7];

			
		tx_data[0]=tx_buff[0];
		tx_data[1]=tx_buff[1];
		tx_data[2]=tx_buff[2];
		tx_data[3]=tx_buff[3];
		tx_data[4]=tx_buff[4];
		tx_data[5]=tx_buff[5];
		tx_data[6]=tx_buff[6];
		tx_data[7]=tx_buff[7];
		tx_data[8]=tx_buff[8];
		tx_data[9]=tx_buff[9];
		tx_data[10]=tx_buff[10];
			
			
	

		
//SC2=!SC2;		

}

void EX1_isr()interrupt 2	  //脉宽测量	 高优先级
{
	u16 width;u8 i;
	width=TH1;
	width<<=8;
	width+=TL1;
	TL1=0,TH1=0;
	
	if(width<500)
	{
		t_ppm=0;
	}
	else if(width<1750)
	{
		PPM_buff[t_ppm]=width-605;
		t_ppm++;
		if(t_ppm>7)
		{
			t_ppm=0;
			if(PPM_updata<20)PPM_updata+=5;
			for(i=0;i<8;i++)
			PPM_DATA[i]=PPM_buff[i];
			Xdata_check(PPM_DATA);
		}
	}
	else
	{
		t_ppm=0;
	}

	
}

void ET1_isr()interrupt 3	  //脉宽输出	 高优先级
{
	u16 temp;	
	TR1=0;
	if(PPM_OUT)			  //输出8通道PPM，PPM信号总周期20ms，
	{	

							//每通道固定0.4ms低电平开始，0.6到1.6ms高电平结束
		t_PPMout++;
		switch(t_PPMout)
		{
			case 1:PPM=0;TL1=0x70,TH1=0xfe;temp=64935-PPMout[0];
							T_l=temp;T_h=temp>>8;break;				
			case 2:PPM=1;TL1=T_l;TH1=T_h;break;
			case 3:PPM=0;TL1=0x70,TH1=0xfe;temp=64935-PPMout[1];
							T_l=temp;T_h=temp>>8;break;				
			case 4:PPM=1;TL1=T_l;TH1=T_h;break;
			case 5:PPM=0;TL1=0x70,TH1=0xfe;temp=64935-PPMout[2];
							T_l=temp;T_h=temp>>8;break;				
			case 6:PPM=1;TL1=T_l;TH1=T_h;break;
			case 7:PPM=0;TL1=0x70,TH1=0xfe;temp=64935-PPMout[3];
							T_l=temp;T_h=temp>>8;break;				
			case 8:PPM=1;TL1=T_l;TH1=T_h;break;
			case 9:PPM=0;TL1=0x70,TH1=0xfe;temp=64935-PPMout[4];
							T_l=temp;T_h=temp>>8;break;				
			case 10:PPM=1;TL1=T_l;TH1=T_h;break;
			case 11:PPM=0;TL1=0x70,TH1=0xfe;temp=64935-PPMout[5];
							T_l=temp;T_h=temp>>8;break;				
			case 12:PPM=1;TL1=T_l;TH1=T_h;break;
			case 13:PPM=0;TL1=0x70,TH1=0xfe;temp=64935-PPMout[6];
							T_l=temp;T_h=temp>>8;break;				
			case 14:PPM=1;TL1=T_l;TH1=T_h;break;
			case 15:PPM=0;TL1=0x70,TH1=0xfe;temp=64935-PPMout[7];
							T_l=temp;T_h=temp>>8;break;				
			case 16:PPM=1;TL1=T_l;TH1=T_h;break;
			
			case 17:PPM=0;TL1=0x70,TH1=0xfe;break;				
			case 18:PPM=1;TL1=0xf0,TH1=0xd8;t_PPMout=0;break;
/*			
			case 13:PPM=0;TL1=0xe0,TH1=0xfc;break;				
			case 14:PPM=1;TL1=0x00,TH1=0xb0;t_PPMout=0;break;*/
			
		}
	}
	TR1=1;
}
void UART_SER (void) interrupt 4 //串行中断服务程序
{
    unsigned char Temp;          //定义临时变量 
   
   if(RI)                        //判断是接收中断产生
     {
			
	  RI=0;                      //标志位清零
	  Temp=SBUF;                 //读入缓冲区的值
			if(flydata==11)
			{
				switch(iflycount)
				{
					case 0:{adc1=(Temp-'0')*1000;iflycount++;break;}
					case 1:{adc1=adc1+(Temp-'0')*100;iflycount++;break;}
					case 2:{adc1=adc1+(Temp-'0')*10;iflycount++;break;}
					
					case 3:adc1=adc1+(Temp-'0');iflycount=0;flydata=0;adc11=adc1;break;
				}
				
				}
			if(flydata==22)
			{
				switch(iflycount)
				{
					case 0:{adc2=(Temp-'0')*1000;iflycount++;break;}
					case 1:{adc2=adc2+(Temp-'0')*100;iflycount++;break;}
					case 2:{adc2=adc2+(Temp-'0')*10;iflycount++;break;}
					case 3: adc2=adc2+(Temp-'0');iflycount=0;flydata=0;adc22=adc2;break;
				}
				}
			if(flydata==33)
			{
				switch(iflycount)
				{
					case 0:{adc3=(Temp-'0')*1000;iflycount++;break;}
					case 1:{adc3=adc3+(Temp-'0')*100;iflycount++;break;}
					case 2:{adc3=adc3+(Temp-'0')*10;iflycount++;break;}
					case 3:{adc3=adc3+(Temp-'0');iflycount=0;flydata=0;adc33=adc3;break;}
				}
				}
//			if(flydata==44)
//			{
//				switch(iflycount)
//				{
//					case 0:{adc4=(Temp-'0')*1000;iflycount++;break;}
//					case 1:{adc4=adc4+(Temp-'0')*100;iflycount++;break;}
//					case 2:{adc4=adc4+(Temp-'0')*10;iflycount++;break;}
//					case 3:{adc4=adc4+(Temp-'0');iflycount=0;flydata=0;adc44=adc4;break;}
//				}
//				}
			 switch(Temp)
			 {
				 case 'x':flydata=1;break;
				 case 'y':flydata=2;break;
				 case 'z':flydata=3;break;
//				 //case 'h':flydata=4;break;
				case ':':if(flydata==1)flydata=11;if(flydata==2)flydata=22;if(flydata==3)flydata=33;break;
//				 case ':':if(flydata==1)flydata=11;if(flydata==2)flydata=22;if(flydata==3)flydata=33;if(flydata==4)flydata=44;break;
			 }	
			
				                   //把值输出到P1口，用于观察
    //  SBUF=Temp;                 //把接收到的值再发回电脑端
	 }
   if(TI)                        //如果是发送标志位，清零
     TI=0;
}
void PCA_isr()interrupt 7	//任务定时器   中优先级
{  
	if(CCF0)				//电压采集
	{
		CCF0=0;
		CCAP0L = value0;
    	CCAP0H = value0>> 8;            
    	value0 += 3000;	   //3ms
			  
//		out=!out;
		t_adc++;
		switch(t_adc)	//读取上次转换结果,开始下次转换；
		{			 
			case 1:adc[0]=ADC_RES,adc[0]<<=2,adc[0]+=ADC_RESL;ADC_CONTR=0x89;break;
			case 2:adc[1]=ADC_RES,adc[1]<<=2,adc[1]+=ADC_RESL;ADC_CONTR=0x8a;break;
			case 3:adc[2]=ADC_RES,adc[2]<<=2,adc[2]+=ADC_RESL;ADC_CONTR=0x8b;break;
			case 4:adc[3]=ADC_RES,adc[3]<<=2,adc[3]+=ADC_RESL;ADC_CONTR=0x8c;break;
			case 5:adc[4]=ADC_RES,adc[4]<<=2,adc[4]+=ADC_RESL;ADC_CONTR=0x8d;break;
			case 6:adc[5]=ADC_RES,adc[5]<<=2,adc[5]+=ADC_RESL;ADC_CONTR=0x8e;break;
			case 7:adc[6]=ADC_RES,adc[6]<<=2,adc[6]+=ADC_RESL;ADC_CONTR=0x8f;break;
			case 8:adc[7]=ADC_RES,adc[7]<<=2,adc[7]+=ADC_RESL;ADC_CONTR=0x88;t_adc=0;break;
		 }

	}								   
	else					//实时任务  
	{	CCF1=0;
			
	  
		if(RC_check)		//全频扫描
		{
			CCAP1L = value1;
    	CCAP1H = value1>> 8;            
    	value1 += 1234;	   //2ms
			
			if(RC_check_star)		
			{
				RC_check_star=0;
				RX_mode();
			}
			
			t_check++;
			if(t_check>125)t_check=0;
			
			if(REG_read(0x09))CD_data[t_check]=1;
			else CD_data[t_check]=0;

			
			NRF_channel(t_check);
			
		}
		else if(RC_TX)			  //NRF定时发射信号
		{
			RC_TX_state++;
	
			if(RC_TX_state==1)//TX
			{
				CCAP1L = value1;
				CCAP1H = value1>> 8;            
				value1 += 2000;	   //2ms
				
				t_hopping++;
				if(t_hopping>4)t_hopping=0;
				NRF_channel(hopping[t_hopping]);//顺序跳频
				
				if(NRF_change)
				{
					NRF_change=0;
					NRF_power(TX_power);
					TX_address(address);
					RX_address(address);
				}

				if(NRF_config_change)
				{
					NRF_config_change=0;
					tx_order[0]=0xa0;
					FIFO_write2(tx_order,11);
				}
				else
				{

						FIFO_write2(tx_data,11);

				}
				
				
				
			}
			else if(RC_TX_state==2)//RX
			{
				CCAP1L = value1;
				CCAP1H = value1>> 8;            
				value1 += 16500;	   //17ms
				
				if(IRQ==0)
				{
					CE=0;
					REG_write(0x07,0x20);	//清除TX中断信号
					CE=1;	
				}
				
				RX_mode();
			}
			else if(RC_TX_state==3)
			{
				CCAP1L = value1;
				CCAP1H = value1>> 8;            
				value1 += 1500;	   //1ms
		
				RC_TX_state=0;
				if(IRQ==0)
				{
					FIFO_read2(rx,11);		//读取接收数据
					CE=0;
					REG_write(0x07,0x40);	//清除RX中断信号
					CE=1;	
					receive++;
					timer3=0;
				}
				else
				{
					timer3++;
					if(timer3>49)timer3=49,receive=0;
					
				}
				
				timer2++;
				if(timer2>49)timer2=0,FUCK_sum=receive,receive=0;
				TX_mode();	
				
			}

		}

		
	}	
} 
