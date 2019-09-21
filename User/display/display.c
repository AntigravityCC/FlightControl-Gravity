
#include "include.h" 

/**
 * @brief 根据匿名最新上位机协议写的显示姿态的程序
 * @attention 具体协议说明请查看上位机软件的帮助说明。
 */
  
 
/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：data_transfer.c
 * 描述    ：数据传输
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/


dt_flag_t f;					//需要发送数据的标志
uint8_t data_to_send[50];	//发送数据缓存


uint8_t TxBuffer[256];
uint8_t TxCounter = 0;
uint8_t count = 0;

uint8_t Rx_Buf[256];	//串口接收缓存
void Usart1_Send ( unsigned char *DataToSend , uint8_t data_num )
{
    uint8_t i;
    for ( i = 0; i < data_num; i++ )
    {
        TxBuffer[count++] = * ( DataToSend + i );
			  USART1->DR = TxBuffer[TxCounter++];
    }
  
//    if ( ! ( USART1->CR1 & USART_CR1_TXEIE ) )
//    {
//        USART_ITConfig ( USART1, USART_IT_TXE, ENABLE ); //打开发送中断
//    }

}

//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length)
{
	for(uint8_t i=0;i<length;i++)
	rt_device_write(serial1, 0, dataToSend+i, (sizeof*(dataToSend+i)));

//#ifdef ANO_DT_USE_USB_HID
//	Usb_Hid_Adddata(data_to_send,length);
//#endif
//#ifdef ANO_DT_USE_USART2
//	Usart1_Send(data_to_send, length);
//#endif
//  USART_SendData(USART1, (uint16_t)data_to_send);
	
}

int32_t ParValList[100];
uint16_t paraToSend;
void ANO_DT_SendCmd(uint8_t dest, uint8_t fun, uint16_t cmd1, uint16_t cmd2, uint16_t cmd3, uint16_t cmd4, uint16_t cmd5)
{
	uint8_t _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=dest;
	data_to_send[_cnt++]=0xE0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=fun;
	data_to_send[_cnt++]=BYTE1(cmd1);
	data_to_send[_cnt++]=BYTE0(cmd1);
	data_to_send[_cnt++]=BYTE1(cmd2);
	data_to_send[_cnt++]=BYTE0(cmd2);
	data_to_send[_cnt++]=BYTE1(cmd3);
	data_to_send[_cnt++]=BYTE0(cmd3);
	data_to_send[_cnt++]=BYTE1(cmd4);
	data_to_send[_cnt++]=BYTE0(cmd4);
	data_to_send[_cnt++]=BYTE1(cmd5);
	data_to_send[_cnt++]=BYTE0(cmd5);
	
	data_to_send[4] = _cnt-5;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_SendParame(uint16_t num)
{
	uint8_t _cnt=0;
	int32_t data;
//	if(num > PARNUM)
//		return;
//	ANO_DT_ParUsedToParList();
	data = ParValList[num];
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0xE1;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(num);
	data_to_send[_cnt++]=BYTE0(num);
	data_to_send[_cnt++]=BYTE3(data);
	data_to_send[_cnt++]=BYTE2(data);
	data_to_send[_cnt++]=BYTE1(data);
	data_to_send[_cnt++]=BYTE0(data);
	
	data_to_send[4] = _cnt-5;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}


void ANO_DT_GetParame(uint16_t num,int32_t data)
{
//	if(num > PARNUM)
//		return;
	ParValList[num] = data;
//	ANO_DT_ParListToParUsed();
	paraToSend = num;	//将接收到的参数发回上位机进行双向验证
//	data_save();
}

static uint8_t DT_RxBuffer[256],DT_data_cnt = 0,ano_dt_data_ok;


void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num)
{
	uint8_t sum = 0;
	for(uint8_t i=0;i<(num-1);i++)
		sum += *(data_buf+i);
//	if(!(sum==*(data_buf+num-1)))	
//	{
//		rt_kprintf("sum cheak failed\r\n");   //判断sum
//		return;		
//	}
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))	//判断帧头
	{
		rt_kprintf("head cheak failed\r\n");
		return;		
	}
	if(*(data_buf+2)==MYHWADDR)
	{
		if(*(data_buf+3)==0XE0)			//命令E0
		{
			switch(*(data_buf+5))		//FUN
			{
				case 0x01:
//					if(*(data_buf+6)==0x00 && *(data_buf+7)==0x01)
//						sensor.acc_CALIBRATE = 1;
//					if(*(data_buf+6)==0x00 && *(data_buf+7)==0x02)
//						sensor.gyr_CALIBRATE = 1;
//					if(*(data_buf+6)==0x00 && *(data_buf+7)==0x04)
//						mag.mag_CALIBRATE = 1;
//					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xB0)//读取版本信息
//						f.send_version = 1;
					break;
				case 0x02:
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xAA)	//恢复默认PID
					{
//						PID_Rest();
//						All_PID_Init();
//						data_save();
					}
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xAB)	//恢复默认参数
					{
//						Parame_Reset();
//						data_save();
					}
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xAF)	//恢复所有参数
					{
//						PID_Rest();
//						All_PID_Init();
//						Parame_Reset();
//						data_save();
					}
					break;
				case 0xE1:
					paraToSend = (uint16_t)(*(data_buf+6)<<8)|*(data_buf+7);	//读取参数
					break;
				case 0x10:
//					FlyCtrlDataAnl(data_buf+5);
					break;
				case 0x11:
					break;
				default:
					break;
			}
			ANO_DT_SendCmd(SWJADDR,*(data_buf+5),(uint16_t)(*(data_buf+6)<<8)|*(data_buf+7),(uint16_t)(*(data_buf+8)<<8)|*(data_buf+9),(uint16_t)(*(data_buf+10)<<8)|*(data_buf+11),(uint16_t)(*(data_buf+12)<<8)|*(data_buf+13),(uint16_t)(*(data_buf+14)<<8)|*(data_buf+15));
		}
		else if(*(data_buf+3)==0XE1)	//设置参数
		{
			uint16_t _paraNum = (uint16_t)(*(data_buf+5)<<8)|*(data_buf+6);
			int32_t _paraVal = (int32_t)(((*(data_buf+7))<<24) + ((*(data_buf+8))<<16) + ((*(data_buf+9))<<8) + (*(data_buf+10)));
			ANO_DT_GetParame(_paraNum,_paraVal);
		}
	}
	ANO_DT_SendParame(0);
//	ANO_DT_SendParame(paraToSend);
}

void ANO_DT_Data_Receive_Anl_Task(void)
{
	
	if(ano_dt_data_ok)
	{
		ANO_DT_Data_Receive_Anl(DT_RxBuffer,DT_data_cnt+6);
		ano_dt_data_ok = 0;
	}
}

void ANO_DT_Data_Receive_Prepare(uint8_t data)
{
	static uint8_t _data_len = 0;
	static uint8_t state = 0;
	
	if(state==0&&data==0xAA)	//帧头0xAA
	{
		state=1;
		DT_RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)	//数据源，0xAF表示数据来自上位机
	{
		state=2;
		DT_RxBuffer[1]=data;
	}
	else if(state==2)		//数据目的地
	{
		state=3;
		DT_RxBuffer[2]=data;
	}
	else if(state==3)		//功能字
	{
		state=4;
		DT_RxBuffer[3]=data;
	}
	else if(state==4)		//数据长度
	{
		state = 5;
		DT_RxBuffer[4]=data;
		_data_len = data;
		DT_data_cnt = 0;
	}
	else if(state==5&&_data_len>0)
	{
		_data_len--;
		DT_RxBuffer[5+DT_data_cnt++]=data;
		if(_data_len==0)
			state = 6;
	}
	else if(state==6)
	{
		state = 0;
		DT_RxBuffer[5+DT_data_cnt]=data;
		ano_dt_data_ok = 1;//ANO_DT_Data_Receive_Anl(DT_RxBuffer,DT_data_cnt+5);
	}
	else
		state = 0;
}



void ANO_DT_Send_Version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver)
{
	uint8_t _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
	uint8_t _cnt=0;
	int16_t _temp;
	int32_t _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[4] = _cnt-5;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
//	USART_SendData(USART1,data_to_send[i]);
}
void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,int32_t bar)
{
	uint8_t _cnt=0;
	int16_t _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[4] = _cnt-5;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6)
{
	uint8_t _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[4] = _cnt-5;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Power(uint16_t votage, uint16_t current)
{
	uint8_t _cnt=0;
	uint16_t temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8)
{
	uint8_t _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
		data_to_send[4] = _cnt-5;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	uint8_t _cnt=0;
	int16_t _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}



/**
 * @brief 根据匿名最新上位机协议写的显示姿态的程序
 * @attention 具体协议说明请查看上位机软件的帮助说明。
 */
void Send_TO_ANO_TC1(int data1, int data2 ,int data3)
{
	unsigned char i=0;
	unsigned char _cnt=0,sum = 0;

	uint8_t data_to_send[50];

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;     //字节总数 后续会计算
	

	data_to_send[_cnt++]=BYTE1(data1);
	data_to_send[_cnt++]=BYTE0(data1);
    
	data_to_send[_cnt++]=BYTE1(data2);
	data_to_send[_cnt++]=BYTE0(data2);
    
	data_to_send[_cnt++]=BYTE1(data3);
	data_to_send[_cnt++]=BYTE0(data3);
	
	data_to_send[3] = _cnt-4;
	//和校验
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	//串口发送数据
	ANO_DT_Send_Data(data_to_send, _cnt);

//	Usart_SendByte(USART1,data_to_send[i]);
//		usart_send_char(data_to_send[i]);
}

void Send_TO_ANO_TC1_V65(int16_t data1, int16_t data2 ,int16_t data3)
{
	unsigned char i=0;
	unsigned char _cnt=0,sum = 0;

	uint8_t data_to_send[50];

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0xf1;
	data_to_send[_cnt++]=0;     //字节总数 后续会计算
	

	data_to_send[_cnt++]=BYTE1(data1);
	data_to_send[_cnt++]=BYTE0(data1);
    
	data_to_send[_cnt++]=BYTE1(data2);
	data_to_send[_cnt++]=BYTE0(data2);
    
	data_to_send[_cnt++]=BYTE1(data3);
	data_to_send[_cnt++]=BYTE0(data3);
	
	data_to_send[4] = _cnt-5;
	//和校验
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	//串口发送数据
	ANO_DT_Send_Data(data_to_send, _cnt);

//	Usart_SendByte(USART1,data_to_send[i]);
//		usart_send_char(data_to_send[i]);
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

/*********************************************END OF FILE**********************/
