#ifndef __DISPLAY_H
#define	__DISPLAY_H

#include "stm32f4xx.h"

/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

#define MYHWADDR	0x05
#define SWJADDR		0xAF



typedef struct 
{
		uint8_t send_version;
		uint8_t send_status;
		uint8_t send_senser;
		uint8_t send_pid1;
		uint8_t send_pid2;
		uint8_t send_pid3;
		uint8_t send_pid4;
		uint8_t send_pid5;
		uint8_t send_pid6;
		uint8_t send_rcdata;
		uint8_t send_offset;
		uint8_t send_motopwm;
		uint8_t send_power;

}dt_flag_t;

extern dt_flag_t f;

//void ANO_DT_Data_Exchange(void);
//void ANO_DT_Data_Receive_Prepare(uint8_t data);
//void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num);

void ANO_DT_Send_Version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed);
void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,int32_t bar);
void ANO_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6);
void ANO_DT_Send_Power(uint16_t votage, uint16_t current);
void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8);
void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum);
void Send_TO_ANO_TC1_V65(int16_t data1, int16_t data2 ,int16_t data3);
void Send_TO_ANO_TC1(int data1, int data2 ,int data3);
void ANO_DT_Data_Receive_Anl_Task(void);
void ANO_DT_Data_Receive_Anl(uint8_t *data_buf,uint8_t num);
void ANO_DT_SendParame(uint16_t num);
#endif 
