#ifndef AUTOAIM_H
#define AUTOAIM_H

#include "bsp_usart.h"
#include "usart.h"
#include "gimbal_task.h"

#define USART1_MAX_RECV_LEN		400					//�����ջ����ֽ���
#define USART1_MAX_SEND_LEN		400					//����ͻ����ֽ���

extern uint8_t  USART1_RX_BUF[USART1_MAX_RECV_LEN];
extern uint16_t USART1_RX_STA;
extern int Armor_R_Flag_Before;
extern int Armor_R_Flag_Behind;
extern int autoaim_mode_flag;

#define ChariotRecognition_data_dma_buf_len   42u
#define ChariotRecognition_data_len           12u//����
#define A_Size  60
#define Dis_buf_Size 10

#define Sendtosight_len 14

#define BUFFER_SIZE 5

#include "stm32f4xx.h"
#include "struct_typedef.h"

//***************************************************vision_uart***************************************//
/* Ҫ���͸��Ӿ������Ϣ ��Ϣһ��15���ַ� */

typedef struct {
	uint8_t head;       //֡ͷ
	uint8_t color;      //��ɫ
	uint8_t mode;       //ģʽ
	uint8_t robotID;    //������ID
	
	union yaw_INS {     //YAW������������ ����λ���ȷ��ͣ�
		int16_t yaw_gyro;
		uint8_t yaw_angle_send[2];
	}yaw_angle;
	union pitch_INS {   //PITCH������������ ����λ���ȷ��ͣ�
		int16_t pitch_gyro;
		uint8_t pitch_angle_send[2];
	}pitch_angle;
	union yaw_Acc {     //YAW�������Ǽ��ٶȣ���λ���ȷ��ͣ�
		int16_t yaw_acc;
		uint8_t yaw_acc_send[2];
	}yaw_acc_data;
	union pitch_Acc {   //PITCH�������Ǽ��ٶ� ����λ���ȷ��ͣ�
		int16_t pitch_acc;
		uint8_t pitch_acc_send[2];
	}pitch_acc_data;
	
	uint8_t shoot_speed;//�ӵ��ٶ�
	uint8_t CRCcode;    //CRCУ�� 8У��
	uint8_t end;        //β֡
}VisionDataSend_Typedef;
//������������ݾ�Ϊfloat����ԭʼ����*100��ת����int16_t

/* ���Ӿ����ȡ����Ϣ ��Ϣһ��14���ַ� */
typedef struct {
	uint8_t head;              //ͷ֡[0]
	uint8_t detect_signal;     //װ�װ�����or�Ƿ�ʶ����������װ�װ�[1]
	uint8_t shoot_msg;         //�������� 0 or 1[2]
	union yaw_coordinate {     //YAW������������λ���ȷ��ͣ�[3...4]
		int16_t yaw_predict;
		uint8_t yaw_angle_get[2];
	}yaw_angle;
	
	union pitch_coordinate {   //PITCH������������λ���ȷ��ͣ�[5...6]
		int16_t pitch_predict;
		uint8_t pitch_angle_get[2];
	}pitch_angle;
	
	union x_predict {          //Ԥ������x ����λ���ȷ��ͣ�[7...8]
		int16_t x;
		uint8_t x_get[2];
	}x_data;
	
	union y_predict {          //Ԥ������y ����λ���ȷ��ͣ�[9...10]
		int16_t y;
		uint8_t y_get[2];
	}y_data;
	union chassis_x_speed{   //x�����ٶ�
		int16_t x_speed_order;
		int8_t x_speed[2];
	} chassis_x;
	
	union chassis_y_speed{  //y�����ٶ�
		int16_t y_speed_order;
		int8_t y_speed[2];
	} chassis_y;
	
		union chassis_z_angle{ //z�Ƕ�
		int16_t z_angle_order;
		int8_t z_angle[2];
	} chassis_z;
	uint8_t CRCcode;           //CRCУ�� 8[13]
	uint8_t end;               //β֡ [14]
}VisionDataGet_Typedef;






extern VisionDataSend_Typedef visionDataSend;					    //���͸��Ӿ��������
extern VisionDataGet_Typedef  visionDataGet;               //���������Ӿ��������
extern uint8_t RXdata[15];                               //�о�ûɶ��   /***/
void SendVisionData(VisionDataSend_Typedef* RAW_Data);
extern void GetVisionData(VisionDataGet_Typedef* myVisionDataGet ,uint8_t RAW_Data[18]);
void Tidy_send_vision(VisionDataSend_Typedef *visionDataSend);

//***************************************************vision_uart end***************************************//

typedef enum
{
  closedistance,
	middledistance,
	longdistance
}State_distance;
extern State_distance state_distacne;

typedef struct CRringBuffer_t
{
	float ringBuf[BUFFER_SIZE];
	int16_t tailPosition;
	float lineBuf[BUFFER_SIZE-1];
	float errBuf[BUFFER_SIZE-2];
	float err_Average;
	float predict_Val;
	int16_t lost_COUNT;
	int16_t out_Point;
}CRringBuffer_t;

extern uint8_t ChariotRecognition_data[2][ChariotRecognition_data_dma_buf_len];
extern uint32_t usart1_this_time_rx_len;
extern float last_ChariotRecognition_pitch;
extern float ChariotRecognition_pitch;
extern float last_ChariotRecognition_yaw;
extern float ChariotRecognition_yaw;

extern uint16_t last_Target_Distance;
extern uint16_t Target_Distance;
extern uint16_t Distance_Buf[Dis_buf_Size];
extern uint8_t Dis_Buf_Index;
extern uint8_t Pitch_Add_Angle;
extern CRringBuffer_t CR_ringBuffer;
extern int  GM_Rotete_flag_Before;
extern int  GM_Rotete_flag_Behind;
extern int camera_send_flag;

extern float CR_yaw_Angle[20];
extern uint8_t CR_yaw_Angle_Index;
extern uint8_t CR_yaw_Angle_CNT;
extern int Last_CameraDetectTarget_Flag;
extern int CameraDetectTarget_Flag;
extern float yaw_add_little;
extern float pitch_add_little;
extern int count_Sendtosight;

void Sendtosightway(int value);
//void usart1_Init(u32 bound);
//void ChariotRecognition_Mes_Process(uint8_t *p);

extern void shoot_speed_sending_culculate(void);
extern void SendVisionData_Init(void);
extern void MyUART_Init(void);
extern void SendVisionData(VisionDataSend_Typedef* RAW_Data);

extern void Tidy_send_vision(VisionDataSend_Typedef *visionDataSend);
#endif
