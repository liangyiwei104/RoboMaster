#include "autoaim.h"

#include "main.h"
#include "INS_task.h"
#include "bsp_usart.h"
#include "string.h"
#include "shoot.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "referee.h"
#include "detect_task.h"
#include "cmsis_os.h"
#include "chassis_behaviour.h"
#include "stdio.h"
#define UART_DEBUG_MODE   0
//�ⲿ��������������
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern shoot_control_t shoot_control;
extern gimbal_control_t gimbal_control;
extern fp32 INS_angle[3];
extern fp32 INS_accel[3];
extern fp32 INS_gyro[3];
extern bool_t gimbal_cmd_to_chassis_stop(void);
extern int shoot_speed_add_little;

/***
���յ�8���ֽڣ�ǰ����'R','M';֮��Y���8λ��Y���8λ��֮��P���8λ��P���8λ����������ֽڿ���
��Y�ᣬ��P��
***/

//����
//void ChariotRecognition_Mes_Process(uint8_t *p);


/*--------------------------------��������-----------------------------------*/


__align(8) uint8_t USART1_TX_BUF[USART1_MAX_SEND_LEN]; 	 //���ͻ���,���USART1_MAX_SEND_LEN(400)�ֽ�
uint8_t USART1_RX_BUF[USART1_MAX_RECV_LEN];      //���ջ���,���USART_REC_LEN(400)���ֽ�.
uint16_t USART1_RX_STA=0;						 //����״̬���	

uint32_t usart1_this_time_rx_len = 0;              //USART1�յ������ݸ���
uint8_t ChariotRecognition_data[2][ChariotRecognition_data_dma_buf_len];   //DMA�����������ݵ�˫��������
int32_t ChariotRecognitionTemp[2] = {0,0};      //�����Ƕ����ݵ�����
int16_t ChariotRecognitionDirection[2];  //��������ͷ�����Ĵ�װ������
int16_t Chariot_Rec_Dir_rotate[2];       //��������ͷ�����Ĵ�װ�׺�Сװ������
int CameraDetectTarget_Flag = 0;     //����ͷ����Ŀ���־

int TempShootingFlag=0;      //������־,�޸ĸñ�־����ѡ���Ƿ񷢵�

float last_ChariotRecognition_pitch = 0.0f;        //��һ��pitch�Ƕ�ֵ
float ChariotRecognition_pitch = 0.0f;             //pitch�Ƕ�ֵ
float last_ChariotRecognition_yaw = 0.0f;          //��һ��yaw��Ƕ�ֵ
float ChariotRecognition_yaw = 0.0f;               //yaw�Ƕ�ֵ

float YawCurrentPositionSave   = 0.0f;     //���浱ǰYaw��λ��
float PitchCurrentPositionSave = 0.0f;     //���浱ǰPitch��λ��

uint16_t last_Target_Distance = 0;         //�ϴ�����ͷ��Ŀ��ľ���
uint16_t Target_Distance = 150;            //����ͷ��Ŀ��ľ���

uint16_t Distance_buf[10];     //���뻺����
uint8_t Dis_Buf_Index = 0;
uint8_t Pitch_Add_Angle = 0;
uint8_t enter_CNT = 0;
int Armor_R_Flag_Before=0;
int Armor_R_Flag_Behind=0;

int GM_Rotete_flag_Before=0;        //ǰ�̶�����ͷʶ��Ŀ��
int GM_Rotete_flag_Behind=0;        //��̶�����ͷʶ��Ŀ��
int Time_count=0;

CRringBuffer_t CR_ringBuffer;

float CR_yaw_Angle[20];
uint8_t CR_yaw_Angle_Index = 0;
uint8_t CR_yaw_Angle_CNT   = 0;
int8_t loop_j;

char Sendtosight[Sendtosight_len];            //���͸��Ӿ�

int friction_wheel_count = 0;
float kalman_yaw = 0;
float kalman_pitch = 0;
float kalman_yaw_feedforward = 0;
uint8_t update_flag = 1;
int Last_CameraDetectTarget_Flag=0;
float E_TEST=0;
float E_TEST1=0;
float E_TEST2=0;
float E_TEST3=0;
int camera_send_flag = 0;
int debug_flag =0;
int autoaim_mode_flag = 0;   //����״̬����/�رձ�־

/*----------------------��������-----------------------*/

/***********************************************
  ����1�жϷ�����
  ʹ��˫������ 
************************************************/
void USART1_IRQHandler(void)
{
	 if(huart1.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
		  	USART1_RX_STA =1;
    }

    else if(USART1->SR & UART_FLAG_IDLE)
    {
			  USART1_RX_STA = 2;
        static uint8_t this_time_rx_len = 0;
			
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = ChariotRecognition_data_dma_buf_len - hdma_usart1_rx.Instance->NDTR;//NDTR=15 buf_len =27,rxlen=12 Ȼ��Ͳ������� Ӧ�ð�buf�ĳ�30
//		  	printf("this_time_rx_len = %d\r\n",this_time_rx_len);
//		    printf("ChariotRecognition_data_dma_buf_len = %d\r\n",ChariotRecognition_data_dma_buf_len);
//		    printf("hdma_usart1_rx.Instance->NDTR = %d\r\n",hdma_usart1_rx.Instance->NDTR);

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart1_rx.Instance->NDTR = (uint16_t)ChariotRecognition_data_dma_buf_len;

            //set memory buffer 1
            //�趨������1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);
	   
            if(this_time_rx_len == ChariotRecognition_data_len )
            {
							USART1_RX_STA = 3;
							detect_hook(AIM_TOE);
							GetVisionData(&visionDataGet ,ChariotRecognition_data[0]);
							//ChariotRecognition_Mes_Process(ChariotRecognition_data[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = ChariotRecognition_data_dma_buf_len - hdma_usart1_rx.Instance->NDTR;//NDTR=21 

            //reset set_data_lenght
            //�����趨���ݳ���
           hdma_usart1_rx.Instance->NDTR = (uint16_t)ChariotRecognition_data_dma_buf_len;

            //set memory buffer 0
            //�趨������0
            hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == ChariotRecognition_data_len)
            {
							USART1_RX_STA = 3;
							detect_hook(AIM_TOE);
							GetVisionData(&visionDataGet ,ChariotRecognition_data[0]);
							//ChariotRecognition_Mes_Process(ChariotRecognition_data[1]);
							
            }
        }
    }

}

State_distance state_distacne=closedistance;

extern uint8_t get_robot_id(void);
extern uint8_t get_robot_level(void);

int count_Sendtosight = 0;
//extern int windmill_mode;
float yaw_add_little = 0;
float pitch_add_little = 0;

int shoot_num = 0;
float real_shoot_speed_save[5] = {13.6,13.6,13.6,13.6,13.6};
float remaining_shoot_num = 0;
float last_remaining_shoot_num = 0;
float average_shoot_speed = 13.6;


/***********************************************
       ��5�η�����ƽ��ֵ��ʵʱ����
***********************************************/
void shoot_speed_sending_culculate(void)
{
//	remaining_shoot_num = get_bullet_remaining_num();
//  remaining_shoot_num = get_bullet_speed();
//	if(last_remaining_shoot_num - remaining_shoot_num)
//	{
//		shoot_num++;
//	}
//	
//	if(shoot_num)
//	{
//		real_shoot_speed_save[shoot_num%5] = get_bullet_speed();
//	}
//	
//	average_shoot_speed = (real_shoot_speed_save[0] + real_shoot_speed_save[1] + real_shoot_speed_save[2] + real_shoot_speed_save[3] + real_shoot_speed_save[4]) / 5;	

	average_shoot_speed = get_bullet_speed();
}


VisionDataSend_Typedef visionDataSend;					    //���͸��Ӿ��������
VisionDataGet_Typedef  visionDataGet;               //���������Ӿ��������
uint8_t RXdata[15];                                 //�����Ӿ��������
/***********************************************
             �������ݳ�ʼ��
************************************************/
void SendVisionData_Init(void)
{
	uint8_t robot_id;
	robot_id = get_remain_HP();
	visionDataSend.head 					         = 0x53;		//֡ͷ ����涨0x53
	if(robot_id == 107) visionDataSend.color = 2;
	if(robot_id == 7)   visionDataSend.color = 1;
	visionDataSend.mode 					         = 0x01;		//ģʽ 0Ϊ���� 1ΪС�������� 2Ϊ���������� 3Ϊ����ڱ�ģʽ 4ΪС����ģʽ 5Ϊ¼��ģʽ 6Ϊ���˻�ģʽ 7Ϊ�ڱ�ģʽ 8Ϊ�״�ģʽ �������Ĭ��Ϊ����ģʽ
	visionDataSend.robotID 					       = 0x03;		//��ǰ������ID 0ΪӢ�� 1Ϊ���� 2Ϊ���� 3Ϊ���˻� 4Ϊ�ڱ�
	visionDataSend.yaw_angle.yaw_gyro   	 = 5 ;			//yaw�������ǽǶ�   ��ʼֵ
	visionDataSend.pitch_angle.pitch_gyro	 = 6.0;			//pitch�������ǽǶ� ��ʼֵ
	visionDataSend.yaw_acc_data.yaw_acc 	 = 0;				//yaw���ٶ�				 ��ʼֵ
	visionDataSend.pitch_acc_data.pitch_acc= 0;				//pitch���ٶ�			 ��ʼֵ
	visionDataSend.shoot_speed             = 99;		  //���跢���ٶ�			 ��ʼֵ
	visionDataSend.CRCcode                 = 0;       //CRCУ��
	visionDataSend.end 						         = 0x45;		//֡β ����涨0x45
}
/***********************************************
             �շ����ݵĴ��ڳ�ʼ��
************************************************/
void MyUART_Init(void)
{
	//HAL_UART _Receive_DMA(&huart6, umpireRxBuffer, UMPIRE_RX_BY_DMA_LENGTH	);		//���ڽ��� DMA���� ����ϵͳ����������
	//HAL_UART_Receive_DMA(&huart1,RXdata,15 );		//���ڽ��� DMA���� �Ӿ��鷢��������/***///��֪����ɶ��
	SendVisionData_Init();
}
/***********************************************
                ���Ӿ��������� 
************************************************/

void SendVisionData(VisionDataSend_Typedef* RAW_Data)		//���øú��������Ӿ���Ϣ���Ӿ���
{
	uint8_t data_buffer[14];
	static uint8_t delay_num;
	delay_num++;
	while(delay_num ==100)
	{
	data_buffer[0]  = RAW_Data->head;												//֡ͷ
	data_buffer[1]  = RAW_Data->color;								   	  //��ɫ 0Ϊ������ɫ 1Ϊ������ɫ 2Ϊ��������˫ɫ������ģʽ��
	data_buffer[2]  = RAW_Data->mode;												//ģʽ 0Ϊ���� 1ΪС�������� 2Ϊ���������� 3Ϊ����ڱ�ģʽ 4ΪС����ģʽ 5Ϊ¼��ģʽ 6Ϊ���˻�ģʽ 7Ϊ�ڱ�ģʽ 8Ϊ�״�ģʽ �������Ĭ��Ϊ����ģʽ
	data_buffer[3]  = RAW_Data->robotID;										//��ǰ������ID 0ΪӢ�� 1Ϊ���� 2Ϊ���� 3Ϊ���˻� 4Ϊ�ڱ�
  data_buffer[4]  = RAW_Data->yaw_angle.yaw_angle_send[0];
	data_buffer[5]  = RAW_Data->yaw_angle.yaw_angle_send[1];
//  data_buffer[4]  = (gimbal_control_set->gimbal_autshoot_motor.relative_angle>>8)&0xff;
//	data_buffer[5]  =(gimbal_control_set->gimbal_autshoot_motor.relative_angle)&0xff;		
//	data_buffer[6]  = (gimbal_control_set->gimbal_pitch_motor.relative_angle>>8)&0xff;
//	data_buffer[7]  = (gimbal_control_set->gimbal_pitch_motor.relative_angle)&0xff;
  data_buffer[6]  = RAW_Data->pitch_angle.pitch_angle_send[0];
	data_buffer[7]  = RAW_Data->pitch_angle.pitch_angle_send[1];		
	data_buffer[8]  = RAW_Data->yaw_acc_data.yaw_acc_send[0];
	data_buffer[9]  = RAW_Data->yaw_acc_data.yaw_acc_send[1];
	data_buffer[10] = RAW_Data->pitch_acc_data.pitch_acc_send[0];
	data_buffer[11] = RAW_Data->pitch_acc_data.pitch_acc_send[1];
	data_buffer[12] = RAW_Data->shoot_speed;
	data_buffer[13] = RAW_Data->end;								//֡β
	HAL_GPIO_TogglePin (LED_R_GPIO_Port, LED_R_Pin);
	HAL_UART_Transmit(&huart1, data_buffer, 15, 0xff);//ͨ��DMAһ���Է�������														//����API����
	delay_num =0;
	}
	
	
	
//#ifdef UART_DEBUG_MODE
//	printf("\n----------SendVisionData Start-------\n\n");
//	printf("head:%x  end: %x \n", RAW_Data->head, RAW_Data->end);
////	printf("color:%d  mode: %d ID: %d\n", RAW_Data->color, RAW_Data->mode, RAW_Data->MyID);
////	printf("yaw_Angle:%f  pitch_Angle: %f \n", RAW_Data->yaw_Angle, RAW_Data->pitch_Angle);
////	printf("yaw_velocity:%d  pitch_velocity: %d \n", RAW_Data->yaw_velocity, RAW_Data->pitch_velocity);
////	printf("bullet_velocity:%d  \n", RAW_Data->bullet_velocity);
//	printf("\n---------SendVisionData Finish-------\n");
//#endif
}
/***********************************************
    ������Ӿ����յ�����
    myVisionDataGet    ��   RAW_Data
************************************************/

void GetVisionData(VisionDataGet_Typedef* myVisionDataGet ,uint8_t RAW_Data[20])			//���øú�����ȡ�Ӿ���Ϣ���Ӿ���Ϣ�ᴫ�� �βνṹ�� ��
{
	if(RAW_Data[0]==0x55)
	{
	myVisionDataGet->head = RAW_Data[0];
	myVisionDataGet->detect_signal = RAW_Data[1];
	myVisionDataGet->shoot_msg = RAW_Data[2];
	myVisionDataGet->yaw_angle.yaw_angle_get[0] = RAW_Data[3];
	myVisionDataGet->yaw_angle.yaw_angle_get[1] = RAW_Data[4];
  myVisionDataGet->pitch_angle.pitch_angle_get[0] = RAW_Data[5];
	myVisionDataGet->pitch_angle.pitch_angle_get[1] = RAW_Data[6];
  myVisionDataGet->x_data.x_get[0] = RAW_Data[7];
	myVisionDataGet->x_data.x_get[1] = RAW_Data[8];
	myVisionDataGet->y_data.y_get[0] = RAW_Data[9];
	myVisionDataGet->y_data.y_get[1] = RAW_Data[10];
//	myVisionDataGet->CRCcode = RAW_Data[11];
//	myVisionDataGet->end = RAW_Data[12];
	myVisionDataGet->chassis_x.x_speed[0] = RAW_Data[11];
	myVisionDataGet->chassis_x.x_speed[1] = RAW_Data[12];
	myVisionDataGet->chassis_y.y_speed[0] = RAW_Data[13];
	myVisionDataGet->chassis_y.y_speed[1] = RAW_Data[14];
	myVisionDataGet->chassis_z.z_angle[0] = RAW_Data[15];
	myVisionDataGet->chassis_z.z_angle[1] = RAW_Data[16];
	myVisionDataGet->CRCcode = RAW_Data[17];
	myVisionDataGet->end = RAW_Data[18];
    if(RAW_Data[0] == 0)  myVisionDataGet->head = 0;
    if(RAW_Data[1] == 0)  myVisionDataGet->detect_signal = 0;
    if(RAW_Data[2] == 0)  myVisionDataGet->shoot_msg =0;
    if(RAW_Data[3] == 0)  myVisionDataGet->yaw_angle.yaw_angle_get[0] = 0;
    if(RAW_Data[4] == 0)  myVisionDataGet->yaw_angle.yaw_angle_get[1] = 0;
    if(RAW_Data[5] == 0)  myVisionDataGet->pitch_angle.pitch_angle_get[0] = 0;
    if(RAW_Data[6] == 0)  myVisionDataGet->pitch_angle.pitch_angle_get[1] = 0;
    if(RAW_Data[7] == 0)  myVisionDataGet->x_data.x_get[0] = 0;
    if(RAW_Data[8] == 0)  myVisionDataGet->x_data.x_get[1] = 0;
    if(RAW_Data[9] == 0)  myVisionDataGet->y_data.y_get[0] = 0;
    if(RAW_Data[10] == 0)  myVisionDataGet->y_data.y_get[1] = 0;
		if(RAW_Data[17] == 0)  myVisionDataGet->CRCcode = 0;
		
    // if(RAW_Data[11] == 0)  myVisionDataGet->CRCcode = 0;
    //if(RAW_Data[22] == 0)  myVisionDataGet->end = 0;
//  //  if(RAW_Data[11] == 0)  myVisionDataGet->depth_data.depth_get[0] = 0;
//   // if(RAW_Data[12] == 0)  myVisionDataGet->depth_data.depth_get[1] = 0;
    if(RAW_Data[11] == 0) myVisionDataGet->chassis_x.x_speed[0] = 0;
	  if(RAW_Data[12] == 0) myVisionDataGet->chassis_x.x_speed[1] = 0;
		if(RAW_Data[13] == 0) myVisionDataGet->chassis_y.y_speed[0] = 0;
	  if(RAW_Data[14] == 0) myVisionDataGet->chassis_y.y_speed[1] = 0;
		if(RAW_Data[15] == 0) myVisionDataGet->chassis_z.z_angle[0] = 0;
	  if(RAW_Data[16] == 0) myVisionDataGet->chassis_z.z_angle[1] = 0;

    if(RAW_Data[17] == 0)  myVisionDataGet->end = 0;
//	
   CameraDetectTarget_Flag = myVisionDataGet->detect_signal;
 }
	else
	{
		return;
	}
	
//	
//#ifdef UART_DEBUG_MODE
//	printf("\n------------GetVisionData-----------\n\n");
//	printf("head:%x  end: %x \n", myVisionDataGet->head, myVisionDataGet->end);
////	printf("VisionFlag:%x  VisionShoot: %x \n", myVisionDataGet->VisionFlag, myVisionDataGet->VisionShoot);
////	printf("yaw:%d  pitch: %d \n", myVisionDataGet->RES_Yaw, myVisionDataGet->RES_Pitch);
////	printf("depth:%x  QHH_CRC: %x \n", myVisionDataGet->depth, myVisionDataGet->QHH_CRC);
//	printf("\n---------GetVisionData Finish-------\n");
//#endif
}




/***********************************************
  * @brief          �����͵��Ӿ�������
  * @param[in]      VisionDataSend_Typedef
  * @retval         none
************************************************/



void Tidy_send_vision(VisionDataSend_Typedef *visionDataSend)
{
	uint8_t robot_id;
	robot_id=get_robot_id();
	shoot_speed_sending_culculate();
	if(robot_id <= 7)visionDataSend->color=0x02;
	else if(robot_id >=7)visionDataSend->color=0x01;
	//visionDataSend.color 					         = 0x02;		//��ʼ��ɫ 0Ϊ������ɫ 1Ϊ������ɫ 2Ϊ��������˫ɫ������ģʽ��
	//visionDataSend.mode 					         = 0x01;		//ģʽ 0Ϊ���� 1ΪС�������� 2Ϊ���������� 3Ϊ����ڱ�ģʽ 4ΪС����ģʽ 5Ϊ¼��ģʽ 6Ϊ���˻�ģʽ 7Ϊ�ڱ�ģʽ 8Ϊ�״�ģʽ �������Ĭ��Ϊ����ģʽ
	visionDataSend->yaw_angle.yaw_gyro       = (int16_t)(INS_angle[0] *57.32f* 100);
//	visionDataSend->pitch_angle.pitch_gyro   = (int16_t)(INS_angle[2] *57.32f* 100);
	visionDataSend->pitch_angle.pitch_gyro  = (int16_t)(gimbal_control.gimbal_pitch_motor.relative_angle *57.32f* 100);
	
	visionDataSend->yaw_acc_data.yaw_acc 	 = (int16_t)(INS_gyro[0] * 100);
	visionDataSend->pitch_acc_data.pitch_acc = (int16_t)(INS_gyro[2] * 100);
//	visionDataSend->shoot_speed              = (uint8_t)average_shoot_speed*10;
	  visionDataSend->CRCcode                 = 0;       //CRCУ��;
}
