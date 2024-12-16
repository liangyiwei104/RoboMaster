/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.���̹��ʿ���
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT 
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             ֻ����80w���ʣ���Ҫͨ�����Ƶ�������趨ֵ,������ƹ�����40w������
  *             JUDGE_TOTAL_CURRENT_LIMIT��POWER_CURRENT_LIMIT��ֵ�����е�������ٶ�
  *             (����max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "remote_control.h"
#include "chassis_task.h"

#define SUPER_PEWER_KEY KEY_PRESSED_OFFSET_SHIFT  //�������ݰ���C

#define POWER_LIMIT_level_0         40.0f

#define POWER_LIMIT_level_1_power_priority    60.f   //��������
#define POWER_LIMIT_level_2_power_priority    80.f
#define POWER_LIMIT_level_3_power_priority    100.f

#define POWER_LIMIT_level_1_blood_priority    45.0f
#define POWER_LIMIT_level_2_blood_priority    50.0f
#define POWER_LIMIT_level_3_blood_priority    55.0f

#define POWER_LIMIT_level_1_balance_chassis   60.0f
#define POWER_LIMIT_level_2_balance_chassis   80.0f
#define POWER_LIMIT_level_3_balance_chassis   100.0f

#define POWER_LIMIT_1V1_game  120.0f  //120.0f     60.0f

#define WARNING_POWER_0       35.0f   
#define WARNING_POWER_1       55.0f
#define WARNING_POWER_2       75.0f
#define WARNING_POWER_3       90.0f

#define WARNING_POWER_1V1_game   100.0f//50.0f

#define WARNING_POWER_BUFF  45.0f 
#define WARNING_SUPERCAP_VOTALGE 18.0f

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    	 32000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      	 16000.0f   //16000
#define POWER_TOTAL_CURRENT_LIMIT          20000.0f   //20000.0
#define FLYING_BUFFER_TOTAL_CURRENT_LIMIT  35000.0f
#define SUPERCAP_TOTAL_CURRENT_LIMIT       20000.0f
#define FLYING_SUPERCAP_TOTAL_CURRENT_LIMIT     35000.0f

extern uint8_t get_robot_level(void);
extern uint8_t get_game_type(void);
extern chassis_move_t chassis_move;
fp32 capacitor_power = 0;
uint8_t flying_flag = 0;
uint8_t open_power_flag = 0;
static uint8_t last_open_power_key = 0;
uint16_t key_cnt = 0;
fp32 flying_buff_flag = 0;
/**
  * @brief          limit the power, mainly limit motor current
  * @param[in]      chassis_power_control: chassis data 
  * @retval         none
  */
/**
  * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
  * @param[in]      chassis_power_control: ��������
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_power_control)
{
    fp32 chassis_power = 0.0f;
    fp32 chassis_power_buffer = 0.0f;
    fp32 total_current_limit = 0.0f;
    fp32 total_current = 0.0f;
	  fp32 add_power_limit = 0.0f;
	//�ж��Ƿ�����������,���ΰ�����
//	  	if((chassis_move.chassis_RC->key.v & SUPER_PEWER_KEY) && !last_open_power_key)
//			{
//				key_cnt++;
//			}
//			if(key_cnt%2)
//			{
//				if(open_power_flag == 0)
//				{
//				  open_power_flag = 1;
//				}
//			}
//			else if(!key_cnt%2)
//			{
//				 if(open_power_flag == 1)
//				{
//				  open_power_flag = 0;
//				}
//			}
//			if(chassis_move.chassis_RC->rc.ch[4]==660)
//			{
//				open_power_flag = 1;
//			}
//			else
//			{
//				open_power_flag = 0;
//			}
//			last_open_power_key = chassis_move.chassis_RC->key.v & SUPER_PEWER_KEY;
//		
//     //һֱ��

//			if(chassis_move.chassis_RC->key.v & SUPER_PEWER_KEY)
//			{
//				open_power_flag = 1;
//		  }
//			else if(!chassis_move.chassis_RC->key.v & SUPER_PEWER_KEY)
//			{
//				open_power_flag = 0;
////				flying_flag = 0;
//			}
	
    uint8_t robot_id = get_robot_id();
    if(toe_is_error(REFEREE_TOE))
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
    else if(robot_id == RED_ENGINEER || robot_id == BLUE_ENGINEER || robot_id == 0)
    {
        total_current_limit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
    }
   // else if(chassis_move.chassis_power_data->CapVot)
		//�򿪳�������
		else if (chassis_move.chassis_power_data->CapVot) 
    {
         get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
        // power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
        //���ʳ���80w �ͻ�������С��60j,��Ϊ��������С��60��ζ�Ź��ʳ���80w
			 if(chassis_power_buffer == 250)
				{
					 flying_buff_flag = 1;
				}
				
		 	 if(chassis_move.chassis_power_data->CapVot < 20)
        {
            fp32 power_scale;
					
						if(chassis_move.chassis_power_data->CapVot > 14)
						{
								//scale down WARNING_POWER_BUFF
								//��СWARNING_POWER_BUFF
								power_scale = chassis_move.chassis_power_data->CapVot/ 7 / WARNING_POWER_BUFF;
						}
						else
						{
								//only left 10% of WARNING_POWER_BUFF
								power_scale = 0.4f / WARNING_POWER_BUFF;
						}
//						if(chassis_move.chassis_power_data->CapVot < 5)
//						{
//						  	open_power_flag = 0;
//						}
            //scale down
            //��С
				    if(flying_flag)
						{
							total_current_limit = FLYING_BUFFER_TOTAL_CURRENT_LIMIT * power_scale + POWER_TOTAL_CURRENT_LIMIT;
						}
            else 
						{	
					   	total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale + POWER_TOTAL_CURRENT_LIMIT ;        
						}
					}			
		 else
				{
						if(flying_flag)
						{
							total_current_limit = FLYING_BUFFER_TOTAL_CURRENT_LIMIT+ POWER_TOTAL_CURRENT_LIMIT;
						}					
						else total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT ; 
				}
		}
		//�޳�������
	else
		{
        get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);//����ϵͳ��ȡ���ʼ���������
  			if(chassis_power_buffer < WARNING_POWER_BUFF)//�����������Լ��õ�һ���֣���ʱһ��������
				{
						fp32 power_scale;
						if(chassis_power_buffer > 30.0f)
						{
								//scale down WARNING_POWER_BUFF
								//��СWARNING_POWER_BUFF
								power_scale = chassis_power_buffer / 60;   //45
						}
						else
						{
								//only left 10% of WARNING_POWER_BUFF
								power_scale = 4.0f / WARNING_POWER_BUFF;
						}
						//scale down
						//��С
						total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale*0.7f;		//����ֵ16000��3508����ź�			
				}
			 else//û�����ʣ�������Ҳ��ʼ����
			 {
				 if(chassis_power > get_chassis_power_limit()*9/10)   //0.9
				 {
							fp32 power_scale;
				
							if(chassis_power < get_chassis_power_limit())
							{                    
									//��С
									power_scale = (( get_chassis_power_limit() - chassis_power)/(get_chassis_power_limit() *1/10));                    
							}
							else
							{
								 power_scale = 0.0f;
							}
							total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT*0.7f + POWER_TOTAL_CURRENT_LIMIT * power_scale;									
				 }
				 else
				 {
							total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT*0.7f + POWER_TOTAL_CURRENT_LIMIT ;							 
				 }				
			}		
		}
    
    total_current = 0.0f;
    //calculate the original motor current set
    //����ԭ����������趨
    for(uint8_t i = 0; i < 4; i++)
    {
        total_current += fabs(chassis_power_control->motor_speed_pid[i].out);
    }
    

    if(total_current > total_current_limit)
    {
        fp32 current_scale = total_current_limit / total_current;
        chassis_power_control->motor_speed_pid[0].out*=current_scale;
        chassis_power_control->motor_speed_pid[1].out*=current_scale;
        chassis_power_control->motor_speed_pid[2].out*=current_scale;
        chassis_power_control->motor_speed_pid[3].out*=current_scale;
    }
}
