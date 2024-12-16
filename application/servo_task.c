/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"
#include "referee.h"

#define SERVO_MIN_PWM 1050;   
                              
#define SERVO_MAX_PWM 2000; 

#define PWM_DETAL_VALUE 60   //10

#define SERVO1_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Q
#define SERVO2_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Q
#define SERVO3_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Q
#define SERVO4_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Q

#define SERVO_MINUS_PWM_KEY KEY_PRESSED_OFFSET_E

const RC_ctrl_t *servo_rc;
const static uint16_t servo_key[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};
uint16_t servo_pwm[4] ;

int last_s_servor = 0;
int s_servor = 0;
int servor_key_count=0;
int servor_open_flag = 0;
int servor_real_open_ui = 0;
int last_servor_open_flag = 0;

/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          舵机任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{

    servo_rc = get_remote_control_point();
	
    while(1)
    {
				s_servor = servo_rc->rc.s[0];
			
				if((switch_is_mid(s_servor)&&switch_is_down(last_s_servor)))//(servor_key_count%2)||  !(servor_key_count%2)||
				{
						servor_key_count++;
				}
				else if((switch_is_up(s_servor)&&switch_is_mid(last_s_servor)))
				{
						servor_key_count++;
				}
			
				if(servo_rc->key.v &  KEY_PRESSED_OFFSET_Q)
				{
					servor_open_flag = 1;
				}
				else servor_open_flag = 0;
				
				if(servor_open_flag && (!last_servor_open_flag))
				{
					servor_key_count++;
				}
				
        for(uint8_t i = 0; i < 4; i++)
        {						
							  //单键							
						if((servor_key_count)%2)
						{
							servo_pwm[i] = SERVO_MAX_PWM;
							servor_real_open_ui = 1;
						}
						else if(!(servor_key_count%2))
						{
							servo_pwm[i] = SERVO_MIN_PWM;
							servor_real_open_ui = 0;
						}
							
           servo_pwm_set(servo_pwm[i], i);
									
        }
        osDelay(10);
				last_servor_open_flag = servor_open_flag;
				last_s_servor = s_servor;	
    }
}


