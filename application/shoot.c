/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      �������.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
//#include "autoaim.h"

#include "chassis_task.h"

#define shoot_laser_on()    laser_on()      //���⿪���궨��
#define shoot_laser_off()   laser_off()     //����رպ궨��
//΢������IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

int zero_or_one; //�������ο����йص�


/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          ��ת��ת����
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);



shoot_control_t shoot_control;          //�������
int shoot_init_flag = 0;

/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{
	  //����PID����
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
		//Ħ����PID����
		static const fp32 Fric1_speed_pid[3] = {FRIC_PID_KP, FRIC_PID_KI, FRIC_PID_KD};  
		static const fp32 Fric2_speed_pid[3] = {FRIC_PID_KP, FRIC_PID_KI, FRIC_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //ң����ָ��
    shoot_control.shoot_rc = get_remote_control_point();
    //���ָ��
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
		shoot_control.fric1_motor_measure = get_fric1_motor_measure_point();
		shoot_control.fric2_motor_measure = get_fric2_motor_measure_point();
    //��ʼ��PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.fric1_motor_pid, PID_POSITION, Fric1_speed_pid,FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
		PID_init(&shoot_control.fric2_motor_pid, PID_POSITION, Fric2_speed_pid,FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
		//��������
    shoot_feedback_update();
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
		//autoaim_mode_flag = 0;
}


int shoot_speed_add_little = 0;
int last_shoot_speed_add_little = 0;

extern int id_of_controller;
extern int fast_shoot;
/**
  * @brief          ���ѭ��
  * @param[in]      void
  * @retval         ����can����ֵ
  */
int16_t shoot_control_loop(void)
{

    judge_shoot_speed_add_little();
    shoot_set_mode();        //����״̬��
    shoot_feedback_update(); //��������
		shoot_control.last_shoot_mode = shoot_control.shoot_mode;
	
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //���ò����ֵ��ٶ�
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //���ò����ֵ��ٶ�
        shoot_control.speed_set = 0.0f;

    }
    else if(shoot_control.shoot_mode ==SHOOT_READY_BULLET)  //׼������״̬��ֱ�Ӳ����ٶȸ�0
    {
       shoot_control.speed_set = 0.0f;

    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //���ò����ֵ��ٶ�
       shoot_control.speed_set = 0.0f;

		}
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
//			 if(shoot_control.press_l)
//			 {
//        //���ò����ֵĲ����ٶ�,��������ת��ת����
 //       shoot_control.trigger_speed_set = SINGLE_TRIGGER_SPEED * zero_or_one;
//        trigger_motor_turn_back();				 
//			 }
			 if((shoot_control.press_l)||(switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])))
			 {
				 if(fast_shoot==1){shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED ;}
				 else {shoot_control.trigger_speed_set = TRIGGER_SPEED ;}
        //���ò����ֵĲ����ٶ�,��������ת��ת����
//      shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED ;//TRIGGER_SPEED//CONTINUE_TRIGGER_SPEED
			//shoot_control.trigger_speed_set = SINGLE_TRIGGER_SPEED * zero_or_one;
        trigger_motor_turn_back();				 
			 }
    }
    else if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.speed_set = 0.0f;
    }

    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.given_current = 0;
			  shoot_speed_add_little = 0;
			  last_shoot_speed_add_little = 0;
			  //Ħ����ֹͣ
        shoot_control.fric1_speed_set = 0;
			  shoot_control.fric2_speed_set = 0;
			  PID_calc(&shoot_control.fric1_motor_pid, shoot_control.fric1_speed, shoot_control.fric1_speed_set);
			  PID_calc(&shoot_control.fric2_motor_pid, shoot_control.fric2_speed, shoot_control.fric2_speed_set);
				shoot_control.fric1_given_current = (int16_t)(shoot_control.fric1_motor_pid.out);
			  shoot_control.fric2_given_current = (int16_t)(shoot_control.fric2_motor_pid.out);
			  CAN_cmd_fric(shoot_control.fric1_given_current, shoot_control.fric2_given_current, 0, 0);   //Ħ���ֵ����ֵ
    }
    else  //�����ֹͣģʽ
    {
//								shoot_control.fric1_speed_set = -3.8f;  //2.65
//					      shoot_control.fric2_speed_set = 3.8f;  //2.65 
			if(get_robot_shoot_speed_limit() == 15)
			{
				  shoot_control.fric1_speed_set = 2.1f;  // -1.65f
				  shoot_control.fric2_speed_set = -2.1f;
			}
			else if(get_robot_shoot_speed_limit() == 18) //�ر�ʹ�õ�����
			{
					shoot_control.fric1_speed_set = 2.1f;
					shoot_control.fric2_speed_set = -2.1f;
				
//					shoot_control.fric1_speed_set = -(1.80 + KEY_TO_FRIC_SPEED_SEN * shoot_speed_add_little);
//					shoot_control.fric2_speed_set = 1.80 + KEY_TO_FRIC_SPEED_SEN * shoot_speed_add_little;
			}
			else if(get_robot_shoot_speed_limit() == 30)
			{
					shoot_control.fric1_speed_set = 2.1f;  //2.65
					shoot_control.fric2_speed_set = -2.1f;  //2.65 
//					shoot_control.fric1_speed_set = -(2.60 + KEY_TO_FRIC_SPEED_SEN * shoot_speed_add_little);
//					shoot_control.fric2_speed_set = 2.60 + KEY_TO_FRIC_SPEED_SEN * shoot_speed_add_little;
			}
			else 
			{
					shoot_control.fric1_speed_set = 2.1f;//1.65
					shoot_control.fric2_speed_set = -2.1f;
//				  shoot_control.fric1_speed_set = -(1.65 + KEY_TO_FRIC_SPEED_SEN * shoot_speed_add_little);
//					shoot_control.fric2_speed_set = 1.65 + KEY_TO_FRIC_SPEED_SEN * shoot_speed_add_little;
 	  	}
        shoot_laser_on(); //���⿪��
        //���㲦���ֵ��PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
			  //����Ħ���ֵ��PID
			  PID_calc(&shoot_control.fric1_motor_pid, shoot_control.fric1_speed, shoot_control.fric1_speed_set);
			  PID_calc(&shoot_control.fric2_motor_pid, shoot_control.fric2_speed, shoot_control.fric2_speed_set);
			
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
			  shoot_control.fric1_given_current = (int16_t)(shoot_control.fric1_motor_pid.out);
			  shoot_control.fric2_given_current = (int16_t)(shoot_control.fric2_motor_pid.out);
			
        if(shoot_control.shoot_mode < SHOOT_READY)
        {
            shoot_control.given_current = 0;
						shoot_control.fric1_given_current = 0; 
					  shoot_control.fric2_given_current = 0;
        }

   
    }
		
		CAN_cmd_fric(shoot_control.fric1_given_current, shoot_control.fric2_given_current,0, 0);   //Ħ���ֵ����ֵ�� 
    return shoot_control.given_current;
}


extern uint8_t get_game_type(void);

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
//    static int8_t last_s = RC_SW_UP;

    //�ϲ��жϣ� һ�ο������ٴιر� && shoot_control.shoot_mode == SHOOT_STOP
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])  && !(shoot_control.press_l))   //&& !(shoot_control.press_l)
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
    else if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
		

    else if(shoot_control.shoot_mode == SHOOT_READY_BULLET && shoot_control.key != SWITCH_TRIGGER_ON)
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
    else if(shoot_control.shoot_mode == SHOOT_READY && shoot_control.key != SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
		
    else if(shoot_control.shoot_mode == SHOOT_READY)
    {
        //ң�����󲦸˵���(��¼�嵯��)����갴��һ�Σ��������״̬
        if ( (switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) ) || (shoot_control.press_l))  //|| (shoot_control.press_l && shoot_control.last_press_l == 0) 
        {

					// if(get_game_progress() == 4)
	//	        {    
						 get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
//		
//					      	if(shoot_control.heat + 30 < shoot_control.heat_limit)
//						     	{
						     		shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
//						     	}
//			 	         	else 
//							    {
//							     	shoot_control.shoot_mode = SHOOT_READY;
//							    }					
					  	
	
					
//						     	if(shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE < shoot_control.heat_limit)
//							      {
//							         	shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
//						      	}
//						     	else 
//							      {
//							        	shoot_control.shoot_mode = SHOOT_READY;
//					       		} 
//				     }
//				else
//					{
//						shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
//					}
        }
    }
     if(shoot_control.shoot_mode == SHOOT_DONE)
    {
        if(shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            shoot_control.key_time++;
            if(shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
            {
                shoot_control.key_time = 0;
                shoot_control.shoot_mode = SHOOT_READY_BULLET;
            }
        }
        else
        {
            shoot_control.key_time = 0;
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    }
    


    if(shoot_control.shoot_mode > SHOOT_READY)
    {
        //��곤��һֱ�������״̬ ��������
        if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))   //(shoot_control.press_l) || 
        {
//					if(get_game_progress() == 4)
//   					{
						get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
  					if(shoot_control.heat + 50 < shoot_control.heat_limit)
   						{
								shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
				      }
							else 
							{
								shoot_control.shoot_mode = SHOOT_READY;
							}							
//						}
//						else
//							if(shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE < shoot_control.heat_limit)
//							{
//								shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
//							}
//							else 
//							{
//								shoot_control.shoot_mode = SHOOT_READY;
//							}
			  }
//	    	else 
//					{
//						shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
//					}
//        else if(( !(shoot_control.press_l)) && switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])) 
//        {
				 if(( !(shoot_control.press_l)) && switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])) 
        {
            shoot_control.shoot_mode = SHOOT_READY;
        }
    }
		

    //�����̨״̬�� ����״̬���͹ر����
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
		
}

extern uint8_t get_game_type(void);

int single_shoot_time = 0;



/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

		//Ħ���ֵ�����ݸ���
		shoot_control.fric1_speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * shoot_control.fric1_motor_measure->speed_rpm; 
		shoot_control.fric2_speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * shoot_control.fric2_motor_measure->speed_rpm; 
    //���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    //���������Ƕ�
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    //΢������
    shoot_control.key = BUTTEN_TRIG_PIN;
    //��갴��
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //������ʱ
//    if (shoot_control.press_l)
//    {
//        if (shoot_control.press_l_time < PRESS_LONG_TIME)
//        {
//            shoot_control.press_l_time++;
//        }			
//    }
//    else
//    {
//        shoot_control.press_l_time = 0;
////			  single_shoot_time = 0;
//    }
		
//		if(shoot_control.press_l && !(shoot_control.last_press_l))
//		{
//			single_shoot_time += 25;	
//		}
//		if(single_shoot_time > 0)
//		{
//		  single_shoot_time--;
//		}
//		else 
//		{
//			single_shoot_time = 0;
//		}
//		if(single_shoot_time > SINGLE_SHOOT_MAX_TIME)
//		{
//			single_shoot_time = SINGLE_SHOOT_MAX_TIME;
//		}

    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    //��������µ�ʱ���ʱ
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    //����Ҽ����¼���Ħ���֣�ʹ�������������� �Ҽ��������
    static uint16_t up_time = 0;
    if (shoot_control.press_l)
    {
        up_time = UP_ADD_TIME;
    }
	
    //�������ο���  ��
		if(single_shoot_time > 0 )
		{
			zero_or_one = 1;
		}
		else if(single_shoot_time == 0)
		{
			zero_or_one = 0;
		}
}



//��ת��ת
static void trigger_motor_turn_back(void)
{
    if( shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)  
{

    //ÿ�β��� 1/4PI�ĽǶ�
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + PI_TEN);
        shoot_control.move_flag = 1;
    }
    if(shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_DONE;
    }
    //����Ƕ��ж�
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        //û����һֱ������ת�ٶ�
        shoot_control.trigger_speed_set = TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else
    {
        shoot_control.move_flag = 0;
    }
}

int G_pressed = 0;
int B_pressed = 0;

int last_G_pressed = 0;
int last_B_pressed = 0;

int G_pressed_count = 0;
int B_pressed_count = 0;



void judge_shoot_speed_add_little(void)
{
	if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_G)
	{
		G_pressed = 1;
	}
	else
	{
		G_pressed = 0;
	}
	
	if(G_pressed == 1 && last_G_pressed == 0)
	{
		G_pressed_count++;
		shoot_speed_add_little += 5;
		last_shoot_speed_add_little = 1;
	}
	
	if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_B)
	{
		B_pressed = 1;
	}
	else 
	{
		B_pressed = 0;
	}
	
	if(B_pressed == 1 && last_B_pressed == 0)
	{
		B_pressed_count++;
		shoot_speed_add_little -= 5;
		last_shoot_speed_add_little = 1;
	}
	
	last_G_pressed = G_pressed;
	last_B_pressed = B_pressed;
}
