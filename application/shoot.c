/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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

#define shoot_laser_on()    laser_on()      //激光开启宏定义
#define shoot_laser_off()   laser_off()     //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

int zero_or_one; //单发波形控制有关的


/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);



shoot_control_t shoot_control;          //射击数据
int shoot_init_flag = 0;

/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{
	  //波轮PID参数
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
		//摩擦轮PID参数
		static const fp32 Fric1_speed_pid[3] = {FRIC_PID_KP, FRIC_PID_KI, FRIC_PID_KD};  
		static const fp32 Fric2_speed_pid[3] = {FRIC_PID_KP, FRIC_PID_KI, FRIC_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    //电机指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
		shoot_control.fric1_motor_measure = get_fric1_motor_measure_point();
		shoot_control.fric2_motor_measure = get_fric2_motor_measure_point();
    //初始化PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.fric1_motor_pid, PID_POSITION, Fric1_speed_pid,FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
		PID_init(&shoot_control.fric2_motor_pid, PID_POSITION, Fric2_speed_pid,FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
		//更新数据
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
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{

    judge_shoot_speed_add_little();
    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据
		shoot_control.last_shoot_mode = shoot_control.shoot_mode;
	
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;

    }
    else if(shoot_control.shoot_mode ==SHOOT_READY_BULLET)  //准备发弹状态，直接波轮速度赋0
    {
       shoot_control.speed_set = 0.0f;

    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //设置拨弹轮的速度
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
//        //设置拨弹轮的拨动速度,并开启堵转反转处理
 //       shoot_control.trigger_speed_set = SINGLE_TRIGGER_SPEED * zero_or_one;
//        trigger_motor_turn_back();				 
//			 }
			 if((shoot_control.press_l)||(switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])))
			 {
				 if(fast_shoot==1){shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED ;}
				 else {shoot_control.trigger_speed_set = TRIGGER_SPEED ;}
        //设置拨弹轮的拨动速度,并开启堵转反转处理
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
			  //摩擦轮停止
        shoot_control.fric1_speed_set = 0;
			  shoot_control.fric2_speed_set = 0;
			  PID_calc(&shoot_control.fric1_motor_pid, shoot_control.fric1_speed, shoot_control.fric1_speed_set);
			  PID_calc(&shoot_control.fric2_motor_pid, shoot_control.fric2_speed, shoot_control.fric2_speed_set);
				shoot_control.fric1_given_current = (int16_t)(shoot_control.fric1_motor_pid.out);
			  shoot_control.fric2_given_current = (int16_t)(shoot_control.fric2_motor_pid.out);
			  CAN_cmd_fric(shoot_control.fric1_given_current, shoot_control.fric2_given_current, 0, 0);   //摩擦轮电机赋值
    }
    else  //非射击停止模式
    {
//								shoot_control.fric1_speed_set = -3.8f;  //2.65
//					      shoot_control.fric2_speed_set = 3.8f;  //2.65 
			if(get_robot_shoot_speed_limit() == 15)
			{
				  shoot_control.fric1_speed_set = 2.1f;  // -1.65f
				  shoot_control.fric2_speed_set = -2.1f;
			}
			else if(get_robot_shoot_speed_limit() == 18) //特别使用单项赛
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
        shoot_laser_on(); //激光开启
        //计算拨弹轮电机PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
			  //计算摩擦轮电机PID
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
		
		CAN_cmd_fric(shoot_control.fric1_given_current, shoot_control.fric2_given_current,0, 0);   //摩擦轮电机赋值。 
    return shoot_control.given_current;
}


extern uint8_t get_game_type(void);

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
//    static int8_t last_s = RC_SW_UP;

    //上拨判断， 一次开启，再次关闭 && shoot_control.shoot_mode == SHOOT_STOP
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
        //遥控器左拨杆到上(检录清弹用)或鼠标按下一次，进入射击状态
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
        //鼠标长按一直进入射击状态 保持连发
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
		

    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }
		
}

extern uint8_t get_game_type(void);

int single_shoot_time = 0;



/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

		//摩擦轮电机数据更新
		shoot_control.fric1_speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * shoot_control.fric1_motor_measure->speed_rpm; 
		shoot_control.fric2_speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * shoot_control.fric2_motor_measure->speed_rpm; 
    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
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

    //计算输出轴角度
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    //微动开关
    shoot_control.key = BUTTEN_TRIG_PIN;
    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //长按计时
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

    //射击开关下档时间计时
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

    //鼠标右键按下加速摩擦轮，使得左键低速射击， 右键高速射击
    static uint16_t up_time = 0;
    if (shoot_control.press_l)
    {
        up_time = UP_ADD_TIME;
    }
	
    //单发波形控制  试
		if(single_shoot_time > 0 )
		{
			zero_or_one = 1;
		}
		else if(single_shoot_time == 0)
		{
			zero_or_one = 0;
		}
}



//堵转反转
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
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)  
{

    //每次拨动 1/4PI的角度
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + PI_TEN);
        shoot_control.move_flag = 1;
    }
    if(shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_DONE;
    }
    //到达角度判断
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        //没到达一直设置旋转速度
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
