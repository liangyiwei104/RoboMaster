/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
	
  */

#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "referee.h"



#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
	
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle， robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  *                 
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
	
//前后左右速度过渡		
static void start_run(void);		
	
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif



//底盘运动数据
chassis_move_t chassis_move;
//外部变量及函数声明
extern chassis_behaviour_e chassis_behaviour_mode;
extern gimbal_control_t gimbal_control;
extern fp32 AHRS_cosf(fp32 angle);   //相当于cos函数
extern fp32 AHRS_sinf(fp32 angle);   //相当于sin函数

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
	//超级电容发送时间间隔
	int time_count = 0;
    //wait a time 
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //chassis init
    //底盘初始化
    chassis_init(&chassis_move);
    //make sure all chassis motor is online,
    //判断底盘电机是否都在线
    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }

    while (1)
    {
			//ma超级电容发送时间间隔
			  time_count++;
			  //开始时缓慢加速
		  	 start_run();
        //set chassis control mode
        //设置底盘控制模式
        chassis_set_mode(&chassis_move);
        //when mode changes, some data save
        //模式切换数据保存
        chassis_mode_change_control_transit(&chassis_move);
        //chassis data update
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
        //set chassis control set-point 
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        //chassis control pid calculate
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);

        //make sure  one motor is online at least, so that the control CAN message can be received
        //确保至少一个电机在线， 这样CAN控制包可以被接收到
        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
        {
            //when remote control is offline, chassis motor should receive zero current. 
            //当遥控器掉线的时候，发送给底盘电机零电流.
            if (toe_is_error(DBUS_TOE))
            {
                CAN_cmd_chassis(0, 0, 0, 0);
            }
            else
            {
                //send control current
                //发送控制电流
                CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
//							if(time_count == 40)
//							{
//								power_current_set(get_chassis_power_limit()*100-60);
//								time_count = 0;
//							}
            }
        }
        //os delay
        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

fp32 motor_pid_maxout = 30000;

/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //chassis motor speed PID
    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    
    //chassis angle PID
    //底盘角度pid值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //in beginning， chassis mode is raw 
    //底盘开机状态为原始
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //get remote control point
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //get gyro sensor euler angle point
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //get gimbal motor data point
    //获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    
    //get chassis motor data point,  initialize motor speed PID
    //获取底盘电机数据指针，初始化PID 
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, motor_pid_maxout, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
		//ma
		//获取超级电容数据指针
		chassis_move_init->chassis_power_data = get_power_measure_point();
    //initialize angle PID
    //初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    
    //first order low-pass filter  replace ramp function
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //max and min speed
    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //update data
    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //change to follow gimbal angle mode
    //切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //change to follow chassis yaw angle
    //切入跟随底盘角度模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //change to no follow angle
    //切入不跟随云台模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle， robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //update motor speed, accel is differential of speed PID
        //更新电机速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //calculate vertical speed, horizontal speed ,rotation speed, left hand rule 
    //更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    //计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET);
}
/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
//开始时缓慢加速直至最大

//WASD按键按下计时
int start_time = 0;
//过渡过程速度
float start_run_speed_x = 0;	
float start_run_speed_y = 0;	
//二次函数比例系数
float START_RUN_SEN_X;// = 0.00016 * ((int)get_robot_level()+ 1);    //0.0048;//0.000012  //0.000016//前后     //灵敏度 * 250 * 250 就是速度
float START_RUN_SEN_Y;// = 0.000012;   //0.000012  //左右
extern int flying_flag;

//前后左右速度过渡函数,0.5s内加到最大速度x=1.5 y=1.5  &&(Capacitor_Voltage > 15)
static void start_run()
{
	if(chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)
			{
			  flying_flag = 1;
		  }
	else
			{
				flying_flag = 0;
			}
	if(flying_flag)
		{
		START_RUN_SEN_X = 0.00003 * 0.5 * 7 * 0.1325*0.4;  //shift加到最大速度4.0 
		START_RUN_SEN_Y = 0.000015 * 0.5 * 2.6667*0.4;
		}
		if(xtl_flag)
		{
		START_RUN_SEN_X = 0.000010 * 0.5 *(get_robot_level()+ 2);  //初始  1级乘1.5   2级乘2   3级乘2.5
		START_RUN_SEN_Y = 0.000010 * 0.5 *(get_robot_level()+ 2);			
		}
		else
		{
				START_RUN_SEN_X = 0.000015 * 0.5 *(get_robot_level()+ 1.5)*0.85;  //初始  1级乘1.5   2级乘2   3级乘2.5
				START_RUN_SEN_Y = 0.000015 * 0.5 *(get_robot_level()+ 1.5)*0.85;
		}
		
  	if(chassis_move.chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move.chassis_RC->key.v & CHASSIS_BACK_KEY || chassis_move.chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move.chassis_RC->key.v & CHASSIS_RIGHT_KEY)
	  {
	    start_time++;  //2ms加1
  	}
	  else 
  	{
  		start_time = 0;  //按键松开立马停下
  	}
		
		/* 限时，速度是时间的函数，相当于限速 */
//对抗赛or联盟赛
//		if(start_time <= 0)
//		{
//			start_time = 0;
//		}                          
//		if(start_time >= 250)  //250
//		{
//			start_time = 250;
//		}
		
	if(flying_flag == 1)
	{
		/* 限时，速度是时间的函数，相当于限速 */
		if(start_time <= 0)
		{
			start_time = 0;
		}                          
		if(start_time >= 500)
		{
			start_time = 500;
		}
	}
	else
	{
		//单项赛
		/* 限时，速度是时间的函数，相当于限速 */  
//		if(start_time <= 0)
//		{
//			start_time = 0;
//		}                          
//		if(start_time >= 340)
//		{
//			start_time = 340;
//		}

		if(start_time <= 0)
		{
			start_time = 0;
		}                          
		if(start_time >= 250)
		{
			start_time = 250;
		}
	}
	
	
	if(flying_flag == 1)
	{
		start_run_speed_x = -START_RUN_SEN_X * (start_time * start_time - 1000 * start_time);
	  start_run_speed_y = -START_RUN_SEN_Y * (start_time * start_time - 1000 * start_time);
	}
	else 
	{
		start_run_speed_x = -START_RUN_SEN_X * (start_time * start_time - 500 * start_time);
	  start_run_speed_y = -START_RUN_SEN_Y * (start_time * start_time - 500 * start_time);
	}
}


/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
int CTRL_W_COUNT;
int CTRL_W_flag;
int last_CTRL_W_flag;

int CTRL_S_COUNT;
int CTRL_S_flag;
int last_CTRL_S_flag;

int CTRL_A_COUNT;
int CTRL_A_flag;
int last_CTRL_A_flag;

int CTRL_D_COUNT;
int CTRL_D_flag;
int last_CTRL_D_flag;

extern float pitch_add_little;
extern float yaw_add_little;

void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;   
    vy_set_channel = -vy_channel * CHASSIS_VY_RC_SEN;

    //keyboard set speed set-point
		
	if(!(chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL)) //CTRL?????
	{		
    if ((chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)&&!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)&&!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY))    //?
    {
			    vx_set_channel = start_run_speed_x*180/100;  //前 115
			    vy_set_channel = 0;
//        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if ((chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)&&!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)&&!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY))  //?
    {
			    vx_set_channel = -start_run_speed_x*180/100;  //   115
//        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if ((chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)&&!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)&&!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY))  //?
    {
			    vy_set_channel = start_run_speed_y*180/100;  //  115
//        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if ((chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)&&!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)&&!(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY))  //?
    {
			    vy_set_channel = -start_run_speed_y*180/100;  //115
//        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }
	else if ((chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)&&(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY))    //??
    {
			    vx_set_channel = start_run_speed_x*150/100;  //80
			    vy_set_channel = start_run_speed_y*150/100;  //
//        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if ((chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)&&(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY))  //??
    {
			    vx_set_channel = -start_run_speed_x*150/100;  //80
			    vy_set_channel = start_run_speed_y*150/100;  //
//        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    else if ((chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)&&(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY))  //??
    {
					vx_set_channel = start_run_speed_x*150/100;  //80
			    vy_set_channel = -start_run_speed_y*150/100;  //
//        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if ((chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)&&(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY))  //??
    {
//					vx_set_channel = -start_run_speed_x/100*80;  //
//			    vy_set_channel = -start_run_speed_y/100*80;  //
			    vx_set_channel = -start_run_speed_x*150/100;  //80
			    vy_set_channel = -start_run_speed_y*150/100;  //
//        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }
	}
	else  //CTRL
	{
		if(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
		{
			CTRL_W_flag = 1;
		}
		else 
		{
			CTRL_W_flag = 0;
		}
		
	  if(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
		{
			CTRL_S_flag = 1;
		}
		else 
		{
			CTRL_S_flag = 0;
		}
		
		if(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
		{
			CTRL_A_flag = 1;
		}
		else 
		{
			CTRL_A_flag = 0;
		}
		
		if(chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
		{
			CTRL_D_flag = 1;
		}
		else 
		{
			CTRL_D_flag = 0;
		}
		
		if((CTRL_W_flag == 1) && (last_CTRL_W_flag == 0))
		{
			CTRL_W_COUNT++;
		//	pitch_add_little -= 0.01;
		}
		
		if((CTRL_S_flag == 1) && (last_CTRL_S_flag == 0))
		{
			CTRL_S_COUNT++;
	//		pitch_add_little += 0.01;
		}
		
		if((CTRL_A_flag == 1) && (last_CTRL_A_flag == 0))
		{
			CTRL_A_COUNT++;
	//	  yaw_add_little += 0.01;
		}
		
		if((CTRL_D_flag == 1) && (last_CTRL_D_flag == 0))
		{
			CTRL_D_COUNT++;
	//		yaw_add_little -= 0.01;
		}
		
		last_CTRL_W_flag = CTRL_W_flag;
		last_CTRL_S_flag = CTRL_S_flag;
		last_CTRL_A_flag = CTRL_A_flag;
		last_CTRL_D_flag = CTRL_D_flag;
		
	}

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
//    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
    //stop command, need not slow change, set zero derectly
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

//   *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
		*vx_set = vx_set_channel;
}
/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }


    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    //get three control set-point, 获取三个控制设置值
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //follow gimbal mode
    //跟随云台模式
//    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
//    {
//        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
//        //rotate chassis direction, make sure vertial direction follow gimbal 
//        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
//        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
//        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
//        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
//        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
//        //set control relative angle  set-point
//        //设置控制相对云台角度
//        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
//        //calculate ratation speed
//        //计算旋转PID角速度
//        chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
//        //speed limit
//        //速度限幅 
//        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
//        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
//    }
 if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //rotate chassis direction, make sure vertial direction follow gimbal 
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
         //小陀螺运动分解方法
//			  chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;               
//        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
			    chassis_move_control->vx_set = vx_set;
			    chassis_move_control->vy_set = vy_set;
        //set control relative angle  set-point
        //设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //calculate ratation speed
        //计算旋转PID角速度
        chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
        //speed limit
        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
		
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {   
			 fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //rotate chassis direction, make sure vertial direction follow gimbal 
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //set control relative angle  set-point
        //设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //calculate ratation speed
        //计算旋转PID角速度

			   chassis_move_control->wz_set = 0;
			  
	
        //speed limit
        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //"angle_set" is rotation speed set-point
        //“angle_set” 是旋转速度控制
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //in raw mode, set-point is sent to CAN bus
        //在原始模式，设置值是发送到CAN总线
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
	
}

/**
  * @brief          four mecanum wheels speed is calculated by three param. 
  * @param[in]      vx_set: vertial speed
  * @param[in]      vy_set: horizontal speed
  * @param[in]      wz_set: rotation speed
  * @param[out]     wheel_speed: four mecanum wheels speed
  * @retval         none
  */
/**
  * @brief          四个麦轮速度是通过三个参数计算出来的
  * @param[in]      vx_set: 纵向速度
  * @param[in]      vy_set: 横向速度
  * @param[in]      wz_set: 旋转速度
  * @param[out]     wheel_speed: 四个麦轮速度
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
	  if(chassis_behaviour_mode == CHASSIS_XTL)
		{
      wheel_speed[0] = - (vx_set * AHRS_cosf(-gimbal_control.gimbal_yaw_motor.relative_angle+PI/2))\
			                 - (vy_set * AHRS_sinf(-gimbal_control.gimbal_yaw_motor.relative_angle))\
			                 - (vy_set * AHRS_cosf(-gimbal_control.gimbal_yaw_motor.relative_angle))\
			                 - (vx_set * AHRS_sinf(-gimbal_control.gimbal_yaw_motor.relative_angle+PI/2))\
			                 + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			
			wheel_speed[1] = - (vx_set * AHRS_cosf(-gimbal_control.gimbal_yaw_motor.relative_angle+PI/2))\
			                 + (vy_set * AHRS_sinf(-gimbal_control.gimbal_yaw_motor.relative_angle))\
			                 - (vy_set * AHRS_cosf(-gimbal_control.gimbal_yaw_motor.relative_angle))\
			                 + (vx_set * AHRS_sinf(-gimbal_control.gimbal_yaw_motor.relative_angle+PI/2))\
			                 + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			
			wheel_speed[2] = + (vx_set * AHRS_cosf(-gimbal_control.gimbal_yaw_motor.relative_angle+PI/2))\
			                 + (vy_set * AHRS_sinf(-gimbal_control.gimbal_yaw_motor.relative_angle))\
			                 + (vy_set * AHRS_cosf(-gimbal_control.gimbal_yaw_motor.relative_angle))\
			                 + (vx_set * AHRS_sinf(-gimbal_control.gimbal_yaw_motor.relative_angle+PI/2))\
			                 + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
			
			wheel_speed[3] = + (vx_set * AHRS_cosf(-gimbal_control.gimbal_yaw_motor.relative_angle+PI/2))\
			                 - (vy_set * AHRS_sinf(-gimbal_control.gimbal_yaw_motor.relative_angle))\
			                 + (vy_set * AHRS_cosf(-gimbal_control.gimbal_yaw_motor.relative_angle))\
			                 - (vx_set * AHRS_sinf(-gimbal_control.gimbal_yaw_motor.relative_angle+PI/2))\
			                 + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	    }
else
{	
    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}
}

fp32 motor_speed_pid_kp = 20000.0f;
fp32 motor_speed_pid_ki = 110.0f;
fp32 motor_speed_pid_kd = 0.0f;

fp32 motor_angle_pid_kp = 30.0f;
fp32 motor_angle_pid_ki = 15.0f;
fp32 motor_angle_pid_kd = 90.0f;

/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
extern uint8_t gimbal_turn_flag;
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    //mecanum wheel speed calculation
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        //in raw mode, derectly return
        //raw控制直接返回
        return;
    }

    //calculate the max speed in four wheels, limit the max speed
    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs( chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }
		
		//calculate pid
    //计算pid
		
		if(gimbal_turn_flag == 1)
		{
			for (i = 0; i < 4; i++)
    {
			  chassis_move.motor_speed_pid[i].Kp = motor_speed_pid_kp;   		//20000
			  chassis_move.motor_speed_pid[i].Ki = motor_speed_pid_ki;      //110
			  chassis_move.motor_speed_pid[i].Kd = motor_speed_pid_kd;      //0
			
			  chassis_move.chassis_angle_pid.Kp = motor_angle_pid_kp;      //60//65//60
			  chassis_move.chassis_angle_pid.Ki = motor_angle_pid_ki;       //8     
			  chassis_move.chassis_angle_pid.Kd = motor_angle_pid_kd;       //2
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }
		}
		else if(gimbal_turn_flag == 0)
		{
			for (i = 0; i < 4; i++)
    {
			  chassis_move.motor_speed_pid[i].Kp = motor_speed_pid_kp;   //15000
			  chassis_move.motor_speed_pid[i].Ki = motor_speed_pid_ki;      //10
			  chassis_move.motor_speed_pid[i].Kd = motor_speed_pid_kd;      //0
			
			  chassis_move.chassis_angle_pid.Kp = motor_angle_pid_kp;      //60//65//60
			  chassis_move.chassis_angle_pid.Ki = motor_angle_pid_ki;       //8     
			  chassis_move.chassis_angle_pid.Kd = motor_angle_pid_kd;       //2
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }
		}
//    for (i = 0; i < 4; i++)
//    {
//        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
//    }


    //功率控制
    chassis_power_control(chassis_move_control_loop);


    //赋值电流值
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}
