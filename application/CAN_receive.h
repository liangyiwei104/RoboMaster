/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ???CAN??????,??????,CAN??????????????.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN     hcan1
#define GIMBAL_CAN      hcan1


/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,
	  
} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

//ma   add  2022.1.11  21:58
//�����������ݷ���
typedef struct
{
	 float InputVot;   //���뵽���ݵ�ѹ
	 float CapVot;     //���������ѹ
	 float Input_Current;  //���뵽���ݵ���
	 float Target_Power;    //���뵽���ݹ���
	
}power_measure_t;

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          ????????(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020??????, ?? [-30000,30000]
  * @param[in]      pitch: (0x206) 6020??????, ?? [-30000,30000]
  * @param[in]      shoot: (0x207) 2006??????, ?? [-10000,10000]
  * @param[in]      rev: (0x208) ??,??????
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev2);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ??ID?0x700?CAN?,????3508????????ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);



/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          ????????(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508??????, ?? [-16384,16384]
  * @param[in]      motor2: (0x202) 3508??????, ?? [-16384,16384]
  * @param[in]      motor3: (0x203) 3508??????, ?? [-16384,16384]
  * @param[in]      motor4: (0x204) 3508??????, ?? [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ??yaw 6020??????
  * @param[in]      none
  * @retval         ??????
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ??pitch 6020??????
  * @param[in]      none
  * @retval         ??????
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ?????? 2006??????
  * @param[in]      none
  * @retval         ??????
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ?????? 3508??????
  * @param[in]      i: ????,??[0,3]
  * @retval         ??????
  */
	
//ma
//�������ݳ�繦������
void power_current_set( uint16_t new_power );

//Ħ���ֵ�������
void CAN_cmd_fric(int16_t fric1, int16_t fric2, int16_t rev1, int16_t rev2);

extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

//��������ģ�����
void get_power_measure(power_measure_t *ptr,uint8_t *data1);

////��������ģ�鷢��
//void power_current_set(uint16_t new_power, uint8_t save_flg);

//�������ݷ�������ָ��
extern const power_measure_t *get_power_measure_point(void);

//Ħ�������ݷ���ָ��
extern const motor_measure_t *get_fric1_motor_measure_point(void);

extern const motor_measure_t *get_fric2_motor_measure_point(void);


#endif

