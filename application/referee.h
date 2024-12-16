#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;
typedef __packed struct //0001
{
	uint8_t game_type : 4;         //1�����״�ʦ��   2��������   3���˹�������ս��   4��3V3������   5��1V1������
	uint8_t game_progress : 4;     //0��δ��ʼ   1��׼��   2���Լ�   3��5s����ʱ   4����ս��   5������������  
  uint16_t stage_remain_time;    //��ǰ�׶�ʣ��ʱ��
	uint64_t SyncTimeStamp;      //�����˽��յ���ָ��ľ�ȷUnixʱ��
} ext_game_state_t;             

typedef __packed struct //0002
{
    uint8_t winner;   //0ƽ�֣�1��ʤ��2��ʤ
} ext_game_result_t;
typedef __packed struct
{
    uint16_t red_1_robot_HP;   //��1Ӣ��Ѫ��
    uint16_t red_2_robot_HP;   //��2����
    uint16_t red_3_robot_HP;   //��3����
    uint16_t red_4_robot_HP;   //��4����
    uint16_t red_5_robot_HP;   //��5����
    uint16_t red_7_robot_HP;   //��7�ڱ� 
	  uint16_t red_outpost_HP; //��ǰ��վ
    uint16_t red_base_HP;      //����� 
	  
    uint16_t blue_1_robot_HP;   //��1Ӣ��
    uint16_t blue_2_robot_HP;   //��2����
    uint16_t blue_3_robot_HP;   //��3����
    uint16_t blue_4_robot_HP;   //��4����
    uint16_t blue_5_robot_HP;   //��5����
    uint16_t blue_7_robot_HP;   //��6�ڱ�
  	uint16_t blue_outpost_HP; //��ǰ��վ
    uint16_t blue_base_HP;      //������
	  
} ext_game_robot_HP_t;
typedef __packed struct //0101
{
    uint32_t event_type; 
	/*
	  bit 0������1�Ų�Ѫ�㲹Ѫ״̬��1Ϊ��ռ��
	  bit 1: ����2��
	  bit 2������3��
    bit 3-5��������������״̬   
          3�������ռ��״̬��1Ϊռ��
          4��С�������ؼ���״̬��1Ϊ�Ѽ���
	        5�����������ؼ���״̬
	  bit 6������R2���θߵ�ռ��״̬��1Ϊ��ռ��
	  bit 7������R3
	  bit 8������R4
	  bit 9���������ػ���״̬��1Ϊ�����⻤��Ѫ��
	  bit 10������ǰ��վ״̬��1Ϊǰ��վ���
	  bit 11~31������
  */
} ext_event_data_t;

typedef __packed struct //0x0102
{
    uint8_t supply_projectile_id;     //����վ��ID��1/2
    uint8_t supply_robot_id;          //����������ID��0�ޣ�1Ϊ��Ӣ�ۣ�2Ϊ�칤�̣�3/4/5Ϊ�첽����101Ϊ��Ӣ�ۣ�102Ϊ�����̣�103/104/105Ϊ������
    uint8_t supply_projectile_step;   //�����ڿ���״̬��0Ϊ�رգ�1Ϊ�ӵ�׼���У�2Ϊ�ӵ�����
    uint8_t supply_projectile_num;    // 50/100/150/200
} ext_supply_projectile_action_t;


typedef __packed struct //0x0103
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct
{
    uint8_t level;          //1������   2������   3���и�
    uint8_t foul_robot_id;  //���������ID���и�ʱIDΪ0������/����ʱΪ���������ID
} ext_referee_warning_t;
typedef __packed struct //0x0201
{
	  uint8_t robot_id;      //1����Ӣ��   2���칤��   3/4/5���첽��   6�������   7�����ڱ�   8�������   9�����״�
	                         //101:��Ӣ��  102:������  103/104/105:������  106:������  107:���ڱ�  108:������  109:���״�
    uint8_t robot_level;   //�����˵ȼ�  1��һ��   2������   3������
    uint16_t remain_HP;    //������ʣ��Ѫ��
    uint16_t max_HP;       //����������Ѫ�� 
		uint16_t shooter_id1_17mm_cooling_rate;   //ÿ����ȴֵ
		uint16_t shooter_id1_17mm_cooling_limit;  //��������
		uint16_t shooter_id1_17mm_speed_limit;    //��������
		uint16_t shooter_id2_17mm_cooling_rate;
		uint16_t shooter_id2_17mm_cooling_limit;
		uint16_t shooter_id2_17mm_speed_limit;
		uint16_t shooter_id1_42mm_cooling_rate;
		uint16_t shooter_id1_42mm_cooling_limit;
		uint16_t shooter_id1_42mm_speed_limit;	
		uint16_t chassis_power_limit;  //���̹�����������
    uint8_t mains_power_gimbal_output : 1;   //gimbal�����                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     //gimbal�������1Ϊ��24V�����0Ϊ��24V���
    uint8_t mains_power_chassis_output : 1;  //chassis�����
    uint8_t mains_power_shooter_output : 1;  //shooter�����
} ext_game_robot_state_t;

typedef __packed struct //0x0202
{
    uint16_t chassis_volt;   //���������ѹ, mV
    uint16_t chassis_current;   //�����������, mA
    float chassis_power;   //�����������, W
    uint16_t chassis_power_buffer;   //���̹��ʻ���,J   ע�����¸��ݹ���������250J
	  uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct //0x0203
{
    float x;  
    float y;
    float z;
    float yaw;   //λ��ǹ��
} ext_game_robot_pos_t;

typedef __packed struct //0x0204
{
	uint8_t power_rune_buff;  //bit 0: ������Ѫ����Ѫ״̬   1��ǹ��������ȴ����   2�������˷����ӳ�   3�������˹����ӳ�
} ext_buff_musk_t;

typedef __packed struct //0x0205
{
    uint8_t attack_time;   //�ɹ���ʱ�䣬30s�ݼ���0
} aerial_robot_energy_t;

typedef __packed struct //0x0206
{
	  uint8_t armor_type : 4;   //0:װ��û���ܵ��˺�   1-4���ܵ������װ��id
    uint8_t hurt_type : 4;   //Ѫ���仯����
	                           //0x0��װ���˺���Ѫ
	                           //0x1��ģ����߿�Ѫ
	                           //0x2�������ٿ�Ѫ
	                           //0x3����ǹ��������Ѫ
	                           //0x4�������̹��ʿ�Ѫ
	                           //0x5��װ��ײ����Ѫ
} ext_robot_hurt_t;

typedef __packed struct //0x0207
{
	  uint8_t bullet_type;    //�ӵ�����    1��17mm   2: 42mm
	  uint8_t shooter_id;     //�������ID
    uint8_t bullet_freq;    //�ӵ���Ƶ    
    float bullet_speed;     //�ӵ�����
} ext_shoot_data_t;
typedef __packed struct
{ 
	 uint16_t bullet_remaining_num_17mm;  //�ӵ�ʣ�෢������
   uint16_t bullet_remaining_num_42mm;
   uint16_t coin_remaining_num;  //ʣ����
} ext_bullet_remaining_t;
typedef __packed struct //0x0301
{
    uint16_t send_ID;      //������ID����ҪУ�鷢���ߵ�У����ȷ��
    uint16_t receiver_ID;  //������ID����ҪУ������ߵ�ID��ȷ��
    uint16_t data_cmd_id;  //���ݶε�����ID
    uint16_t data_len;     //�������ݶΣ�x���Ϊ113
    uint8_t *data;
} ext_student_interactive_data_t;

typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;


typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;



extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);

extern uint8_t get_robot_id(void);
extern uint8_t get_remain_HP(void);
extern uint8_t get_robot_level(void);
extern uint8_t get_robot_shoot_speed_limit(void);
extern uint8_t get_game_progress(void);
extern uint8_t get_chassis_power_limit(void);
extern uint8_t get_chassis_power_buffer(void);
extern uint8_t get_bullet_remaining_num(void);
extern uint8_t get_game_progress(void);
extern float get_bullet_speed(void);
extern uint8_t get_game_type(void);
extern uint8_t get_hurt_type(void);
extern uint8_t last_remain_HP;
extern uint8_t remain_HP;

extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);
extern void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1);

extern void get_shoot_remaining_num(void);
#endif
