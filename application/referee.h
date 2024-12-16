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
	uint8_t game_type : 4;         //1：机甲大师赛   2：单项赛   3：人工智能挑战赛   4：3V3联盟赛   5：1V1联盟赛
	uint8_t game_progress : 4;     //0：未开始   1：准备   2：自检   3：5s倒计时   4：对战中   5：比赛结算中  
  uint16_t stage_remain_time;    //当前阶段剩余时间
	uint64_t SyncTimeStamp;      //机器人接收到该指令的精确Unix时间
} ext_game_state_t;             

typedef __packed struct //0002
{
    uint8_t winner;   //0平局，1红胜，2蓝胜
} ext_game_result_t;
typedef __packed struct
{
    uint16_t red_1_robot_HP;   //红1英雄血量
    uint16_t red_2_robot_HP;   //红2工程
    uint16_t red_3_robot_HP;   //红3步兵
    uint16_t red_4_robot_HP;   //红4步兵
    uint16_t red_5_robot_HP;   //红5步兵
    uint16_t red_7_robot_HP;   //红7哨兵 
	  uint16_t red_outpost_HP; //红前哨站
    uint16_t red_base_HP;      //红基地 
	  
    uint16_t blue_1_robot_HP;   //蓝1英雄
    uint16_t blue_2_robot_HP;   //蓝2工程
    uint16_t blue_3_robot_HP;   //蓝3步兵
    uint16_t blue_4_robot_HP;   //蓝4步兵
    uint16_t blue_5_robot_HP;   //蓝5步兵
    uint16_t blue_7_robot_HP;   //蓝6哨兵
  	uint16_t blue_outpost_HP; //蓝前哨站
    uint16_t blue_base_HP;      //蓝基地
	  
} ext_game_robot_HP_t;
typedef __packed struct //0101
{
    uint32_t event_type; 
	/*
	  bit 0：己方1号补血点补血状态，1为已占领
	  bit 1: 己方2号
	  bit 2：己方3号
    bit 3-5：己方能量机关状态   
          3：打击点占领状态，1为占领
          4：小能量机关激活状态，1为已激活
	        5：大能量机关激活状态
	  bit 6：己方R2环形高地占领状态，1为已占领
	  bit 7：己方R3
	  bit 8：己方R4
	  bit 9：己方基地护盾状态，1为有虚拟护盾血量
	  bit 10：己方前哨站状态，1为前哨站存活
	  bit 11~31：保留
  */
} ext_event_data_t;

typedef __packed struct //0x0102
{
    uint8_t supply_projectile_id;     //补给站口ID：1/2
    uint8_t supply_robot_id;          //补弹机器人ID：0无，1为红英雄，2为红工程，3/4/5为红步兵，101为蓝英雄，102为蓝工程，103/104/105为蓝步兵
    uint8_t supply_projectile_step;   //出弹口开闭状态：0为关闭，1为子弹准备中，2为子弹下落
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
    uint8_t level;          //1：黄牌   2：红牌   3：判负
    uint8_t foul_robot_id;  //犯规机器人ID，判负时ID为0，黄牌/红牌时为犯规机器人ID
} ext_referee_warning_t;
typedef __packed struct //0x0201
{
	  uint8_t robot_id;      //1：红英雄   2：红工程   3/4/5：红步兵   6：红空中   7：红哨兵   8：红飞镖   9：红雷达
	                         //101:蓝英雄  102:蓝工程  103/104/105:蓝步兵  106:蓝空中  107:蓝哨兵  108:蓝飞镖  109:蓝雷达
    uint8_t robot_level;   //机器人等级  1：一级   2：二级   3：三级
    uint16_t remain_HP;    //机器人剩余血量
    uint16_t max_HP;       //机器人上限血量 
		uint16_t shooter_id1_17mm_cooling_rate;   //每秒冷却值
		uint16_t shooter_id1_17mm_cooling_limit;  //热量上限
		uint16_t shooter_id1_17mm_speed_limit;    //射速上限
		uint16_t shooter_id2_17mm_cooling_rate;
		uint16_t shooter_id2_17mm_cooling_limit;
		uint16_t shooter_id2_17mm_speed_limit;
		uint16_t shooter_id1_42mm_cooling_rate;
		uint16_t shooter_id1_42mm_cooling_limit;
		uint16_t shooter_id1_42mm_speed_limit;	
		uint16_t chassis_power_limit;  //底盘功率限制上限
    uint8_t mains_power_gimbal_output : 1;   //gimbal口输出                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     //gimbal口输出，1为有24V输出，0为无24V输出
    uint8_t mains_power_chassis_output : 1;  //chassis口输出
    uint8_t mains_power_shooter_output : 1;  //shooter口输出
} ext_game_robot_state_t;

typedef __packed struct //0x0202
{
    uint16_t chassis_volt;   //底盘输出电压, mV
    uint16_t chassis_current;   //底盘输出电流, mA
    float chassis_power;   //底盘输出功率, W
    uint16_t chassis_power_buffer;   //底盘功率缓冲,J   注：飞坡根据规则增加至250J
	  uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct //0x0203
{
    float x;  
    float y;
    float z;
    float yaw;   //位置枪口
} ext_game_robot_pos_t;

typedef __packed struct //0x0204
{
	uint8_t power_rune_buff;  //bit 0: 机器人血量补血状态   1：枪口热量冷却加速   2：机器人防御加成   3：机器人攻击加成
} ext_buff_musk_t;

typedef __packed struct //0x0205
{
    uint8_t attack_time;   //可攻击时间，30s递减至0
} aerial_robot_energy_t;

typedef __packed struct //0x0206
{
	  uint8_t armor_type : 4;   //0:装甲没有受到伤害   1-4：受到打击的装甲id
    uint8_t hurt_type : 4;   //血量变化类型
	                           //0x0：装甲伤害扣血
	                           //0x1：模块掉线扣血
	                           //0x2：超射速扣血
	                           //0x3：超枪口热量扣血
	                           //0x4：超底盘功率扣血
	                           //0x5：装甲撞击扣血
} ext_robot_hurt_t;

typedef __packed struct //0x0207
{
	  uint8_t bullet_type;    //子弹类型    1：17mm   2: 42mm
	  uint8_t shooter_id;     //发射机构ID
    uint8_t bullet_freq;    //子弹射频    
    float bullet_speed;     //子弹射速
} ext_shoot_data_t;
typedef __packed struct
{ 
	 uint16_t bullet_remaining_num_17mm;  //子弹剩余发射数量
   uint16_t bullet_remaining_num_42mm;
   uint16_t coin_remaining_num;  //剩余金币
} ext_bullet_remaining_t;
typedef __packed struct //0x0301
{
    uint16_t send_ID;      //发送者ID，需要校验发送者的校验正确性
    uint16_t receiver_ID;  //接受者ID，需要校验接收者的ID正确性
    uint16_t data_cmd_id;  //数据段的内容ID
    uint16_t data_len;     //内容数据段，x最大为113
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
