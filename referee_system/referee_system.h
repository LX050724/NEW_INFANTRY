#ifndef _REFEREE_SYSTEM_H_
#define _REFEREE_SYSTEM_H_
#include "freertos_task.h"
#include "CRC.h"

typedef enum 
{
	REFEREE_SYSTEM_DISABLE = 0,
	REFEREE_SYSTEM_ENABLE,
}REFEREE_SYSTEM_SWITCH;

#define threshold (referee_system_QueueLength-2)

#define Match_status_data																REFEREE_SYSTEM_ENABLE
#define Result_data			    														REFEREE_SYSTEM_ENABLE
#define Match_robot_survival_data 											REFEREE_SYSTEM_ENABLE
#define Site_event_data 																REFEREE_SYSTEM_ENABLE
#define Site_supply_station_action_identification_data 	REFEREE_SYSTEM_ENABLE
#define Robot_state_data																REFEREE_SYSTEM_ENABLE
#define Real_time_power_and_heat_data										REFEREE_SYSTEM_ENABLE
#define Robot_position_data															REFEREE_SYSTEM_ENABLE
#define Robot_gain_data																	REFEREE_SYSTEM_ENABLE
#define Air_robot_energy_status_data										REFEREE_SYSTEM_ENABLE
#define Damage_status_data															REFEREE_SYSTEM_ENABLE
#define Real_time_shooting_data													REFEREE_SYSTEM_ENABLE

#define CMD_code_Match_status_data															 ((uint16_t)0x0001)
#define CMD_code_Result_data			    													 ((uint16_t)0x0002)
#define CMD_code_Match_robot_survival_data 										 	 ((uint16_t)0x0003)
#define CMD_code_Site_event_data 																 ((uint16_t)0x0101)
#define CMD_code_Site_supply_station_action_identification_data	 ((uint16_t)0x0102)
#define CMD_code_Field_resupply_station_booking_bullet_data			 ((uint16_t)0x0103)
#define CMD_code_Robot_state_data														 		 ((uint16_t)0x0201)
#define CMD_code_Real_time_power_and_heat_data								 	 ((uint16_t)0x0202)
#define CMD_code_Robot_position_data													 	 ((uint16_t)0x0203)
#define CMD_code_Robot_gain_data															 	 ((uint16_t)0x0204)
#define CMD_code_Air_robot_energy_status_data										 ((uint16_t)0x0205)
#define CMD_code_Damage_status_data													 		 ((uint16_t)0x0206)
#define CMD_code_Real_time_shooting_data											 	 ((uint16_t)0x0207)
#define CMD_code_Robots_interact_with_data											 ((uint16_t)0x0301)

#define Data_long_Match_status_data														   ((uint8_t)12)
#define Data_long_Result_data			    													 ((uint8_t)10)
#define Data_long_Match_robot_survival_data 									   ((uint8_t)11)
#define Data_long_Site_event_data 															 ((uint8_t)13)
#define Data_long_Site_supply_station_action_identification_data ((uint8_t)12)
#define Data_long_Robot_state_data															 ((uint8_t)24)
#define Data_long_Real_time_power_and_heat_data									 ((uint8_t)23)
#define Data_long_Robot_position_data													   ((uint8_t)25)
#define Data_long_Robot_gain_data															   ((uint8_t)10)
#define Data_long_Air_robot_energy_status_data									 ((uint8_t)12)
#define Data_long_Damage_status_data													   ((uint8_t)10)
#define Data_long_Real_time_shooting_data												 ((uint8_t)15)

typedef enum
{
	OK = 0,
	CRC_ERROR,
	Error,
}referee_system_StatusTypeDef;

typedef enum
{
	CRC_DISABLE = 0,
	CRC_ENABLE,
}CRC_StatusTypeDef;

typedef enum
{
	None= 0,
	R_Hero = 1,
	R_Engineering=2,
	R_Infantry1=3,
	R_Infantry2=4,
	R_Infantry3=5,
	R_Air=6,
	R_Sentinel=7,
	B_Hero=11,
	B_Engineering=12,
	B_Infantry1=13,
	B_Infantry2=14,
	B_Infantry3=15,
	B_Air=16,
	B_Sentinel=17,
}Robot_ID;

typedef union
{
	uint8_t buff[4];
	float floatout;
}uint8_t_to_float;

/*Match_status_data*/
typedef union 
{
	uint8_t Match_status_IN;
	struct 
	{
		uint8_t game_type : 4;
		uint8_t game_stage : 4;
	}domain;
} ext_game_state_t;

/*Result_data*/
typedef __packed struct
{
	uint8_t winner;
} ext_game_result_t;

/*Match_robot_survival_data*/
typedef enum
{
	survive=0,
	death,
}robot_survivors;

typedef union
{
	uint8_t robot_legion[2];
	struct
	{
		uint16_t R_Hero : 1;
		uint16_t R_Engineering : 1;
		uint16_t R_Infantry1 : 1;
		uint16_t R_Infantry2 : 1;
		uint16_t R_Infantry3 : 1;
		uint16_t R_Air : 1;
		uint16_t R_Sentinel : 1;
		uint16_t : 1;
		uint16_t B_Hero : 1;
		uint16_t B_Engineering : 1;
		uint16_t B_Infantry1 : 1;
		uint16_t B_Infantry2 : 1;
		uint16_t B_Infantry3 : 1;
		uint16_t B_Air : 1;
		uint16_t B_Sentinel : 1;
	}domain;
} ext_game_robot_survivors_t;

/*Site_event_data*/
typedef enum
{
	Not_occupy = 0,
	occupation,
}occupations;

typedef enum
{
	not_occupied_not_activated=0,
	occupied_not_activated,
	not_occupied_activated,
	occupied_and_activated,
}Power_off_state;

typedef enum
{
	no_defense=0,
	sentry_defence=1,
	invincible=3,
}Base_defense;

typedef union
{
	uint8_t event_type[4];
	struct
	{
		uint32_t apron : 1;
		uint32_t Blood_point1 : 1;
		uint32_t Blood_point2 : 1;
		uint32_t Blood_point3 : 1;
		uint32_t Energy_authorities : 2;
		uint32_t mark : 1;
		uint32_t bunker : 1;
		uint32_t Resource_island : 1;
		uint32_t Base_defenses : 2;
	}domain;
}ext_event_data_t;

/*Site_supply_station_action_identification_data*/
typedef enum 
{
	Shut_down=0,
	Ready_to_fire,
	bullet,
}bullet_state;

typedef struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
} ext_supply_projectile_action_t;

/*Robot_state_data*/
typedef union
{
	uint8_t mains_power_in;
	struct
	{
		uint8_t mains_power_gimbal_output : 1;
		uint8_t mains_power_chassis_output : 1;
		uint8_t mains_power_shooter_output : 1;
	}domain;
}mains_powertypedef;

typedef struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t remain_HP;
	uint16_t max_HP;
	uint16_t shooter_heat0_cooling_rate;
	uint16_t shooter_heat0_cooling_limit;
	uint16_t shooter_heat1_cooling_rate;
	uint16_t shooter_heat1_cooling_limit;
	mains_powertypedef mains_power;
} ext_game_robot_state_t;

/*Real_time_power_and_heat_data*/
typedef struct
{
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t chassis_power_buffer;
	uint16_t shooter_heat0;
	uint16_t shooter_heat1;
} ext_power_heat_data_t;

/*Robot_position_data*/
typedef struct
{
	float x;
	float y;
	float z;
	float yaw;
} ext_game_robot_pos_t;

/*Robot_gain_data*/


typedef union
{
	uint8_t buff_musk_in;
	struct
	{
		uint8_t blood : 1;
		uint8_t cooling_speed : 1;
		uint8_t Defense_bonus : 1;
		uint8_t Bonus_to_attack : 1;
	}domain;
}ext_buff_musk_t;

/*Air_robot_energy_status_data*/
typedef __packed struct
{
	uint8_t energy_point;
	uint16_t attack_time;
} aerial_robot_energy_t;

/*Damage_status_data*/
typedef enum
{
	Armor_damage = 0,
	Module_dropped_thread,
	Excess_muzzle_heat,
	Super_chassis_power,
}Types_of_blood;

typedef union
{
	uint8_t robot_hurt_in;
	struct
	{
		uint8_t armor_id : 4;
		uint8_t hurt_type : 4;
	} domain;
}ext_robot_hurt_t;

/*Real_time_shooting_data*/
typedef enum
{
	Little_projectile=1,
	big_projectile,
}projectiletype;

typedef __packed struct
{
	uint8_t bullet_type;
	uint8_t bullet_freq;
	float bullet_speed;
} ext_shoot_data_t;


extern ext_game_state_t                 ext_game_state;
extern ext_game_result_t                ext_game_result;
extern ext_game_robot_survivors_t       ext_game_robot_survivors;
extern ext_event_data_t                 ext_event_data;
extern ext_supply_projectile_action_t   ext_supply_projectile_action;
extern ext_game_robot_state_t           ext_game_robot_state;
extern ext_power_heat_data_t            ext_power_heat_data;
extern ext_game_robot_pos_t             ext_game_robot_pos;
extern ext_buff_musk_t                  ext_buff_musk;
extern ext_robot_hurt_t                 ext_robot_hurt;
extern ext_shoot_data_t                 ext_shoot_data;
extern aerial_robot_energy_t            aerial_robot_energy;


referee_system_StatusTypeDef referee_system_Rx(uint8_t * RxMsg, CRC_StatusTypeDef CRCSwitch);

void Match_status_data_processing(uint8_t *Msg);
void Result_data_processing(uint8_t *Msg);
void Match_robot_survival_data_processing(uint8_t *Msg);
void Site_event_data_processing(uint8_t *Msg);
void Site_supply_station_action_identification_data_processing(uint8_t *Msg);
void Robot_state_data_processing(uint8_t *Msg);
void Real_time_power_and_heat_data_processing(uint8_t *Msg);
void Robot_position_data_processing(uint8_t *Msg);
void Robot_gain_data_processing(uint8_t *Msg);
void Air_robot_energy_status_data_processing(uint8_t *Msg);
void Damage_status_data_processing(uint8_t *Msg);
void Real_time_shooting_data_processing(uint8_t *Msg);

#endif
