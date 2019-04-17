#include "referee_system.h"



ext_game_state_t                 ext_game_state;
ext_game_result_t                ext_game_result;
ext_game_robot_survivors_t       ext_game_robot_survivors;
ext_event_data_t                 ext_event_data;
ext_supply_projectile_action_t   ext_supply_projectile_action;
ext_game_robot_state_t           ext_game_robot_state;
ext_power_heat_data_t            ext_power_heat_data;
ext_game_robot_pos_t             ext_game_robot_pos;
ext_buff_musk_t                  ext_buff_musk;
ext_robot_hurt_t                 ext_robot_hurt;
ext_shoot_data_t                 ext_shoot_data;
aerial_robot_energy_t            aerial_robot_energy;

referee_system_StatusTypeDef referee_system_Rx(uint8_t * RxMsg, CRC_StatusTypeDef CRCSwitch)
{
	uint16_t CMD_code= (RxMsg[6] << 8) | RxMsg[5];
	uint8_t *Msg=&RxMsg[7];
	
	if (RxMsg[0] == 0xA5)
	{
		switch (CMD_code)
		{
#if Match_status_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Match_status_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_long_Match_status_data))
				{
					Match_status_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Match_status_data_processing(Msg);
				return OK;
			}
#endif

#if Result_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Result_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_long_Result_data))
				{
					Result_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Result_data_processing(Msg);
				return OK;
			}
#endif

#if Match_robot_survival_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Match_robot_survival_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_long_Match_robot_survival_data))
				{
					Match_robot_survival_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Match_robot_survival_data_processing(Msg);
				return OK;
			}
#endif

#if Site_event_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Site_event_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_long_Site_event_data))
				{
					Site_event_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Site_event_data_processing(Msg);
				return OK;
			}
#endif

#if Site_supply_station_action_identification_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Site_supply_station_action_identification_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_long_Site_supply_station_action_identification_data))
				{
					Site_supply_station_action_identification_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Site_supply_station_action_identification_data_processing(Msg);
				return OK;
			}
#endif

#if Robot_state_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Robot_state_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_long_Robot_state_data))
				{
					Robot_state_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Robot_state_data_processing(Msg);
				return OK;
			}
#endif

#if Real_time_power_and_heat_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Real_time_power_and_heat_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_long_Real_time_power_and_heat_data))
				{
					Real_time_power_and_heat_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Real_time_power_and_heat_data_processing(Msg);
				return OK;
			}
#endif

#if Robot_position_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Robot_position_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_long_Robot_position_data))
				{
					Robot_position_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Robot_position_data_processing(Msg);
				return OK;
			}
#endif

#if Robot_gain_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Robot_gain_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_long_Robot_gain_data))
				{
					Robot_gain_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Robot_gain_data_processing(Msg);
				return OK;
			}
#endif

#if Air_robot_energy_status_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Air_robot_energy_status_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_long_Air_robot_energy_status_data))
				{
					Air_robot_energy_status_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Air_robot_energy_status_data_processing(Msg);
				return OK;
			}
#endif

#if Damage_status_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Damage_status_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_long_Damage_status_data))
				{
					Damage_status_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Damage_status_data_processing(Msg);
				return OK;
			}
#endif

#if Real_time_shooting_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Real_time_shooting_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_long_Real_time_shooting_data))
				{
					Real_time_shooting_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Real_time_shooting_data_processing(Msg);
				return OK;
			}
#endif
		}
	}
	return Error;
}

void Match_status_data_processing(uint8_t *RxMsg)
{
	ext_game_state.Match_status_IN = RxMsg[0];
}

void Result_data_processing(uint8_t *Msg)
{
	ext_game_result.winner = Msg[0];
}

void Match_robot_survival_data_processing(uint8_t *Msg)
{
	ext_game_robot_survivors.robot_legion[0] = Msg[1];
	ext_game_robot_survivors.robot_legion[1] = Msg[0];
}

void Site_event_data_processing(uint8_t *Msg)
{
	ext_event_data.event_type[0] = Msg[0];
	ext_event_data.event_type[1] = Msg[1];
	ext_event_data.event_type[2] = Msg[2];
	ext_event_data.event_type[3] = Msg[3];
}

void Site_supply_station_action_identification_data_processing(uint8_t *Msg)
{
	ext_supply_projectile_action.supply_projectile_id = Msg[0];
	ext_supply_projectile_action.supply_robot_id = Msg[1];
	ext_supply_projectile_action.supply_projectile_step = Msg[2];
}

void Robot_state_data_processing(uint8_t *Msg)
{
	ext_game_robot_state.robot_id = Msg[0];
	ext_game_robot_state.robot_level = Msg[1];
	ext_game_robot_state.remain_HP = Msg[3] << 8 | Msg[2];
	ext_game_robot_state.max_HP = Msg[5] << 8 | Msg[4];
	ext_game_robot_state.shooter_heat0_cooling_rate = Msg[7] << 8 | Msg[6];
	ext_game_robot_state.shooter_heat0_cooling_limit = Msg[9]<<8|Msg[8];
	ext_game_robot_state.shooter_heat1_cooling_rate = Msg[11] << 8 | Msg[10];
	ext_game_robot_state.shooter_heat1_cooling_limit = Msg[13] << 8 | Msg[12];
	ext_game_robot_state.mains_power.mains_power_in = Msg[14];
}

void Real_time_power_and_heat_data_processing(uint8_t *Msg)
{
	uint8_t_to_float u8_to_float;
	ext_power_heat_data.chassis_volt = Msg[1] << 8 | Msg[0];
	ext_power_heat_data.chassis_current = Msg[3] << 8 | Msg[2];
	u8_to_float.buff[0] = Msg[4];
	u8_to_float.buff[1] = Msg[5];
	u8_to_float.buff[2] = Msg[6];
	u8_to_float.buff[3] = Msg[7];
	ext_power_heat_data.chassis_power = u8_to_float.floatout;
	ext_power_heat_data.chassis_power_buffer = Msg[9]<<8|Msg[8];
	ext_power_heat_data.shooter_heat0 = Msg[11] << 8 | Msg[10];
	ext_power_heat_data.shooter_heat1 = Msg[13] << 8 | Msg[12];
}

void Robot_position_data_processing(uint8_t *Msg)
{
	uint8_t_to_float u8_to_float;
	u8_to_float.buff[0] = Msg[0];
	u8_to_float.buff[1] = Msg[1];
	u8_to_float.buff[2] = Msg[2];
	u8_to_float.buff[3] = Msg[3];
	ext_game_robot_pos.x = u8_to_float.floatout;

	u8_to_float.buff[0] = Msg[4];
	u8_to_float.buff[1] = Msg[5];
	u8_to_float.buff[2] = Msg[6];
	u8_to_float.buff[3] = Msg[7];
	ext_game_robot_pos.y = u8_to_float.floatout;

	u8_to_float.buff[0] = Msg[8];
	u8_to_float.buff[1] = Msg[9];
	u8_to_float.buff[2] = Msg[10];
	u8_to_float.buff[3] = Msg[11];
	ext_game_robot_pos.z = u8_to_float.floatout;

	u8_to_float.buff[0] = Msg[12];
	u8_to_float.buff[1] = Msg[13];
	u8_to_float.buff[2] = Msg[14];
	u8_to_float.buff[3] = Msg[15];
	ext_game_robot_pos.yaw = u8_to_float.floatout;
}

void Robot_gain_data_processing(uint8_t *Msg)
{
	ext_buff_musk.buff_musk_in = Msg[0];
}

void Air_robot_energy_status_data_processing(uint8_t *Msg)
{
	aerial_robot_energy.energy_point = Msg[0];
	aerial_robot_energy.attack_time = Msg[2] << 8 | Msg[1];
}

void Damage_status_data_processing(uint8_t *Msg)
{
	ext_robot_hurt.robot_hurt_in = Msg[0];
}

void Real_time_shooting_data_processing(uint8_t *Msg)
{
	uint8_t_to_float u8_to_float;
	u8_to_float.buff[0] = Msg[2];
	u8_to_float.buff[1] = Msg[3];
	u8_to_float.buff[2] = Msg[4];
	u8_to_float.buff[3] = Msg[5];
	ext_shoot_data.bullet_type = Msg[0];
	ext_shoot_data.bullet_freq = Msg[1];
	ext_shoot_data.bullet_speed = u8_to_float.floatout;
	
}

