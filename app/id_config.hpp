#include "config_struct.hpp"
#include "CanDB.h"
#include <limits>

#ifndef CONFIG_HPP
#define CONFIG_HPP


// ids 11bit 0b110 0001 0000  and 18 bit 0b00 0000 0000 0000 0000
//mask 11bit 0b111 1111 0000  and 18 bit 0b00 0000 0000 0000 0000
// for some reason filter mask is not working properly so we have to do it in software
// why 5 bits shift because we want tu push the 11 bit id to the 16 bit regiser strting from 5th bit

const ID_CONFIG config_id_default ={

  .can_filter_mask_high = 0xff0,
  .can_filter_mask_low = 0x000,
  .can_filter_id_high = 0x600,
  .can_filter_id_low = 0x000,

  .can_konarm_status_frame_id = 0x601,
  .can_konarm_set_pos_frame_id = 0x602,
  .can_konarm_get_pos_frame_id = 0x603,
  .can_konarm_clear_errors_frame_id = 0x604,


  .stepper_motor_steps_per_rev = 400.0f,
  .stepper_motor_gear_ratio = 75.0f,
  .stepper_motor_max_velocity = 0.0f,
  .stepper_motor_min_velocity = 0.0f,
  .stepper_motor_reverse = false,

  .encoder_arm_offset = 0.0f,
  .encoder_arm_reverse = false,
  .encoder_arm_dead_zone_correction_angle = 0.0f,
  .encoder_arm_velocity_sample_amount=0,

  .encoder_motor_offset = 0.0f,
  .encoder_motor_reverse = false,
  .encoder_motor_dead_zone_correction_angle = 0.0f,
  .encoder_motor_velocity_sample_amount = 0,

  .pid_p = 0.0f,
  .pid_i = 0.0f,
  .pid_d = 0.0f,

  .movement_max_velocity = 0.0f,
  .movement_limit_lower = 0.0f,
  .movement_limit_upper = 0.0f,
};

const ID_CONFIG config_id_1 ={

  .can_filter_mask_high = 0xff0,
  .can_filter_mask_low = 0x000,
  .can_filter_id_high = 0x610,
  .can_filter_id_low = 0x000,

  .can_konarm_status_frame_id = CAN_KONARM_1_STATUS_FRAME_ID,
  .can_konarm_set_pos_frame_id = CAN_KONARM_1_SET_POS_FRAME_ID,
  .can_konarm_get_pos_frame_id = CAN_KONARM_1_GET_POS_FRAME_ID,
  .can_konarm_clear_errors_frame_id = CAN_KONARM_1_CLEAR_ERRORS_FRAME_ID,


  .stepper_motor_steps_per_rev = 400.0f,
  .stepper_motor_gear_ratio = 75.0f,
  .stepper_motor_max_velocity = PI,
  .stepper_motor_min_velocity = 0.1f,
  .stepper_motor_reverse = false,

  .encoder_arm_offset = -5.23547649f,
  .encoder_arm_reverse = true,
  .encoder_arm_dead_zone_correction_angle = 0.0f,
  .encoder_arm_velocity_sample_amount=0,

  .encoder_motor_offset = 0.0f,
  .encoder_motor_reverse = false,
  .encoder_motor_dead_zone_correction_angle = 0.0f,
  .encoder_motor_velocity_sample_amount = 10,

  .pid_p = 0.9f,
  .pid_i = 0.0f,
  .pid_d = 0.1f,

  .movement_max_velocity = PI,
  .movement_limit_lower = -PI,
  .movement_limit_upper = PI,
};

const ID_CONFIG config_id_2 = {
  .can_filter_mask_high = 0xff0,
  .can_filter_mask_low = 0x000,
  .can_filter_id_high = 0x620,
  .can_filter_id_low = 0x000,

  .can_konarm_status_frame_id = CAN_KONARM_2_STATUS_FRAME_ID,
  .can_konarm_set_pos_frame_id = CAN_KONARM_2_SET_POS_FRAME_ID,
  .can_konarm_get_pos_frame_id = CAN_KONARM_2_GET_POS_FRAME_ID,
  .can_konarm_clear_errors_frame_id = CAN_KONARM_2_CLEAR_ERRORS_FRAME_ID,

  .stepper_motor_steps_per_rev = 400.0f,
  .stepper_motor_gear_ratio = 75.0f,
  .stepper_motor_max_velocity = PI,
  .stepper_motor_min_velocity = 0.1f,
  .stepper_motor_reverse = false,

  .encoder_arm_offset = -4.219981f,
  .encoder_arm_reverse = false,
  .encoder_arm_dead_zone_correction_angle = PI,
  .encoder_arm_velocity_sample_amount=0,

  .encoder_motor_offset = 0.0f,
  .encoder_motor_reverse = false,
  .encoder_motor_dead_zone_correction_angle = 0.0f,
  .encoder_motor_velocity_sample_amount = 10,

  .pid_p = 0.9f,
  .pid_i = 0.0f,
  .pid_d = 0.1f,

  .movement_max_velocity = PI,
  .movement_limit_lower = -PI_d2,
  .movement_limit_upper = PI_d2,
};

const ID_CONFIG config_id_3 = {
  .can_filter_mask_high = 0xff0,
  .can_filter_mask_low = 0x000,
  .can_filter_id_high = 0x630,
  .can_filter_id_low = 0x000,

  .can_konarm_status_frame_id = CAN_KONARM_3_STATUS_FRAME_ID,
  .can_konarm_set_pos_frame_id = CAN_KONARM_3_SET_POS_FRAME_ID,
  .can_konarm_get_pos_frame_id = CAN_KONARM_3_GET_POS_FRAME_ID,
  .can_konarm_clear_errors_frame_id = CAN_KONARM_3_CLEAR_ERRORS_FRAME_ID,

  .stepper_motor_steps_per_rev = 400.0f,
  .stepper_motor_gear_ratio = 75.0f,
  .stepper_motor_max_velocity = PI,
  .stepper_motor_min_velocity = 0.1f,
  .stepper_motor_reverse = false,

  .encoder_arm_offset = -0.240067f,
  .encoder_arm_reverse = true,
  .encoder_arm_dead_zone_correction_angle = PI_m3d2,
  .encoder_arm_velocity_sample_amount=0,

  .encoder_motor_offset = 0.0f,
  .encoder_motor_reverse = false,
  .encoder_motor_dead_zone_correction_angle = 0.0f,
  .encoder_motor_velocity_sample_amount = 10,

  .pid_p = 0.9f,
  .pid_i = 0.0f,
  .pid_d = 0.1f,

  .movement_max_velocity = PI,
  .movement_limit_lower = -1.089126f,
  .movement_limit_upper =  4.236856f,
};

const ID_CONFIG config_id_4 = {
   .can_filter_mask_high = 0xff0,
  .can_filter_mask_low = 0x000,
  .can_filter_id_high = 0x640,
  .can_filter_id_low = 0x000,

  .can_konarm_status_frame_id = CAN_KONARM_4_STATUS_FRAME_ID,
  .can_konarm_set_pos_frame_id = CAN_KONARM_4_SET_POS_FRAME_ID,
  .can_konarm_get_pos_frame_id = CAN_KONARM_4_GET_POS_FRAME_ID,
  .can_konarm_clear_errors_frame_id = CAN_KONARM_4_CLEAR_ERRORS_FRAME_ID,

  .stepper_motor_steps_per_rev = 400.0f,
  .stepper_motor_gear_ratio = 71.9f,
  .stepper_motor_max_velocity = PI,
  .stepper_motor_min_velocity = 0.1f,
  .stepper_motor_reverse = false,

  .encoder_arm_offset = 0.0f,
  .encoder_arm_reverse = true,
  .encoder_arm_dead_zone_correction_angle = PI_m3d2,
  .encoder_arm_velocity_sample_amount=0,

  .encoder_motor_offset = 0.0f,
  .encoder_motor_reverse = false,
  .encoder_motor_dead_zone_correction_angle = 0.0f,
  .encoder_motor_velocity_sample_amount = 10,

  .pid_p = 0.9f,
  .pid_i = 0.0f,
  .pid_d = 0.1f,

  .movement_max_velocity = PI,
  .movement_limit_lower = std::numeric_limits<float>::min(),
  .movement_limit_upper =  std::numeric_limits<float>::max(),
};


const ID_CONFIG config_id_5 = {
   .can_filter_mask_high = 0xff0,
  .can_filter_mask_low = 0x000,
  .can_filter_id_high = 0x650,
  .can_filter_id_low = 0x000,

  .can_konarm_status_frame_id = CAN_KONARM_5_STATUS_FRAME_ID,
  .can_konarm_set_pos_frame_id = CAN_KONARM_5_SET_POS_FRAME_ID,
  .can_konarm_get_pos_frame_id = CAN_KONARM_5_GET_POS_FRAME_ID,
  .can_konarm_clear_errors_frame_id = CAN_KONARM_5_CLEAR_ERRORS_FRAME_ID,

  .stepper_motor_steps_per_rev = 400.0f,
  .stepper_motor_gear_ratio = 71.9f,
  .stepper_motor_max_velocity = PI,
  .stepper_motor_min_velocity = 0.1f,
  .stepper_motor_reverse = false,

  .encoder_arm_offset = 0.0f,
  .encoder_arm_reverse = true,
  .encoder_arm_dead_zone_correction_angle = PI_m3d2,
  .encoder_arm_velocity_sample_amount=0,

  .encoder_motor_offset = 0.0f,
  .encoder_motor_reverse = false,
  .encoder_motor_dead_zone_correction_angle = 0.0f,
  .encoder_motor_velocity_sample_amount = 10,

  .pid_p = 0.9f,
  .pid_i = 0.0f,
  .pid_d = 0.1f,

  .movement_max_velocity = PI,
  .movement_limit_lower = std::numeric_limits<float>::min(),
  .movement_limit_upper =  std::numeric_limits<float>::max(),
};

const ID_CONFIG config_id_6 = {
   .can_filter_mask_high = 0xff0,
  .can_filter_mask_low = 0x000,
  .can_filter_id_high = 0x660,
  .can_filter_id_low = 0x000,

  .can_konarm_status_frame_id = CAN_KONARM_5_STATUS_FRAME_ID,
  .can_konarm_set_pos_frame_id = CAN_KONARM_5_SET_POS_FRAME_ID,
  .can_konarm_get_pos_frame_id = CAN_KONARM_5_GET_POS_FRAME_ID,
  .can_konarm_clear_errors_frame_id = CAN_KONARM_5_CLEAR_ERRORS_FRAME_ID,

  .stepper_motor_steps_per_rev = 400.0f,
  .stepper_motor_gear_ratio = 71.9f,
  .stepper_motor_max_velocity = PI,
  .stepper_motor_min_velocity = 0.1f,
  .stepper_motor_reverse = false,

  .encoder_arm_offset = 0.0f,
  .encoder_arm_reverse = true,
  .encoder_arm_dead_zone_correction_angle = PI_m3d2,
  .encoder_arm_velocity_sample_amount=0,

  .encoder_motor_offset = 0.0f,
  .encoder_motor_reverse = false,
  .encoder_motor_dead_zone_correction_angle = 0.0f,
  .encoder_motor_velocity_sample_amount = 10,

  .pid_p = 0.9f,
  .pid_i = 0.0f,
  .pid_d = 0.1f,

  .movement_max_velocity = PI,
  .movement_limit_lower = std::numeric_limits<float>::min(),
  .movement_limit_upper =  std::numeric_limits<float>::max(),
};


#endif