#include "config_struct.hpp"
#include "main_prog.hpp"

#ifndef CONFIG_HPP
#define CONFIG_HPP

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

#endif