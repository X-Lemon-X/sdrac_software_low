
#ifndef CONFIG_STRUCT_HPP
#define CONFIG_STRUCT_HPP

struct ID_CONFIG {

// CAN 
uint32_t can_filter_mask_high;
uint32_t can_filter_mask_low;
uint32_t can_filter_id_high;
uint32_t can_filter_id_low;

uint32_t can_konarm_status_frame_id;
uint32_t can_konarm_set_pos_frame_id;
uint32_t can_konarm_get_pos_frame_id;
uint32_t can_konarm_clear_errors_frame_id;

// Steper motor config
float stepper_motor_steps_per_rev;
float stepper_motor_gear_ratio;
float stepper_motor_max_velocity;
float stepper_motor_min_velocity;
bool stepper_motor_reverse;

// Encoder pos arm
float encoder_arm_offset;
bool  encoder_arm_reverse;
float encoder_arm_dead_zone_correction_angle;
uint16_t encoder_arm_velocity_sample_amount;

// Encoder pos motor
float encoder_motor_offset;
bool  encoder_motor_reverse;
float encoder_motor_dead_zone_correction_angle;
uint16_t encoder_motor_velocity_sample_amount;

// pid config
float pid_p;
float pid_i;
float pid_d;

//--------------------Movement config

/// @brief Maximum velocity of the arm
float movement_max_velocity;
/// @brief upper limit position of the arm
float movement_limit_lower;
/// @brief lower limit position of the arm
float movement_limit_upper;



};

#endif