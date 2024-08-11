#include "movement_controler.hpp"


namespace CONTROLER{

class BasicControler: public MOVEMENT_CONTROLER::MovementEquation{
private:
  float max_acceleration;
  float target_pos_max_error;
  float previous_velocity;
  float previous_position;
  float previous_time;
  float max_velocity;
public:
  BasicControler(TIMING::Ticker &ticker);
  
  void begin_state(float current_position, float current_velocity, float current_time) override;
  float calculate(float current_position, float target_position, float current_velocity, float target_velocity) override;
  
  void set_max_acceleration(float max_acceleration);
  void set_target_pos_max_error(float target_pos_max_error);
  void set_max_velocity(float max_velocity);
};
  
} // namespace CONTROLER