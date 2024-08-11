#include "movement_controler.hpp"


namespace PDCONTROLER{

class PdControler: public MOVEMENT_CONTROLER::MovementEquation{
private:
  float Kp;
  float Kd;
  float Ki;
  float previous_velocity;
  float previous_position;
  float previous_time;
public:
  PdControler(TIMING::Ticker &ticker);
  
  void begin_state(float current_position, float current_velocity, float current_time) override;
  float calculate(float current_position, float target_position, float current_velocity, float target_velocity) override;
  
  void set_Kp(float Kp){this->Kp = Kp;};
  void set_Kd(float Kd){this->Kd = Kd;};
  void set_Ki(float Ki){this->Ki = Ki;};
};
  
} // namespace PD_Controler
