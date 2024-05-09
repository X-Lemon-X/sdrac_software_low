#include "movement_controler.hpp"


namespace PDCONTROLER{

class PdControler: public MOVEMENT_CONTROLER::MovementEquation{
private:
  float Kp = 1.0;
  float Kd = 1.0;

  float previous_velocity = 0.0;
  float previous_position = 0.0;
  float previous_time = 0.0;
public:
  PdControler(TIMING::Ticker &ticker): MOVEMENT_CONTROLER::MovementEquation(ticker){};
  
  
  float calculate(float current_position, float target_position, float current_velocity, float target_velocity) override;
  void begin_state(float current_position, float current_velocity, float current_time) override;

  void set_Kp(float Kp){this->Kp = Kp;};
  void set_Kd(float Kd){this->Kd = Kd;};
};
  
} // namespace PD_Controler
