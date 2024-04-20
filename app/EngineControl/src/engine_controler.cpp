
#include "main.h"
#include "stm32f4xx_hal.h"
#include "engine_controler.hpp"


using namespace ENGINE_CONTROLER;


EngineControler::EngineControler(TIMING::Ticker &_ticker, STEPER_MOTOR::SteperMotor &_steper_motor, ENCODER::Encoder &_encoder):
ticker(ticker),steper_motor(steper_motor),encoder(encoder){}

void EngineControler::init(){
}

void EngineControler::handler(){
}

void EngineControler::set_velocity(float speed){
  steper_motor.set_speed(speed);
}

void EngineControler::set_enable(bool enable){
  steper_motor.set_enable(enable);
}

void EngineControler::set_position(float position){
}