#include "filter.hpp"

using namespace FILTERS;

FilterBase::FilterBase(TIMING::Ticker &_ticker): ticker(_ticker){}

float FilterBase::calculate(float x){return x;}


FilterAlphaBeta::FilterAlphaBeta(TIMING::Ticker &_ticker): FilterBase(_ticker){
  alfa =0.2;
  beta = 0.1;
  ypri = 0;
  ypost = 0;
  vpri = 0;
  vpost = 0;
};


float FilterAlphaBeta::calculate(float x){
  float time = FilterBase::ticker.get_seconds();
  float dt = prev_time - time;
  ypri = ypost + dt*vpost;
  vpri = vpost;
  ypost = ypri + alfa*(x - ypri);
  vpost = vpri + beta*(x - ypri)/dt;
  prev_time =  time;
  return ypost;
}