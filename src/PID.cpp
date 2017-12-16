#include "PID.h"
#include <iostream>

using namespace std;

PID::PID() {}
PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::Tune(){
  Tune(0.1, 100);
}

void PID::Tune(double tol, unsigned int steps_per_adj) {
  state = STABILIZE;
  twdlr.insert(Kp, 1);
  twdlr.insert(Kd, 1);
  twdlr.set_steps_per_iteration(steps_per_adj);
  twdlr.tolerance = tol;
}

bool PID::IsTuning() {
  return (state != TUNED);
}
unsigned int PID::TwiddlerAdjustments() {
  return twdlr.iterations;
}
void PID::ResetTwiddlerAdjustments() {
  twdlr.iterations = 0;
}

bool PID::NeedsReset() {
  return (state == STABILIZE);
}

void PID::UpdateError(double cte, double speed) {
  if (state == TUNED) ;
  else if(!twdlr.isStable()) {
    twdlr.twiddle(cte);
    cout << twdlr << endl;
    auto ks = twdlr.get_gains();
    if (state == STABILIZE) {
      Kp = ks[0];
      Kd = ks[1];
    }
    else if (state == PD_CORRECTION) {
      Kp = ks[0];
      Kd = ks[1];
    }
    else if (state == I_CORRECTION) {
      Ki = ks[0];
    }
  }
  // The PID Controller stablilized
  else if (state == STABILIZE){
    // Prepare for next state
    twdlr.reset();
    twdlr.insert(Kp, 0.1);
    twdlr.insert(Kd, 0.1);
    twdlr.set_steps_per_iteration(100);
    twdlr.tolerance = 0.001;
    state = PD_CORRECTION;
  }
  // Proportional and Derivative corrections are complete
  else if (state == PD_CORRECTION){
    // Prepare for next state
    twdlr.reset();
    twdlr.insert(Ki, 0.001);
    twdlr.set_steps_per_iteration(100);
    twdlr.tolerance = 0.0001;
    state = I_CORRECTION;
    i_error = 0;
  }
  // Integration correction is complete
  else if (state == I_CORRECTION) {
    // Its tuned
    state = TUNED;
  }

  cout << "Ks  [" << Kp << ", " << Ki <<", " << Kd << "], Tuning state : " << state << endl;
  cout << "Errors  [" << p_error << ", " << i_error <<", " << d_error << "]" << endl;
  d_error = cte - p_error;
  p_error = cte;
  i_error += (cte);
  this->speed = speed;
}

double PID::TotalError() {
  return (-p_error*Kp - i_error*Ki -speed*d_error*Kd);
}

std::ostream& operator << (std::ostream& os, const PID::TWDLR_STATES& state) {
  switch(state) {
  case PID::TWDLR_STATES::STABILIZE: os << "STABILIZE"; break;
  case PID::TWDLR_STATES::PD_CORRECTION: os << "PD_CORRECTION"; break;
  case PID::TWDLR_STATES::I_CORRECTION: os << "I_CORRECTION"; break;
  case PID::TWDLR_STATES::TUNED: os << "TUNED"; break;
  }
  return os;
}

