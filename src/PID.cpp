#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID():p_error(0.0), i_error(0.0), d_error(0.0),
           Kp(0.0), Ki(0.0), Kd(0.0), tune(false), state(TUNED) {}

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
  this->tune = true;
  state = STABILIZE;
  //twdlr_st = 0;
  twdlr.insert(Kp, 1);
  twdlr.insert(Kd, 1);
  twdlr.set_steps_per_iteration(steps_per_adj);
  twdlr.tolerance = tol;
}

bool PID::IsTuning() {
  //return (tune && ((!twdlr.isStable()) || (twdlr_st != 3)));
  return (tune);
}
unsigned int PID::TwiddlerAdjustments() {
  return twdlr.iterations;
}
void PID::ResetTwiddlerAdjustments() {
  twdlr.iterations = 0;
}

bool PID::NeedsReset() {
  //return (twdlr_st == 0);
  return (state == STABILIZE);
}

void PID::UpdateError(double cte, double speed) {
  if (!tune) ;
  else if(!twdlr.isStable()) {
    twdlr.twiddle(cte);
    cout << twdlr << endl;
    auto ks = twdlr.get_gains();
    if (state == STABILIZE) {
      Kp = ks[0];
      Kd = ks[1];
    }
    else if (state == PD_CORERCTION) {
      Kp = ks[0];
      Kd = ks[1];
    }
    else if (state == I_CORRECTION) {
      Ki = ks[0];
    }
  }
  else if (state == STABILIZE){
    twdlr.reset();
    twdlr.insert(Kp, 0.1);
    twdlr.insert(Kd, 0.1);
    twdlr.set_steps_per_iteration(100);
    twdlr.tolerance = 0.001;
    state = PD_CORERCTION;
  }
  else if (state == PD_CORERCTION){
    twdlr.reset();
    twdlr.insert(Ki, 0.001);
    twdlr.set_steps_per_iteration(100);
    twdlr.tolerance = 0.0001;
    state = I_CORRECTION;
    i_error = 0;
  }
  else if (state == I_CORRECTION) {
    state = TUNED;
    tune = false;
  }

  cout << "Ks  [" << Kp << ", " << Ki <<", " << Kd << "], Tuning state " << state << endl;
  cout << "Errors  [" << p_error << ", " << i_error <<", " << d_error << "]" << endl;
  d_error = cte - p_error;
  p_error = cte;
  i_error += (cte);
  this->speed = speed;
}

double PID::TotalError() {
  return (-p_error*Kp - i_error*Ki -speed*d_error*Kd);
}

