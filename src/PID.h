#ifndef PID_H
#define PID_H

#include "twiddler.h"

class PID {
public:
  typedef enum STATES {
    STABILIZE,
    PD_CORRECTION,
    I_CORRECTION,
    TUNED
  } TWDLR_STATES;

private:
  // Errors
  double p_error = 0.0;
  double i_error = 0.0;
  double d_error = 0.0;

  double speed;

  // Coefficients
  double Kp = 0.0;
  double Ki = 0.0;
  double Kd = 0.0;

  Twiddler twdlr;
  TWDLR_STATES state = TUNED;

  // Internal method to set the tuning parameters
  void Tune(double tol, unsigned int steps_per_adj);

public:
  // Constructor
  PID();

  // Destructor.
  virtual ~PID();

  // Initialize PID.
  void Init(double Kp, double Ki, double Kdi);

  // Tune the hyper parameters
  void Tune();

  // Update the PID error variables given cross track error.
  void UpdateError(double cte, double speed);

  // Calculate the total PID error.
  double TotalError();

  // Check if it is still tuning
  bool IsTuning();
  // Find the number of adjustmenst it has done so far
  unsigned int TwiddlerAdjustments();

  // Reset the adjustments
  void ResetTwiddlerAdjustments();

  //Find out if the the simulator need to be reset
  bool NeedsReset();
};
std::ostream& operator << (std::ostream& os, const PID::TWDLR_STATES& state);

#endif /* PID_H */
