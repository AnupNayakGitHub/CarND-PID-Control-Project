#ifndef PID_H
#define PID_H

#include "twiddler.h"

class PID {
//public:
private:
  // Errors
  double p_error;
  double i_error;
  double d_error;

  double speed;

  // Coefficients
  double Kp;
  double Ki;
  double Kd;

  bool tune;
  Twiddler twdlr;
  typedef enum STATES {
    STABILIZE,
    PD_CORERCTION,
    I_CORRECTION,
    TUNED
  } TWDLR_STATES;
  TWDLR_STATES state;

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

#endif /* PID_H */
