#pragma once

namespace pid_controller_project {

class PID {
  double p_error_;
  double i_error_;
  double d_error_;

  double Kp_;
  double Ki_;
  double Kd_;

public:

  PID(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

} // namespace pid_controller_project
