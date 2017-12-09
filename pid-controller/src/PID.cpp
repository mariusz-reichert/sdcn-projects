#include "PID.hpp"

using namespace std;

namespace pid_controller_project {

PID::PID(double Kp, double Ki, double Kd)
: p_error_{0.0}
, i_error_{0.0}
, d_error_{0.0}
, Kp_{Kp}
, Ki_{Ki}
, Kd_{Kd}
{}

void PID::UpdateError(double cte) {
  d_error_  = cte - p_error_;
  p_error_  = cte;
  i_error_ += cte;
}

double PID::TotalError() {
  return Kp_*p_error_ + Ki_*i_error_ + Kd_*d_error_;
}

} // namespace pid_controller_project
