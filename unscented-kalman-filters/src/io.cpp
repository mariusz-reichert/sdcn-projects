#include <fstream>

#include "state.hpp"
#include "measurement.hpp"
#include "tools.hpp"

namespace unscented_kalman_filter_project {

using std::istream;
using std::ostream;

istream& operator>>(istream& ins, State& x) {
  ins >> x(0)   /* px */
      >> x(1)   /* py */
      >> x(2)   /* vx */
      >> x(3);  /* vy */

  return ins;
}


ostream& operator<<(ostream& outs, const State& x) {
  outs << x(0) << "\n"   /* px */
       << x(1) << "\n"   /* py */
       << x(2) << "\n"   /* vx */
       << x(3) << "\n";  /* vy */

  return outs;
}


istream& operator>>(istream& ins, Measurement& m) {
  ins >> m.value(0) 
      >> m.value(1);

  if (m.source == SensorType::kRadar) {
    ins >> m.value(2);
  } 

  ins >> m.timestamp;
  
  return ins;
}


ostream& operator<<(ostream& outs, const Measurement& m) {
  if (m.source == SensorType::kLaser) {
    outs << m.value(0) << "\t"
         << m.value(1) << "\t";
  }
  else {
    const auto xy = tools::ConvertFromPolarToCartesian(m.value(0), 
                                                       m.value(1));

    outs << xy.first << "\t"
         << xy.second << "\t";
  }

  return outs;
}

} // namespace unscented_kalman_filter_project 
