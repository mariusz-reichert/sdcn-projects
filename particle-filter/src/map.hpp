#pragma once

#include <vector>
#include <iosfwd>

namespace particle_filter_project {

struct Landmark {
  uint id = 0;
  double x = 0.0;
  double y = 0.0;
};

using Map = std::vector<Landmark>;

std::istream& operator>>(std::istream& ins, Landmark& l);

} // namespace particle_filter_project
