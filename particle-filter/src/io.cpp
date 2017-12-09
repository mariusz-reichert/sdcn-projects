#include <fstream>

#include "map.hpp"

namespace particle_filter_project {

using std::istream;

istream& operator>>(istream& ins, Landmark& l) {
  ins >> l.x
      >> l.y
      >> l.id;

  return ins;
}

} // namespace particle_filter_project
