#include "gcode/value.h"

namespace gca {

  ostream& operator<<(ostream& s, const value& v) {
    v.print(s);
    return s;
  }

}
