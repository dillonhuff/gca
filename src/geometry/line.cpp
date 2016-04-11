#include "geometry/line.h"

namespace gca {
  
  ostream& operator<<(ostream& out, line l) {
    cout << l.start << " -> " << l.end << endl;
    return out;
  }
}
