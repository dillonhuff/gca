#include "synthesis/cut.h"

namespace gca {

  ostream& operator<<(ostream& stream, const cut& c) {
    c.print(stream);
    return stream;
  }

  ostream& operator<<(ostream& stream, const vector<cut*>& c) {
    for (vector<cut*>::const_iterator it = c.begin(); it != c.end(); ++it) {
      stream << **it << endl;
    }
    return stream;
  }

}
