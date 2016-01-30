#include "instr.h"

namespace gca {

  ostream& operator<<(ostream& stream, const instr& i) {
    i.print(stream);
    return stream;
  }

}
