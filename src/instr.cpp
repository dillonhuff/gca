#include "instr.h"

namespace gca {

  bool instr::operator==(const instr& other) {
    if (c != other.c || v != other.v) {
      return false;
    }
    if (c == GCA_G) {
      return x == other.x && y == other.y && z == other.z;
    }
    return true;
  }
  
  void instr::print(ostream& s) {
    if (c == GCA_M) {
      cout << 'M' << v;
    } else if (c == GCA_G) {
      cout << 'G' << v;
      s << " X" << x;
      s << " Y" << y;
      s << " Z" << z;
    } else {
      assert(false);
    }
  }

  ostream& operator<<(ostream& stream, instr& i) {
    i.print(stream);
    return stream;
  }

}
