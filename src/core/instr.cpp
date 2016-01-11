#include "instr.h"

namespace gca {

  bool instr::operator==(const instr& other) {
    if (c != other.c || v != other.v) {
      return false;
    }
    return true;
  }
  
  void instr::print(ostream& s) const {
    if (c == GCA_M) {
      cout << 'M' << v;
    } else if (is_G()) {
      s << 'G' << v;
    } else if (is_G()) {
      cout << 'G' << v;
    } else if (is_F()) {
      cout << 'F' << v;
    } else if (is_T()) {
      cout << 'T' << v;
    } else if (is_S()) {
      cout << 'S' << v;
    } else {
      assert(false);
    }
  }

  ostream& operator<<(ostream& stream, const instr& i) {
    i.print(stream);
    return stream;
  }

}
