#include <cassert>

#include "synthesis/machine.h"

namespace gca {

  ostream& operator<<(ostream& out, const tool_name t) {
    switch (t) {
    case DRILL:
      out << "DRILL";
      break;
    case DRAG_KNIFE:
      out << "DRAG_KNIFE";
      break;
    default:
      out << "NO_TOOL";
      break;
      //      assert(false);
    }
    return out;
  }

  ostream& operator<<(ostream& out, const machine_name t) {
    switch (t) {
    default:
      cout << "Unsupported machine_name" << endl;
      assert(false);
    }
    return out;
  }

}
