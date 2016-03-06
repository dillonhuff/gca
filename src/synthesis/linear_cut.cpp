#include "synthesis/linear_cut.h"

namespace gca {

  void linear_cut::print(ostream& other) const {
    other << "LINEAR CUT: " << tool_no << " ";
    if (!feedrate->is_omitted()) {
      other << "F" << *feedrate << " ";
    } else {
      other << "<F omitted> ";
    }
    if (!spindle_speed->is_omitted()) {
      other << "S" << *spindle_speed << " ";
    } else {
      other << "<S omitted> ";
    }
    other << start << " -> " << end;
  }

}
