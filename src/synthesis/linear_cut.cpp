#include "synthesis/linear_cut.h"

namespace gca {

  void linear_cut::print(ostream& other) const {
    other << "LINEAR CUT: " << tool_no << " ";
    if (!get_feedrate()->is_omitted()) {
      other << "F" << *get_feedrate() << " ";
    } else {
      other << "<F omitted> ";
    }
    if (!get_spindle_speed()->is_omitted()) {
      other << "S" << *get_spindle_speed() << " ";
    } else {
      other << "<S omitted> ";
    }
    other << get_start() << " -> " << get_end();
  }

}
