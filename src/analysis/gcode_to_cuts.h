#ifndef GCA_GCODE_TO_CUTS_H
#define GCA_GCODE_TO_CUTS_H

#include "core/lexer.h"
#include "synthesis/cut.h"
#include "synthesis/machine.h"

namespace gca {

  enum gcode_to_cuts_result {
    GCODE_TO_CUTS_SUCCESS,
    GCODE_TO_CUTS_PATHOLOGICAL_TOOLPATH,
    GCODE_TO_CUTS_UNSUPPORTED_SETTINGS
  };

  ostream& operator<<(ostream& out, const gcode_to_cuts_result r);

  gcode_to_cuts_result gcode_to_cuts(const vector<block>& blocks, vector<vector<cut*>>& cuts);

}

#endif
