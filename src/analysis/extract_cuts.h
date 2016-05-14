#ifndef GCA_EXTRACT_CUTS_H
#define GCA_EXTRACT_CUTS_H

#include "analysis/machine_state.h"
#include "gcode/lexer.h"

namespace gca {

  void extract_cuts(const vector<block> blocks, vector<vector<machine_state>>& ms);

}

#endif
