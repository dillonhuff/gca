#ifndef GCA_GCODE_TO_CUTS_H
#define GCA_GCODE_TO_CUTS_H

#include "core/lexer.h"
#include "synthesis/cut.h"
#include "synthesis/machine.h"

namespace gca {

  vector<cut*> gcode_to_cuts(const vector<block>& blocks);

}

#endif
