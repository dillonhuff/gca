#ifndef GCA_RETARGET_H
#define GCA_RETARGET_H

#include "analysis/gcode_to_cuts.h"
#include "core/lexer.h"
#include "synthesis/cut.h"
#include "synthesis/shapes_to_gcode.h"

namespace gca {

  vector<cut*> retarget_toolpath(vector<cut*>& path);
  vector<block> generate_gcode(vector<vector<cut*>> paths);
  vector<vector<block>> haas_to_minimill(vector<block>& p);
  
}

#endif
