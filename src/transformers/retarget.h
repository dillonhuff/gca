#ifndef GCA_RETARGET_H
#define GCA_RETARGET_H

#include <map>

#include "analysis/gcode_to_cuts.h"
#include "core/lexer.h"
#include "synthesis/cut.h"
#include "synthesis/shapes_to_gcode.h"

namespace gca {

  struct tool_info {
    double length, diameter;

    tool_info() : length(-1), diameter(-1) {}

    tool_info(double lengthp, double diameterp) :
      length(lengthp), diameter(diameterp) {}
  };

  typedef map<int, tool_info> tool_table;
  
  vector<cut*> retarget_toolpath(const vector<cut*>& path);
  vector<block> generate_gcode(const vector<cut*>& paths);
  
  vector<vector<cut*>> haas_to_minimill(const vector<vector<cut*>> & p,
					const tool_table& old_tools,
					const tool_table& new_tools);
  
  vector<vector<block>> haas_to_minimill(const vector<block>& p,
					 const tool_table& old_tools,
					 const tool_table& new_tools);
  
}

#endif
