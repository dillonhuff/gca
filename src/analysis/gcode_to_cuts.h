#ifndef GCA_GCODE_TO_CUTS_H
#define GCA_GCODE_TO_CUTS_H

#include "core/gprog.h"
#include "core/instrs/instr.h"
#include "core/lexer.h"
#include "synthesis/cut.h"
#include "synthesis/machine.h"

namespace gca {

  struct gcode_settings {
    orientation initial_coord_orient;
    point initial_pos;
    point initial_orient;
    tool_name initial_tool;

    bool sanity_check() const;
  };

  vector<cut*> gcode_to_cuts(gprog& p, const gcode_settings& settings);

  vector<cut*> gcode_to_cuts(const vector<block>& blocks);
}

#endif
