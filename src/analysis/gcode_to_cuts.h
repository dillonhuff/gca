#ifndef GCA_GCODE_TO_CUTS_H
#define GCA_GCODE_TO_CUTS_H

#include "core/gprog.h"
#include "core/instrs/instr.h"
#include "synthesis/cut.h"

namespace gca {

  struct gcode_settings {
    orientation initial_coord_orient;
    point initial_pos;
    point initial_orient;
  };

  vector<cut*> gcode_to_cuts(const gprog& p, const gcode_settings& settings);
}

#endif
