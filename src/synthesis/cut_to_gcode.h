#ifndef GCA_CUT_TO_GCODE_H
#define GCA_CUT_TO_GCODE_H

#include "core/gprog.h"
#include "core/lexer.h"
#include "synthesis/cut.h"
#include "synthesis/shape_layout.h"

namespace gca {

  vector<block> gcode_blocks_for_cuts(const vector<cut*>& cuts,
				      const cut_params& params);
  void append_cut(const cut* ci, gprog& p);
  void append_cut_with_settings(const cut* last,
				const cut* next,
				gprog& p,
				const cut_params& params);
}

#endif
