#ifndef GCA_CUT_TO_GCODE_H
#define GCA_CUT_TO_GCODE_H

#include "core/gprog.h"
#include "synthesis/cut.h"
#include "synthesis/shape_layout.h"

namespace gca {

  void append_cut(cut* ci, gprog& p, const cut_params& params);
  void append_cut(cut* ci, gprog& p);
}

#endif
