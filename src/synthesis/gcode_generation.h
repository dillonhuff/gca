#ifndef GCA_GCODE_GENERATION_H
#define GCA_GCODE_GENERATION_H

#include "gcode/cut.h"
#include "geometry/polyline.h"
#include "synthesis/toolpath.h"

namespace gca {

  std::vector<cut*> polyline_cuts(const polyline& p);

  std::vector<block> emco_f1_code(const toolpath& pocket_lines);


}

#endif
