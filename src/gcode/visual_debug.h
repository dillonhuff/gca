#pragma once

#include "gcode/cut.h"

namespace gca {

  void vtk_debug_cuts(const std::vector<cut*>& cuts);

  std::vector<polyline> cuts_to_polylines(const std::vector<cut*>& cuts);

}
