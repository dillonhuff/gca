#pragma once

#include "gcode/cut.h"

namespace gca {

  std::vector<cut*> clip_transition_heights(const vector<cut*>& path,
					    double new_safe_height);

  std::vector<std::vector<cut*>>
  clip_transition_heights(std::vector<std::vector<cut*>>& paths,
			  double new_safe_height);

}
