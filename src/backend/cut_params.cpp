#include "backend/cut_params.h"

#include <cmath>

namespace gca {

  double chip_load(const double spindle_speed,
		   const double feedrate_ipm,
		   const double num_flutes) {
    return feedrate_ipm / (spindle_speed * num_flutes);
  }

  double surface_feet_per_minute(const double spindle_speed,
				 const double tool_diameter_inches) {
    double tool_diameter_feet = tool_diameter_inches / 12.0;
    double tool_circumference_feet = M_PI*tool_diameter_feet;

    return tool_circumference_feet * spindle_speed;
  }

  
}
