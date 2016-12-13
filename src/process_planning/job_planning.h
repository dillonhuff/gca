#pragma once

#include "process_planning/direction_selection.h"

namespace gca {

  std::vector<fixture_setup> plan_jobs(const triangular_mesh& stock,
				       const triangular_mesh& part,
				       const fixtures& f,
				       const std::vector<tool>& tools);

}
