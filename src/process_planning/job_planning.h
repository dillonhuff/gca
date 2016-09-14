#pragma once

#include "synthesis/fixture_analysis.h"

namespace gca {

  std::vector<fixture_setup> plan_jobs(const triangular_mesh& stock,
				       const triangular_mesh& part,
				       const fixtures& f,
				       const std::vector<tool>& tools);

}
