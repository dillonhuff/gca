#pragma once

#include "geometry/triangular_mesh.h"

namespace gca {

  int analyze(const triangular_mesh& m,
	      const std::vector<index_t> faces_touching_fixed_jaw,
	      const std::vector<index_t> faces_touching_clamp_jaw);

}
