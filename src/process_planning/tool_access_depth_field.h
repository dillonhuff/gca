#pragma once

#include "backend/tool.h"
#include "geometry/surface.h"

namespace gca {

  std::vector<surface> accessable_surfaces(const triangular_mesh& m,
					   const tool& t);
}
