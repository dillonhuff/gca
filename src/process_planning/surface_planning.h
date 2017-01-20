#pragma once

#include "geometry/triangular_mesh.h"

namespace gca {

  class surface_milling_constraints {
  public:
    bool has_unmillable_inside_corner() const { return true; }
  };

  surface_milling_constraints
  build_surface_milling_constraints(const triangular_mesh& part);

}
