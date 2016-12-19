#pragma once

#include "geometry/triangular_mesh.h"

namespace gca {

  struct mandatory_volume {
    triangular_mesh volume;
    point direction;
  };

  std::vector<std::vector<mandatory_volume> >
  mandatory_volumes(const triangular_mesh& part);
  
}
