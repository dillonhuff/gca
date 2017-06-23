#pragma once

#include "geometry/triangular_mesh.h"

namespace gca {

  void write_to_ply(const triangular_mesh& m, const std::string& path);

}
