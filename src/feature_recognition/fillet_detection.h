#pragma once

#include "geometry/surface.h"

namespace gca {

  std::vector<std::vector<surface> > detect_fillets(const triangular_mesh& m);
}
