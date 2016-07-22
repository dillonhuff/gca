#ifndef GCA_MESH_OPERATIONS_H
#define GCA_MESH_OPERATIONS_H

#include "geometry/plane.h"
#include "geometry/triangular_mesh.h"

namespace gca {

  triangular_mesh
  clip_mesh(const triangular_mesh& m, const plane pl);

}

#endif
