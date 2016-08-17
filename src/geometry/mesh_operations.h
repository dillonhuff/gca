#ifndef GCA_MESH_OPERATIONS_H
#define GCA_MESH_OPERATIONS_H

#include "geometry/plane.h"
#include "geometry/triangular_mesh.h"

namespace gca {

  std::vector<oriented_polygon>
  mesh_cross_section(const triangular_mesh& m,
		     const plane p);
  
  triangular_mesh
  clip_mesh(const triangular_mesh& m, const plane pl);

  triangular_mesh
  boolean_difference(const triangular_mesh& a, const triangular_mesh& b);
}

#endif
