#pragma once

#include "geometry/polygon.h"
#include "geometry/polygon_3.h"
#include "geometry/triangular_mesh.h"

namespace gca {

  vector<oriented_polygon> mesh_bounds(const vector<index_t>& faces,
				       const triangular_mesh& mesh);

  oriented_polygon max_area_outline(const std::vector<index_t>& inds,
				    const triangular_mesh& m);

  
}
