#ifndef GCA_EXTRUSION_H
#define GCA_EXTRUSION_H

#include "geometry/mesh_operations.h"

namespace gca {

  typedef std::vector<index_t> index_poly;
  
  triangular_mesh
  extrude_layers(const std::vector<point>& pts,
		 const std::vector<index_poly>& poly_layers,
		 const std::vector<double>& layer_depths,
		 const point extrude_dir);
}

#endif
