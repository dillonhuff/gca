#ifndef GCA_AXIS_LOCATION_H
#define GCA_AXIS_LOCATION_H

#include "geometry/triangular_mesh.h"

namespace gca {

  point part_axis(const triangular_mesh& m);

  point part_axis(const std::vector<index_t>& viable_inds,
		  const triangular_mesh& part);

}

#endif
