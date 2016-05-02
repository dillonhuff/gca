#ifndef GCA_MILLABILITY_H
#define GCA_MILLABILITY_H

#include "geometry/triangular_mesh.h"

namespace gca {

  std::vector<index_t> millable_faces(const point normal,
				      const triangular_mesh& part);
}

#endif
