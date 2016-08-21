#ifndef GCA_ROTATION_H
#define GCA_ROTATION_H

#include "geometry/matrix.h"
#include "geometry/triangular_mesh.h"

namespace gca {

  typedef ublas::matrix<double> rotation;

  rotation rotate_from_to(const point from, const point to);

  triangular_mesh apply(const rotation& r, const triangular_mesh& m); 
}

#endif
