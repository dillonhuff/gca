#ifndef GCA_HOMOGENEOUS_TRANSFORMATION_H
#define GCA_HOMOGENEOUS_TRANSFORMATION_H

#include <boost/optional.hpp>

#include "geometry/matrix.h"
#include "geometry/plane.h"
#include "geometry/triangular_mesh.h"

namespace gca {

  typedef std::pair<const ublas::matrix<double>, const ublas::vector<double>>
    homogeneous_transform;

  triangular_mesh apply(const homogeneous_transform& t, const triangular_mesh& m);

  boost::optional<homogeneous_transform>
  mate_planes(const plane a, const plane b, const plane c,
	      const plane ap, const plane bp, const plane cp);

}

#endif