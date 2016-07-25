#include <boost/numeric/ublas/io.hpp>

#include "geometry/homogeneous_transformation.h"

namespace gca {
  
  triangular_mesh apply(const homogeneous_transform& t, const triangular_mesh& m) {
    triangular_mesh rotated =
      m.apply([t](const point p)
	      { return times_3(t.first, p); });
    triangular_mesh shifted =
      rotated.apply_to_vertices([t](const point p)
				{ return p + from_vector(t.second); });
    return shifted;
  }

  boost::optional<homogeneous_transform>
  mate_planes(const plane a, const plane b, const plane c,
	      const plane ap, const plane bp, const plane cp) {

    const ublas::matrix<double> rotation =
      plane_basis_rotation(a.normal(), b.normal(), c.normal(),
			   -1*ap.normal(), -1*bp.normal(), -1*cp.normal());

    if (!within_eps(determinant(rotation), 1.0, 0.001)) {
      return boost::none;
    }

    const ublas::vector<double> displacement =
      plane_basis_displacement(rotation,
			       ap.normal(), bp.normal(), cp.normal(),
			       ap.pt(), bp.pt(), cp.pt(),
			       a.pt(), b.pt(), c.pt());

    std::pair<const ublas::matrix<double>,
	      const ublas::vector<double> > p =
      std::make_pair(rotation, displacement);
    return p;
  }
}
