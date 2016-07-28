#ifndef GCA_MILLABILITY_H
#define GCA_MILLABILITY_H

#include "geometry/surface.h"
#include "geometry/triangular_mesh.h"

namespace gca {

  std::vector<index_t> millable_faces(const point normal,
				      const triangular_mesh& part);

  std::vector<index_t> side_millable_faces(const point normal,
					   const std::vector<index_t>& all_face_inds,
					   const triangular_mesh& part);

  std::vector<surface>
  surfaces_visible_from(const std::vector<surface>& surfaces_left,
			const point n);

}

#endif
