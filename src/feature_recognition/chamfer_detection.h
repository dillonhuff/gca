#pragma once

#include "geometry/triangular_mesh.h"

namespace gca {

  std::vector<std::vector<index_t> >
  chamfer_regions(const triangular_mesh& mesh,
		  const point n,
		  const std::vector<double>& chamfer_angles);

  std::vector<index_t>
  chamfer_faces(const triangular_mesh& mesh,
		const point n,
		const std::vector<double>& chamfer_angles);
  
}
