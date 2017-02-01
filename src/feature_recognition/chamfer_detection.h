#pragma once

#include "geometry/triangular_mesh.h"
#include "backend/tool.h"

namespace gca {

  struct chamfer {
    std::vector<index_t> faces;
    tool t;
  };

  std::vector<chamfer>
  chamfer_regions(const triangular_mesh& mesh,
		  const point n,
		  const std::vector<tool>& chamfer_angles);

  std::vector<index_t>
  chamfer_faces(const triangular_mesh& mesh,
		const point n,
		const std::vector<tool>& chamfer_angles);
  
}
