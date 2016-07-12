#ifndef GCA_FACE_CLIPPING_H
#define GCA_FACE_CLIPPING_H

#include "synthesis/fixture_analysis.h"
#include "synthesis/tool.h"
#include "synthesis/vice.h"
#include "synthesis/workpiece_clipping.h"

namespace gca {

  workpiece clipped_workpiece(const workpiece aligned_workpiece,
			      const triangular_mesh& part_mesh);
  
  std::pair<triangular_mesh, std::vector<fixture_setup> >
  axis_by_axis_clipping(const workpiece w, 
			const triangular_mesh& part_mesh,
			std::vector<surface>& surfaces_to_cut,
			const std::vector<tool>& tools,
			const fixtures& f);
  
}

#endif
