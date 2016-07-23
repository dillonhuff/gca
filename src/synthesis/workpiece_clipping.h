#ifndef GCA_WORKPIECE_CLIPPING_H
#define GCA_WORKPIECE_CLIPPING_H

#include "synthesis/fixture_analysis.h"
#include "synthesis/workpiece.h"

namespace gca {

  triangular_mesh stock_mesh(const workpiece& w);
  
  //  std::pair<triangular_mesh, std::vector<fixture_setup> >
  clipping_plan
  workpiece_clipping_programs(const workpiece aligned_workpiece,
			      const triangular_mesh& part_mesh,
			      std::vector<surface>& surfaces_to_cut,
			      const std::vector<tool>& tools,
			      const fixtures& fixes);


  fixture_setup
  clip_base(const triangular_mesh& aligned,
	    const triangular_mesh& part,
	    const fixture& f);
  
}

#endif
