#ifndef GCA_WORKPIECE_CLIPPING_H
#define GCA_WORKPIECE_CLIPPING_H

#include "synthesis/clipping_plan.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/workpiece.h"

namespace gca {

  void add_surface_pockets(vector<gca::pocket>& pockets,
			   const triangular_mesh& m,
			   const vector<gca::surface>& surfs);

  void add_surface_pockets(vector<gca::pocket>& pockets,
			   const triangular_mesh& m,
			   const std::vector<std::vector<index_t>>& surfs);
  
  // TODO: Move to workpiece?
  triangular_mesh stock_mesh(const workpiece& w);
  
  clipping_plan
  workpiece_clipping_programs(const std::vector<workpiece>& workpiece,
			      const triangular_mesh& part_mesh,
			      const std::vector<tool>& tools,
			      const fixtures& fixes);

  pocket face_down(const triangular_mesh& stock,
		   const triangular_mesh& part,
		   const triangular_mesh& out);
  
  fixture_setup
  clip_base(const triangular_mesh& aligned,
	    const triangular_mesh& part,
	    const fixture& f);

  pocket face_down(const triangular_mesh& stock,
		   const triangular_mesh& part);

  pocket contour_around(const triangular_mesh& stock,
			const triangular_mesh& part);

  std::vector<pocket>
  make_pockets(const triangular_mesh& part,
	       const triangular_mesh& stock,
	       const std::vector<surface>& surfaces);



}

#endif
