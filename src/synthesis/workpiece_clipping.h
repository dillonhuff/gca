#ifndef GCA_WORKPIECE_CLIPPING_H
#define GCA_WORKPIECE_CLIPPING_H

#include "synthesis/clipping_plan.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/workpiece.h"

namespace gca {

  void add_surface_pockets(vector<gca::pocket>& pockets,
			   const rigid_arrangement& a,
			   const vector<gca::surface>& surfs);

  // TODO: Move to workpiece?
  triangular_mesh stock_mesh(const workpiece& w);
  
  clipping_plan
  workpiece_clipping_programs(const workpiece aligned_workpiece,
			      const triangular_mesh& part_mesh,
			      const std::vector<tool>& tools,
			      const fixtures& fixes);


  fixture_setup
  clip_top_and_sides_transform(const triangular_mesh& wp_mesh,
			       const triangular_mesh& part_mesh,
			       const std::vector<surface>& surfaces,
			       const fixture& f);

  fixture_setup
  clip_base_transform(const triangular_mesh& aligned,
		      const triangular_mesh& part,
		      const std::vector<surface>& surfaces,
		      const fixture& f);

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
			   const std::vector<surface>& surfaces);



}

#endif
