#ifndef GCA_JAW_CUTOUT_H
#define GCA_JAW_CUTOUT_H

#include "geometry/rigid_arrangement.h"
#include "synthesis/clipping_plan.h"
#include "synthesis/contour_planning.h"

namespace gca {

  typedef rigid_arrangement soft_jaws;

  soft_jaws make_soft_jaws(const contour_surface_decomposition& surfs,
			   const point axis,
			   const vice& v);
  
  boost::optional<clipping_plan>
  custom_jaw_plan(const workpiece& w,
		  const triangular_mesh& aligned,
		  const triangular_mesh& part_mesh,
		  const contour_surface_decomposition& surfs,
		  const fixture& top_fix,
		  const fabrication_inputs& fab_inputs);

}

#endif
