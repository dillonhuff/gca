#ifndef GCA_JAW_CUTOUT_H
#define GCA_JAW_CUTOUT_H

#include "synthesis/clipping_plan.h"
#include "synthesis/contour_planning.h"

namespace gca {

  struct soft_jaws {
    point axis;
    triangular_mesh* notch;
    triangular_mesh* a_jaw;
    triangular_mesh* an_jaw;
  };

  soft_jaws make_soft_jaws(const contour_surface_decomposition& surfs,
			   const vice& v);
  
  boost::optional<clipping_plan>
  custom_jaw_plan(const triangular_mesh& aligned,
		  const triangular_mesh& part_mesh,
		  const contour_surface_decomposition& surfs,
		  const fixture& top_fix,
		  const fabrication_inputs& fab_inputs);

}

#endif
