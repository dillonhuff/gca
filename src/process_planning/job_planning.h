#pragma once

#include "process_planning/direction_selection.h"

namespace gca {

  std::vector<fixture_setup> plan_jobs(const triangular_mesh& stock,
				       const triangular_mesh& part,
				       const fixtures& f,
				       const std::vector<tool>& tools);

  fixture_setup
  create_setup(const homogeneous_transform& s_t,
	       const triangular_mesh& wp_mesh,
	       const triangular_mesh& part_mesh,
	       const std::vector<feature*>& features,
	       const fixture& f,
	       const tool_access_info& tool_info,
	       const std::vector<chamfer>& chamfers,
	       const std::vector<freeform_surface>& freeforms);

  boost::optional<std::pair<fixture, homogeneous_transform> >
  find_next_fixture_side_vice(feature_decomposition* decomp,
			      Nef_polyhedron& stock_nef,
			      const triangular_mesh& current_stock,
			      const point n,
			      const fixtures& f);
  

  Nef_polyhedron
  subtract_features(const Nef_polyhedron& m,
		    const std::vector<feature*>& features);

  Nef_polyhedron
  subtract_chamfers(const Nef_polyhedron& stock_nef,
		    const std::vector<chamfer>& chamfers,
		    const triangular_mesh& part,
		    const point n);
  
  Nef_polyhedron
  subtract_freeforms(const Nef_polyhedron& stock_nef,
		     const std::vector<freeform_surface>& chamfers,
		     const triangular_mesh& part,
		     const point n);

  boost::optional<std::pair<fixture, homogeneous_transform> >
  find_next_fixture_side_vice(const double depth,
			      Nef_polyhedron& stock_nef,
			      const triangular_mesh& current_stock,
			      const point n,
			      const fixtures& f);
  
}
