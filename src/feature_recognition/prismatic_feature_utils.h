#pragma once

#include "geometry/triangular_mesh.h"
#include "feature_recognition/prismatic_feature.h"

namespace gca {

  boost::optional<feature>
  extract_feature(const feature& original,
		  const triangular_mesh& portion);

  feature
  extract_extrusion_feature(const point n,
			    const triangular_mesh& m);

  double volume(const feature& f);

  triangular_mesh feature_mesh(const feature& f,
			       const double base_dilation,
			       const double top_extension,
			       const double base_extension);

  triangular_mesh feature_mesh(const feature& f);  

  int curve_count(const feature& f);

}
