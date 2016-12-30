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
  
}
