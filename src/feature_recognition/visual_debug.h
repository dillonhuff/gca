#ifndef GCA_FEATURE_RECOGNITION_VISUAL_DEBUG_H
#define GCA_FEATURE_RECOGNITION_VISUAL_DEBUG_H

#include "feature_recognition/feature_decomposition.h"
#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"

namespace gca {

  void vtk_debug_polygon(const labeled_polygon_3& p);

  void vtk_debug_feature_decomposition(feature_decomposition* f);
}

#endif

