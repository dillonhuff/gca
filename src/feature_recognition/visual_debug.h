#ifndef GCA_FEATURE_RECOGNITION_VISUAL_DEBUG_H
#define GCA_FEATURE_RECOGNITION_VISUAL_DEBUG_H

#include "feature_recognition/feature_decomposition.h"
#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"

namespace gca {

  void vtk_debug_feature_decomposition(feature_decomposition* f);
  void vtk_debug_feature_tree(feature_decomposition* f);

  void vtk_debug_feature(const feature& f);

  void vtk_debug_features(const std::vector<feature*>& f);

  void vtk_debug_features(const std::vector<feature*>& fs);
  
}

#endif

