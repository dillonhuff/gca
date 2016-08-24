#ifndef GCA_FEATURE_TO_POCKET_H
#define GCA_FEATURE_TO_POCKET_H

#include "feature_recognition/feature_decomposition.h"
#include "synthesis/toolpath_generation.h"

namespace gca {

  std::vector<pocket>
  feature_pockets(feature_decomposition& r);

  std::vector<pocket>
  feature_pockets(feature_decomposition& r, const point n);

}

#endif
