#ifndef GCA_REGION_H
#define GCA_REGION_H

#include <utility>

#include "simulators/mill_tool.h"

using namespace std;

namespace gca {

  class region {
  public:
    double resolution;
    double height;
    
  region(double x_w, double y_w, double z_w, double xy_resolution) :
      resolution(xy_resolution), height(z_w) {}

    double volume_removed() {
      return 0.0;
    }

    void update(point p, const mill_tool& t) {
      vector<column> to_update;
      t.columns_to_update(p, resolution, to_update);
    }
  };

}
#endif
