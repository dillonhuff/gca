#ifndef GCA_REGION_H
#define GCA_REGION_H

namespace gca {

  class region {
  public:
    region(double x_w, double y_w, double height, double xy_resolution) {}

    double volume_removed() {
      return 0.0;
    }
  };

}
#endif
