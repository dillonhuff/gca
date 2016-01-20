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
    double x_len, y_len;
    int num_x_elems, num_y_elems;
    double total_volume_removed;
    float* column_heights;
    
  region(double x_w, double y_w, double z_w, double xy_resolution) :
    resolution(xy_resolution), height(z_w), x_len(x_w), y_len(y_w),
      num_x_elems(x_w/static_cast<double>(xy_resolution)),
      num_y_elems(y_w/static_cast<double>(xy_resolution)),
      total_volume_removed(0) {
      column_heights = static_cast<float*>(malloc(sizeof(float)*num_x_elems*num_y_elems));
      for (int i = 0; i < num_x_elems; i++) {
	for (int j = 0; j < num_y_elems; j++) {
	  set_column_height(i, j, z_w);
	}
      }
    }

    ~region() {
      delete column_heights;
    }

    double volume_removed() {
      return total_volume_removed;
    }

    inline float column_height(int i, int j) {
      return *(column_heights + i*num_y_elems + j);
    }

    inline void set_column_height(int i, int j, float f) {
      *(column_heights + i*num_y_elems + j) = f;
    }

    void update(point p, const mill_tool& t) {
      for (int i = 0; i < num_x_elems; i++) {
	for (int j = 0; j < num_y_elems; j++) {
	  if (t.contains(p, resolution, i, j)) {
	    double z_diff = static_cast<double>(column_height(i, j)) - p.z;
	    if (z_diff > 0) {
	      set_column_height(i, j, p.z);
	      //cout << "New height at (" << i << ", " << j << ") = " << p.z << endl;
	      total_volume_removed += resolution*resolution*z_diff;
	    }
	  }
	}
      }
    }
  };

}
#endif
