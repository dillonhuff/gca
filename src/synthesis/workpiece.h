#ifndef GCA_WORKPIECE_H
#define GCA_WORKPIECE_H

#include "geometry/point.h"
#include "backend/material.h"

namespace gca {

  struct workpiece {
    point sides[3];
    material stock_material;

    workpiece(double x, double y, double z, material p_stock_material) {
      sides[0] = point(x, 0, 0);
      sides[1] = point(0, y, 0);
      sides[2] = point(0, 0, z);
      stock_material = p_stock_material;
    }

    workpiece(point x, point y, point z, material p_stock_material) {
      sides[0] = x;
      sides[1] = y;
      sides[2] = z;
      stock_material = p_stock_material;
    }

  };

}

#endif
