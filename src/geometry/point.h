#ifndef GCA_POINT_H
#define GCA_POINT_H

#include <iostream>
#include <vector>

#include "geometry/point.h"

using namespace std;

namespace gca {

  class point {
  public:
    double x, y, z;

    point() {
      x = 0;
      y = 0;
      z = 0;
    }

  point(double xp, double yp, double zp) :
    x(xp), y(yp), z(zp) {}

    bool operator==(const point& other) const {
      return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const point& other) const {
      return !(*this == other);
    }
    
    point operator+(const point& other) const {
      return point(x + other.x, y + other.y, z + other.z);
    }

    point operator-(const point& other) const {
      return point(x - other.x, y - other.y, z - other.z);
    }

    double len() const;

    point normalize() const;

    point rotate_z(double degrees) const;

    void print(ostream& s) const;

    double dot(point v) const {
      return x*v.x + y*v.y + z*v.z;
    }

  };

  point operator*(double a, const point& other);
  bool within_eps(const point& l, const point& r, double eps=0.0000001);
  bool within_eps(double l, double r, double eps=0.0000001);
  point extend_back(point start, point end, double l);
  double angle_between(point u, point v);
  ostream& operator<<(ostream& s, const point& p);
  ostream& operator<<(ostream& s, const vector<point>& p);
  
}

#endif
