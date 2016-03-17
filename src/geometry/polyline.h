#ifndef GCA_POLYLINE_H
#define GCA_POLYLINE_H

#include <cassert>
#include <vector>

#include "geometry/line.h"
#include "geometry/point.h"

using namespace std;

namespace gca {

  class polyline {
  protected:
    vector<point> points;

  public:

    polyline(const vector<point>& pointsp) : points(pointsp) {}

    vector<point>::iterator begin() { return points.begin(); }
    vector<point>::iterator end() { return points.end(); }
    vector<point>::const_iterator begin() const { return points.begin(); }
    vector<point>::const_iterator end() const { return points.end(); }

    vector<line> lines() const {
      assert(points.size() > 1);
      vector<line> ls;
      for (vector<point>::const_iterator it = points.begin();
	   it != points.end() - 1; ++it) {
	ls.push_back(line(*it, *(it + 1)));
      }
      return ls;
    }
      
  };
}

#endif
