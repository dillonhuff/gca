#include "geometry/triangle.h"

namespace gca {

  bool is_upward_facing(const triangle& t, double tolerance) {
    return t.normal.z > tolerance;
  }

  bool same_orientation(const triangle& x, const triangle& y, double tolerance) {
    return within_eps((x.normal - y.normal).len(), 0.0, tolerance);
  }

  vector<oriented_polygon> merge_triangles(const vector<triangle>& tris) {
    vector<oriented_polygon> ps;
    return ps;
  }
  
  ostream& operator<<(ostream& out, const triangle& t) {
    cout << "---- TRIANGLE ----" << endl;
    cout << t.normal << endl;;
    cout << t.v1 << endl;;
    cout << t.v2 << endl;;
    cout << t.v3 << endl;;
    return out;
  }

}
