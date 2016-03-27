#include <numeric>

#include "geometry/line.h"
#include "geometry/point.h"
#include "synthesis/toolpath.h"
#include "system/algorithm.h"

namespace gca {

  bool operator==(const toolpath& x, const toolpath& y)
  { return true; }
  
  bool operator!=(const toolpath& x, const toolpath& y)
  { return !(x == y); }

  parametric_curve mk_parametric_curve(const cut& c) {
    return parametric_curve(line(c.start, c.end));
  }

  vector<toolpath> cuts_to_toolpaths(const vector<cut*> cuts) {
    vector<toolpath> t;
    for (auto cut : cuts) {
      machine_settings s = cut->settings;
      parametric_curve c = mk_parametric_curve(*cut);
      t.push_back(toolpath(s, c));
    }
    return t;
  }

}
