#include <numeric>

#include "geometry/arc.h"
#include "geometry/line.h"
#include "geometry/point.h"
#include "synthesis/circular_arc.h"
#include "synthesis/toolpath.h"
#include "system/algorithm.h"

namespace gca {

  parametric_curve mk_parametric_curve(const cut& c) {
    if (c.is_linear_cut()) {
      return parametric_curve(line(c.get_start(), c.end));
    } else if (c.is_circular_arc()) {
      const circular_arc& ca = static_cast<const circular_arc&>(c);
      return parametric_curve(arc(ca.get_start(), ca.end, ca.start_offset, ca.dir));
    } else {
      assert(false);
    }
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
