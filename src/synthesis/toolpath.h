#ifndef GCA_TOOLPATH_H
#define GCA_TOOLPATH_H

#include <vector>

#include "analysis/machine_state.h"
#include "geometry/parametric_curve.h"
#include "synthesis/cut.h"

using namespace std;

namespace gca {
  
  typedef vector<cut*> cut_group;

  struct toolpath {
    machine_settings s;
    parametric_curve c;
    toolpath(machine_settings sp, parametric_curve cp) : s(sp), c(cp) {}

    inline point start() const { return c.value(0); }
    inline point end() const { return c.value(1.0); }
  };

  bool operator==(const toolpath& x, const toolpath& y);
  bool operator!=(const toolpath& x, const toolpath& y);

  vector<toolpath> cuts_to_toolpaths(const vector<cut*> cuts);
}

#endif
