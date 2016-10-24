#ifndef GCA_MACHINE_H
#define GCA_MACHINE_H

#include <iostream>

using namespace std;

namespace gca {

  enum machine_name {
    CAMASTER = 0,
    PROBOTIX_V90_MK2_VFD,
    EMCO_F1,
    WELLS
  };

  enum tool_name {
    NO_TOOL = 0,
    DRAG_KNIFE,
    DRILL
  };

  ostream& operator<<(ostream& out, const tool_name t);
  ostream& operator<<(ostream& out, const machine_name t);
}

#endif
