#ifndef GCA_LINEAR_CUT_H
#define GCA_LINEAR_CUT_H

#include "geometry/point.h"
#include "synthesis/cut.h"

namespace gca {

  class linear_cut : public cut {
  public:
  linear_cut(point sp, point ep) :
    cut(sp, ep) {}

    bool operator==(const cut& other) const {
      if (other.is_linear_cut()) {
	bool res = within_eps(start, other.start) && within_eps(end, other.end);
	return res;
      }
      return false;
    }

    inline bool is_linear_cut() const { return true; }
    
  };

}

#endif
