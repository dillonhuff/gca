#ifndef GCA_LINEAR_CUT_H
#define GCA_LINEAR_CUT_H

#include "core/arena_allocator.h"
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

    cut* shift(point sh) const {
      linear_cut* mem = allocate<linear_cut>();
      return new (mem) linear_cut(start + sh, end + sh);
    }

    inline bool is_linear_cut() const { return true; }
    
  };

}

#endif
