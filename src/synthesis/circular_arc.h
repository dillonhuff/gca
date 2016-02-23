#ifndef GCA_CIRCULAR_ARC_H
#define GCA_CIRCULAR_ARC_H

#include "core/arena_allocator.h"
#include "geometry/direction.h"
#include "synthesis/cut.h"

namespace gca {

  class circular_arc : public cut {
  public:
    point start_offset;
    direction dir;
    plane pl;

  circular_arc(point sp, point ep, point so, direction pdir, plane ppl) :
    cut(sp, ep), start_offset(so), dir(pdir), pl(ppl) {}

    static circular_arc* make(point sp, point ep, point offset, direction dir, plane pl) {
      circular_arc* mem = allocate<circular_arc>();
      return new (mem) circular_arc(sp, ep, offset, dir, pl);
    }

    bool operator==(const cut& other) const {
      if (other.is_circular_arc()) {
	const circular_arc& ci = static_cast<const circular_arc&>(other);
	return within_eps(start, ci.start) && within_eps(end, ci.end) && within_eps(start_offset, ci.start_offset);
      }
      return false;
    }

    cut* shift(point sh) const {
      circular_arc* mem = allocate<circular_arc>();
      return new (mem) circular_arc(start + sh, end + sh, start_offset, dir, pl);
    }

    virtual point initial_orient() const {
      double theta = dir == CLOCKWISE ? 90 : -90;
      return (end - start).rotate_z(theta).normalize();
    }

    inline bool is_circular_arc() const { return true; }
    
    inline point center_to_start_vec() const { return -1 * start_offset; }
    inline point center_to_end_vec() const { return end - center(); }
    inline point center() const { return start + start_offset; }
  };

}

#endif
