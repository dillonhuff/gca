#ifndef GCA_HOLE_PUNCH_H
#define GCA_HOLE_PUNCH_H

#include "synthesis/cut.h"

namespace gca {

  class hole_punch : public cut {
  public:
    double radius;
  hole_punch(point center, double rp) : cut(center, center), radius(rp) {}

    inline bool is_hole_punch() const { return true; }

    bool operator==(const cut& other) const {
      if (other.is_hole_punch()) {
	const hole_punch& other_hp = static_cast<const hole_punch&>(other);
	return other_hp.start == start && other_hp.radius == radius;
      }
      return false;
    }
  };
}

#endif
