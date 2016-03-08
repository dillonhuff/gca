#ifndef GCA_HOLE_PUNCH_H
#define GCA_HOLE_PUNCH_H

#include "core/arena_allocator.h"
#include "synthesis/cut.h"

namespace gca {

  class hole_punch : public cut {
  public:
    double radius;
  hole_punch(point center, double rp) : cut(center, center), radius(rp) {}
  hole_punch(point center, double rp, tool_name t) : cut(center, center, t), radius(rp) {}

    static hole_punch* make(point center, double rad) {
      hole_punch* mem = allocate<hole_punch>();
      hole_punch* hole = new (mem) hole_punch(center, rad);
      return hole;
    }

    static hole_punch* make(point center, double rad, tool_name t) {
      hole_punch* mem = allocate<hole_punch>();
      return new (mem) hole_punch(center, rad, t);
    }
    
    inline bool is_hole_punch() const { return true; }

    cut* shift(point sh) const {
      hole_punch* mem = allocate<hole_punch>();
      hole_punch* hole = new (mem) hole_punch(start + sh, radius);
      hole->tool_no = tool_no;
      return hole;
    }

    cut* scale(double s) const {
      hole_punch* mem = allocate<hole_punch>();
      hole_punch* hole = new (mem) hole_punch(s*start, s*radius);
      hole->tool_no = tool_no;
      return hole;
    }

    cut* scale_xy(double s) const {
      hole_punch* m = static_cast<hole_punch*>(copy());
      m->start = point(s*start.x, s*start.y, start.z);
      m->end = point(s*end.x, s*end.y, end.z);
      m->radius = s*radius;
      return m;
    }
    
    bool operator==(const cut& other) const {
      if (!same_cut_properties(*this, other)) {
	return false;
      }
      if (other.is_hole_punch()) {
	const hole_punch& other_hp = static_cast<const hole_punch&>(other);
	return other_hp.start == start && other_hp.radius == radius;
      }
      return false;
    }

    virtual cut* copy() const {
      hole_punch* h = hole_punch::make(start, radius);
      h->tool_no = tool_no;
      h->feedrate = feedrate;
      return h;
    }

    void print(ostream& other) const {
      other << "HOLE PUNCH: " << tool_no << " ";
      if (!feedrate->is_omitted()) {
	other << "F" << *feedrate << " ";
      } else {
	other << "<F omitted> ";
      }
      if (!spindle_speed->is_omitted()) {
	other << "S" << *spindle_speed << " ";
      } else {
	other << "<S omitted> ";
      }
      other << start << " RADIUS: " << radius;
    }

  };
}

#endif
