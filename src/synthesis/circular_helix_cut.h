#ifndef GCA_CIRCULAR_HELIX_CUT_H
#define GCA_CIRCULAR_HELIX_CUT_H

#include "utils/arena_allocator.h"
#include "geometry/direction.h"
#include "synthesis/cut.h"

namespace gca {

  class circular_helix_cut : public cut {
  public:
    point start_offset;
    direction dir;
    plane pl;

  circular_helix_cut(point sp, point ep, point so, direction pdir, plane ppl) :
    cut(sp, ep), start_offset(so), dir(pdir), pl(ppl) {
    sanity_check();
  }

  circular_helix_cut(point sp, point ep, point so, direction pdir, plane ppl, tool_name tn) : cut(sp, ep, tn), start_offset(so), dir(pdir), pl(ppl)  {
    sanity_check();
  }

    void sanity_check() const {
      point s(get_start().x, get_start().y, 0);
      point e(get_end().x, get_end().y, 0);
      point c(center().x, center().y, 0);
      double sdiff = (c - s).len();
      double ediff =  (c - e).len();
      double tolerance = 0.0005;
      if (!(within_eps(sdiff, ediff, tolerance))) {
	cout << "Error: !(within_eps((center() - start).len(), (center() - end).len()))" << endl;
	cout << "(center() - start).len() = " << sdiff << endl;
	cout << "(center() - end).len()   = " << ediff << endl;
	cout << "Difference               = " << sdiff - ediff << endl;
	cout << "Tolerance                = " << tolerance << endl;
	cout << "center                   = " << center() << endl;
	cout << "start                    = " << get_start() << endl;
	cout << "end                      = " << get_end() << endl;
	cout << "start_offset             = " << start_offset << endl;
	assert(false);
      }
    }
    
    static circular_helix_cut* make(point sp, point ep, point offset, direction dir, plane pl) {
      circular_helix_cut* mem = allocate<circular_helix_cut>();
      return new (mem) circular_helix_cut(sp, ep, offset, dir, pl);
    }

    static circular_helix_cut* make(point sp, point ep, point offset, direction dir, plane pl, tool_name tn) {
      circular_helix_cut* mem = allocate<circular_helix_cut>();
      return new (mem) circular_helix_cut(sp, ep, offset, dir, pl, tn);
    }
    
    bool operator==(const cut& other) const {
      if (!same_cut_properties(*this, other)) {
	return false;
      }
      if (other.is_circular_helix_cut()) {
	const circular_helix_cut& ci = static_cast<const circular_helix_cut&>(other);
	return pl == ci.pl && dir == ci.dir && within_eps(get_start(), ci.get_start()) && within_eps(get_end(), ci.get_end()) && within_eps(start_offset, ci.start_offset);
      }
      return false;
    }

    cut* shift(point sh) const {
      //circular_helix_cut* mem = allocate<circular_helix_cut>();
      circular_helix_cut* arc = static_cast<circular_helix_cut*>(copy()); // new (mem) circular_helix_cut(start + sh, end + sh, start_offset, dir, pl);
      arc->set_start(get_start() + sh);
      arc->set_end(get_end() + sh);
      return arc;
    }

    cut* scale(double s) const {
      circular_helix_cut* arc = static_cast<circular_helix_cut*>(copy());
      arc->set_start(s*get_start());
      arc->set_end(s*get_end());
      arc->start_offset = s*start_offset;
      arc->tool_no = tool_no;
      return arc;
    }

    cut* scale_xy(double s) const {
      circular_helix_cut* m = static_cast<circular_helix_cut*>(copy());
      m->set_start(point(s*get_start().x, s*get_start().y, get_start().z));
      m->set_end(point(s*get_end().x, s*get_end().y, get_end().z));
      m->start_offset = point(s*start_offset.x, s*start_offset.y, start_offset.z);
      return m;
    }
    
    virtual point initial_orient() const {
      double theta = dir == CLOCKWISE ? 90 : -90;
      return (get_end() - get_start()).rotate_z(theta).normalize();
    }

    virtual point final_orient() const {
      return initial_orient().rotate_z(180);
    }
    
    inline bool is_circular_helix_cut() const { return true; }
    
    inline point center_to_start_vec() const { return -1 * start_offset; }
    inline point center_to_end_vec() const { return get_end() - center(); }
    inline point center() const { return get_start() + start_offset; }

    virtual cut* copy() const {
      circular_helix_cut* arc = circular_helix_cut::make(get_start(), get_end(), start_offset, dir, pl);
      arc->tool_no = tool_no;
      arc->set_feedrate(settings.feedrate);
      arc->set_spindle_speed(settings.spindle_speed);
      return arc;
    }

    void print(ostream& other) const {
      other << "CIRCULAR ARC: " << tool_no << " ";
      if (!get_feedrate()->is_omitted()) {
	other << "F" << *get_feedrate() << " ";
      } else {
	other << "<F omitted> ";
      }
      if (!get_spindle_speed()->is_omitted()) {
	other << "S" << *get_spindle_speed() << " ";
      } else {
	other << "<S omitted> ";
      }
      other << " offset: " << start_offset;
      other << " dir: " << dir;
    }

  };

}

#endif
