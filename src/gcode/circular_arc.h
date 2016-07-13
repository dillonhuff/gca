#ifndef GCA_CIRCULAR_ARC_H
#define GCA_CIRCULAR_ARC_H

#include "utils/arena_allocator.h"
#include "geometry/arc.h"
#include "geometry/direction.h"
#include "gcode/cut.h"

namespace gca {

  class circular_arc : public cut {
  public:
    point start_offset;
    direction dir;
    work_plane pl;

    circular_arc(point sp, point ep, point so, direction pdir, work_plane ppl) :
      cut(sp, ep), start_offset(so), dir(pdir), pl(ppl) {
      sanity_check();
      c = arc(sp, ep, so, pdir);
    }

    circular_arc(point sp, point ep, point so, direction pdir, work_plane ppl, tool_name tn) : cut(sp, ep, tn), start_offset(so), dir(pdir), pl(ppl)  {
      sanity_check();
      c = arc(sp, ep, so, pdir);
    }

    void sanity_check() {
      double sdiff = (center() - get_start()).len();
      double ediff =  (center() - get_end()).len();
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
    
    static circular_arc* make(point sp, point ep, point offset, direction dir, work_plane pl) {
      circular_arc* mem = allocate<circular_arc>();
      return new (mem) circular_arc(sp, ep, offset, dir, pl);
    }

    static circular_arc* make(point sp, point ep, point offset, direction dir, work_plane pl, tool_name tn) {
      circular_arc* mem = allocate<circular_arc>();
      return new (mem) circular_arc(sp, ep, offset, dir, pl, tn);
    }
    
    bool operator==(const cut& other) const {
      if (!same_cut_properties(*this, other)) {
	return false;
      }
      if (other.is_circular_arc()) {
	const circular_arc& ci = static_cast<const circular_arc&>(other);
	return pl == ci.pl && dir == ci.dir && within_eps(get_start(), ci.get_start()) && within_eps(get_end(), ci.get_end()) && within_eps(start_offset, ci.start_offset);
      }
      return false;
    }

    virtual point initial_orient() const {
      double theta = dir == CLOCKWISE ? 90 : -90;
      return (get_end() - get_start()).rotate_z(theta).normalize();
    }

    virtual point final_orient() const {
      return initial_orient().rotate_z(180);
    }
    
    inline bool is_circular_arc() const { return true; }
    
    inline point center_to_start_vec() const { return -1 * start_offset; }
    inline point center_to_end_vec() const { return get_end() - center(); }
    inline point center() const { return get_start() + start_offset; }

    virtual cut* copy() const {
      circular_arc* arc = circular_arc::make(get_start(), get_end(), start_offset, dir, pl);
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
      other << get_start() << " -> " << get_end();
      other << " offset: " << start_offset;
      other << " dir: " << dir;
    }

  };

}

#endif
