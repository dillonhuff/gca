#ifndef GCA_MOVE_INSTR_H
#define GCA_MOVE_INSTR_H

#include "core/instr.h"

namespace gca {

  class move_instr : public instr {
  protected:
    point position;
    orientation orient;

  public:
    double feedrate;

    move_instr(instr_class cp, instr_val vp, point p, orientation orientp=GCA_ABSOLUTE) {
      assert(cp == GCA_G);
      c = cp;
      v = vp;
      position = p;
      feed_rate = -1.0;
      orient = orientp;
    }
    
    move_instr(instr_class cp, instr_val vp, point p, double frp, orientation orientp=GCA_ABSOLUTE) {
      assert(cp == GCA_G);
      assert(frp > 0);
      c = cp;
      v = vp;
      position = p;
      feed_rate = frp;
      orient = orientp;
    }

    inline point pos() const {
      assert(is_G());
      return position;
    }

    inline bool is_abs() const {
      assert(is_G());
      return orient == GCA_ABSOLUTE;
    }

    inline bool is_rel() const {
      assert(is_G1() || is_G0());
      return orient == GCA_RELATIVE;
    }

    inline void swap_orientation() {
      assert(is_G1());
      orient = orient == GCA_ABSOLUTE ? GCA_RELATIVE : GCA_ABSOLUTE;
    }
    
  };
}

#endif
