#ifndef GCA_MOVE_INSTR_H
#define GCA_MOVE_INSTR_H

#include "core/instr.h"

namespace gca {

  class move_instr : public instr {
  protected:
    point position;
    orientation orient;

  public:
    double feed_rate;

    move_instr(move_instr* i) {
      c = i->c;
      v = i->v;
      position = i->position;
      orient = i->orient;
      feed_rate = i->feed_rate;
    }

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

    inline bool is_move_instr() const { return true; }
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

    virtual void print(ostream& s) const {
      assert(is_G());
      s << 'G' << v << ' ';
      s << 'F' << feed_rate << ' ';
      s << 'X' << position.x << ' ';
      s << 'Y' << position.y << ' ';
      s << 'Z' << position.z << ' ';
    }

    bool operator==(const instr& other) {
      if (!other.is_move_instr()) {
	return false;
      }
      const move_instr& other_move = static_cast<const move_instr&>(other);
      if (c != other_move.c || v != other_move.v) {
	return false;
      }
      if (c == GCA_G && (v == 0 || v == 1)) {
        point lp = pos();
        point op = other_move.pos();
        bool res = within_eps(lp, op) && feed_rate == other_move.feed_rate && orient == other_move.orient;
        if (!(orient == other_move.orient)) {
	  cout << "!ORIENT " << *this << " != " << other_move << endl;
        }
        if (!(feed_rate == other_move.feed_rate)) {
	  cout << "!FEED RATE " << *this << " != " << other_move << endl;
        }
        if (!within_eps(lp, op)) {
	  cout << "!within EPS " << *this << " != " << other_move << endl;
        }
        if (!res) {
	  cout << *this << " != " << other_move << endl;
        }
        return res;
      }
      return true;
    }
    
  };
}

#endif
