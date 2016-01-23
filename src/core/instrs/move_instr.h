#ifndef GCA_MOVE_INSTR_H
#define GCA_MOVE_INSTR_H

#include "core/instrs/instr.h"
#include "core/value.h"

namespace gca {

  class move_instr : public instr {
  protected:
    value* x;
    value* y;
    value* z;

  public:
    value* feed_rate;

    move_instr(move_instr* i) {
      c = i->c;
      v = i->v;
      x = i->x;
      y = i->y;
      z = i->z;
      feed_rate = i->feed_rate;
    }

    move_instr(instr_class cp, instr_val vp, value* xp, value* yp, value* zp, value* frp) {
      assert(cp == GCA_G);
      assert(frp > 0);
      c = cp;
      v = vp;
      x = xp;
      y = yp;
      z = zp;
      feed_rate = frp;
    }
    
    inline point pos() const {
      assert(is_G());
      assert(x->is_lit() && y->is_lit() && z->is_lit());
      lit* x_lit = static_cast<lit*>(x);
      lit* y_lit = static_cast<lit*>(y);
      lit* z_lit = static_cast<lit*>(z);
      return point(x_lit->v, y_lit->v, z_lit->v);
    }

    double x_with_default(double default_value) {
      assert(!x->is_var());
      if (x->is_lit()) {
	lit* x_lit = static_cast<lit*>(x);
	return x_lit->v;
      }
      return default_value;
    }

    double y_with_default(double default_value) {
      assert(!y->is_var());
      if (y->is_lit()) {
	lit* y_lit = static_cast<lit*>(y);
	return y_lit->v;
      }
      return default_value;
    }

    double z_with_default(double default_value) {
      assert(!z->is_var());
      if (z->is_lit()) {
	lit* z_lit = static_cast<lit*>(z);
	return z_lit->v;
      }
      return default_value;
    }
    
    inline bool is_move_instr() const { return true; }

    inline void swap_orientation() {
      assert(is_G1());
    }

    virtual void print(ostream& s) const {
      assert(is_G());
      s << 'G' << v << ' ';
      if (!feed_rate->is_omitted()) { s << 'F' << *feed_rate << ' '; }
      if (!x->is_omitted()) { s << 'X' << *x << ' '; }
      if (!y->is_omitted()) { s << 'Y' << *y << ' '; }
      if (!z->is_omitted()) { s << 'Z' << *z << ' '; }
    }

    bool operator==(const instr& other) const {
      if (!other.is_move_instr()) {
	return false;
      }
      const move_instr& other_move = static_cast<const move_instr&>(other);
      if (c != other_move.c || v != other_move.v) {
	return false;
      }
      if (c == GCA_G && (v == 0 || v == 1)) {
	bool pr = *x == *(other_move.x) && *y == *(other_move.y) && *z == *(other_move.z);
	bool fr = *feed_rate == *(other_move.feed_rate);
        return fr && pr;
      }
      return true;
    }
    
  };
}

#endif
