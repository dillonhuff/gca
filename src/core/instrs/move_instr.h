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

    move_instr(value* xp, value* yp, value* zp, value* frp) {
      x = xp;
      y = yp;
      z = zp;
      feed_rate = frp;
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

    inline value* get_x() const { return x; }
    inline value* get_y() const { return y; }
    inline value* get_z() const { return z; }

    inline void set_x(value* n) { x = n; }
    inline void set_y(value* n) { y = n; }
    inline void set_z(value* n) { z = n; }
    
    inline bool is_concrete() const {
      return x->is_lit() && y->is_lit() && z->is_lit();
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

    virtual bool operator==(const instr& other) const {
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

  class g0_instr : public move_instr {
  public:
  g0_instr(instr_class cp, instr_val vp, value* xp, value* yp, value* zp, value* frp) : move_instr(cp, vp, xp, yp, zp, frp) {}
  g0_instr(g0_instr* i) : move_instr(i) {}

    virtual inline bool is_G0() const { return true; }
  };

  class g1_instr : public move_instr {
  public:
  g1_instr(instr_class cp, instr_val vp, value* xp, value* yp, value* zp, value* frp) : move_instr(cp, vp, xp, yp, zp, frp) {}
  g1_instr(g1_instr* i) : move_instr(i) {}
    
    virtual inline bool is_G1() const { return true; }
  };

  class g53_instr : public move_instr {
  public:
  g53_instr(instr_class cp, instr_val vp, value* xp, value* yp, value* zp, value* frp) : move_instr(cp, vp, xp, yp, zp, frp) {}
  g53_instr(g53_instr* i) : move_instr(i) {}
    virtual inline bool is_G53() const { return true; }
  };
  
}

#endif
