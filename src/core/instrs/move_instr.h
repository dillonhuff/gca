#ifndef GCA_MOVE_INSTR_H
#define GCA_MOVE_INSTR_H

#include "core/arena_allocator.h"
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
      x = i->x;
      y = i->y;
      z = i->z;
      feed_rate = i->feed_rate;
    }

    move_instr(value* xp, value* yp, value* zp, value* frp) {
      assert(frp > 0);
      x = xp;
      y = yp;
      z = zp;
      feed_rate = frp;
    }

    virtual void print(ostream& s) const { assert(false); }
    virtual bool operator==(const instr& other) const { assert(false); }
    
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

    void print_move_data(ostream& s, double eps) const {
      if (!feed_rate->is_omitted()) { s << 'F' << *feed_rate << ' '; }
      if (!x->is_omitted()) {
	s << 'X';
	x->print_eps(s, eps);
	s << ' ';
      }
      if (!y->is_omitted()) {
	s << 'Y';
	y->print_eps(s, eps);
	s << ' ';
      }
      if (!z->is_omitted()) {
	s << 'Z';
	z->print_eps(s, eps);
	s << ' ';
      }
    }

    bool same_pos_and_feed_rate(const move_instr& other_move) const {
      bool pr = *x == *(other_move.x) && *y == *(other_move.y) && *z == *(other_move.z);
      bool fr = *feed_rate == *(other_move.feed_rate);
      return fr && pr;
    }
    
  };

  class g0_instr : public move_instr {
  public:
    g0_instr(value* xp, value* yp, value* zp, value* frp) : move_instr(xp, yp, zp, frp) {}
    g0_instr(g0_instr* i) : move_instr(i) {}

    static g0_instr* make(point p) {
      g0_instr* mem = allocate<g0_instr>();
      return new (mem) g0_instr(lit::make(p.x), lit::make(p.y), lit::make(p.z), omitted::make());
    }

    static g0_instr* make(double x, double y, double z) {
      g0_instr* mem = allocate<g0_instr>();
      return new (mem) g0_instr(lit::make(x), lit::make(y), lit::make(z), omitted::make());
    }

    static g0_instr* make(value* x, value* y, value* z) {
      g0_instr* mem = allocate<g0_instr>();
      return new (mem) g0_instr(x, y, z, omitted::make());
    }
        
    virtual inline bool is_G0() const { return true; }

    void print(ostream& s) const {
      cout << "G0 ";
      print_move_data(s, 0.000001);
    }

    virtual instr* copy() const {
      return new (allocate<g0_instr>()) g0_instr(x, y, z, feed_rate);
    }
    
    bool operator==(const instr& other) const {
      return other.is_G0() &&
	same_pos_and_feed_rate(static_cast<const move_instr&>(other));
    }
  };

  class g1_instr : public move_instr {
  public:
    g1_instr(value* xp, value* yp, value* zp, value* frp) : move_instr(xp, yp, zp, frp) {}
    g1_instr(g1_instr* i) : move_instr(i) {}

    static g1_instr* make(double x, double y, double z) {
      g1_instr* mem = allocate<g1_instr>();
      return new (mem) g1_instr(lit::make(x), lit::make(y), lit::make(z), lit::make(1.0));
    }
    
    static g1_instr* make(double x, double y, double z, double feed_rate) {
      g1_instr* mem = allocate<g1_instr>();
      lit* v = allocate<lit>();
      lit* frp = new (v) lit(feed_rate);
      return new (mem) g1_instr(lit::make(x), lit::make(y), lit::make(z), frp);
    }
  
    static g1_instr* make(value* x, value* y, value* z, value* feed_rate) {
      g1_instr* mem = allocate<g1_instr>();
      return new (mem) g1_instr(x, y, z, feed_rate);
    }
    
    static g1_instr* make(double x, double y, double z, value* feed_rate) {
      g1_instr* mem = allocate<g1_instr>();
      return new (mem) g1_instr(lit::make(x), lit::make(y), lit::make(z), feed_rate);
    }
    
    virtual inline bool is_G1() const { return true; }
    void print(ostream& s) const {
      cout << "G1 ";
      print_move_data(s, 0.000001);
    }
    
    virtual instr* copy() const {
      return new (allocate<g1_instr>()) g1_instr(x, y, z, feed_rate);
    }
    
    bool operator==(const instr& other) const {
      return other.is_G1() &&
	same_pos_and_feed_rate(static_cast<const move_instr&>(other));
    }
    
  };

  class g53_instr : public move_instr {
  public:
    g53_instr(value* xp, value* yp, value* zp, value* frp) : move_instr(xp, yp, zp, frp) {}
    g53_instr(g53_instr* i) : move_instr(i) {}

    static g53_instr* make(value* x, value* y, value* z) {
      g53_instr* mem = allocate<g53_instr>();
      return new (mem) g53_instr(x, y, z, omitted::make());
    }

    
    virtual inline bool is_G53() const { return true; }
    
    void print(ostream& s) const {
      cout << "G53 ";
      print_move_data(s, 0.000001);
    }

    bool operator==(const instr& other) const {
      return other.is_G53() &&
	same_pos_and_feed_rate(static_cast<const move_instr&>(other));
    }
    
  };

  class g28_instr : public move_instr {
  public:
    g28_instr(value* xp, value* yp, value* zp, value* frp) : move_instr(xp, yp, zp, frp) {}
    g28_instr(g28_instr* i) : move_instr(i) {}

    static g28_instr* make(value* x, value* y, value* z) {
      g28_instr* mem = allocate<g28_instr>();
      return new (mem) g28_instr(x, y, z, omitted::make());
    }

    
    virtual inline bool is_G28() const { return true; }
    
    void print(ostream& s) const {
      cout << "G28 ";
      print_move_data(s, 0.000001);
    }

    bool operator==(const instr& other) const {
      return other.is_G28() &&
	same_pos_and_feed_rate(static_cast<const move_instr&>(other));
    }
    
  };

  class g83_instr : public move_instr {
  public:
    bool return_to_r;
    value* r;
    value* q;
    g83_instr(bool return_to_rp,
	      value* xp, value* yp, value* zp,
	      value* rp,
	      value* qp,
	      value* frp) : move_instr(xp, yp, zp, frp),
			    return_to_r(return_to_rp), r(rp), q(qp) {}
    g83_instr(g83_instr* i) : move_instr(i) {}

    static g83_instr* make(bool return_to_rp,
			   value* xp, value* yp, value* zp,
			   value* rp,
			   value* qp,
			   value* frp) {
      g83_instr* mem = allocate<g83_instr>();
      return new (mem) g83_instr(return_to_rp,
				 xp, yp, zp,
				 rp,
				 qp,
				 frp);
    }

    
    virtual inline bool is_G83() const { return true; }
    
    void print(ostream& s) const {
      cout << "G83 ";
      print_move_data(s, 0.000001);
    }

    bool operator==(const instr& other) const {
      if (!other.is_G83()) {
	return false;
      }
      const g83_instr& ci = static_cast<const g83_instr&>(other);
      return same_pos_and_feed_rate(ci) &&
	*(ci.r) == *(this->r) &&
	*(ci.q) == *(this->q);
    }
    
  };
  
}

#endif
