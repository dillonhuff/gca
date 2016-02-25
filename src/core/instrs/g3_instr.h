#ifndef GCA_G3_INSTR_H
#define GCA_G3_INSTR_H

#include "core/instrs/move_instr.h"

namespace gca {

  class g3_instr : public circular_arc_instr {
  public:
  g3_instr(value* xp, value* yp, value* zp,
	   value* ip, value* jp, value* kp,
	   value* frp,
	   plane pl) : circular_arc_instr(xp, yp, zp, ip, jp, kp, frp, pl) {}

    // TODO: Add plane parameter
    static g3_instr* make(value* x, value* y, value* z,
			  value* i, value* j, value* k,
			  value* feed_rate) {
      g3_instr* mem = allocate<g3_instr>();
      return new (mem) g3_instr(x, y, z, i, j, k, feed_rate, XY);
    }
  
    
    virtual inline bool is_G3() const { return true; }
    virtual inline bool is_g3_instr() const { return true; }

    void print(ostream& s) const {
      s << "G3 ";
      print_move_data(s, 0.000001);
      print_arc_data(s, 0.00001);
    }

    inline double get_i_val() const {
      assert(i->is_lit());
      lit* il = static_cast<lit*>(i);
      return il->v;
    }

    inline double get_j_val() const {
      assert(j->is_lit());
      lit* jl = static_cast<lit*>(j);
      return jl->v;
    }

    inline double get_k_val() const {
      assert(k->is_lit());
      lit* kl = static_cast<lit*>(k);
      return kl->v;
    }

    inline value* get_k() const { return k; }
    
    virtual bool operator==(const instr& other) const {
      if (!other.is_g3_instr()) {
	return false;
      }
      const g3_instr& other_move = static_cast<const g3_instr&>(other);

      
      // TODO: Give these bools better names
      bool pr = *x == *(other_move.x) && *y == *(other_move.y) && *z == *(other_move.z);
      bool qr = *i == *(other_move.i) && *j == *(other_move.j) && *k == *(other_move.k);
      bool fr = *feed_rate == *(other_move.feed_rate);
      return fr && pr && qr;
    }

  };
}

#endif
