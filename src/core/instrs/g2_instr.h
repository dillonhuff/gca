#ifndef GCA_G2_INSTR_H
#define GCA_G2_INSTR_H

#include "core/instrs/circular_arc_instr.h"

namespace gca {

  class g2_instr : public circular_arc_instr {
  public:
    g2_instr(value* xp, value* yp, value* zp,
	     value* ip, value* jp, value* kp,
	     value* frp,
	     plane pl) : circular_arc_instr(xp, yp, zp, ip, jp, kp, frp, pl) {}

    virtual inline bool is_G2() const { return true; }    
    virtual inline bool is_g2_instr() const { return true; }

    void print(ostream& s) const {
      s << "G2 ";
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
      if (!other.is_g2_instr()) {
	return false;
      }
      const g2_instr& other_move = static_cast<const g2_instr&>(other);

      // TODO: Give these bools better names
      bool pr = *x == *(other_move.x) && *y == *(other_move.y) && *z == *(other_move.z);
      bool qr = *i == *(other_move.i) && *j == *(other_move.j) && *k == *(other_move.k);
      bool fr = *feed_rate == *(other_move.feed_rate);
      return fr && pr && qr;
    }

  };
}

#endif
