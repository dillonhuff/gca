#ifndef GCA_G2_INSTR_H
#define GCA_G2_INSTR_H

#include "core/instrs/move_instr.h"

namespace gca {

  class g2_instr : public move_instr {
  public:
    value* i;
    value* j;
    value* k;
    
    g2_instr(value* x, value* y, value* z,
		      value* ip, value* jp, value* kp,
	     value* feed_rate) : move_instr(x, y, z, feed_rate),
    i(ip), j(jp), k(kp) {}

    virtual inline bool is_g2_instr() const { return true; }

    void print(ostream& s) const {
      s << "Printing G2" << endl;
      s << "G02 ";
      if (!feed_rate->is_omitted()) { s << 'F' << *feed_rate << ' '; }
      if (!x->is_omitted()) { s << 'X' << *x << ' '; }
      if (!y->is_omitted()) { s << 'Y' << *y << ' '; }
      if (!z->is_omitted()) { s << 'Z' << *z << ' '; }
      if (!i->is_omitted()) { s << 'I' << *i << ' '; }
      if (!j->is_omitted()) { s << 'J' << *j << ' '; }
      if (!k->is_omitted()) { s << 'K' << *k << ' '; }      
    }

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
