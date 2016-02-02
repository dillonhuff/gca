#ifndef GCA_CIRCULAR_ARC_INSTR_H
#define GCA_CIRCULAR_ARC_INSTR_H

#include "core/instrs/move_instr.h"

namespace gca {

  class circular_arc_instr : public move_instr {
  public:
    value* i;
    value* j;
    value* k;
    
    circular_arc_instr(value* x, value* y, value* z,
		       value* ip, value* jp, value* kp,
		       value* feed_rate) : move_instr(x, y, z, feed_rate),
      i(ip), j(jp), k(kp) {}

    void print_arc_data(ostream& s, double eps) const {
      if (!i->is_omitted()) {
	s << 'I';
	i->print_eps(s, eps);
	s << ' ';
      }
      if (!j->is_omitted()) {
	s << 'J';
	j->print_eps(s, eps);
	s << ' ';
      }
      if (!k->is_omitted()) {
	s << 'K';
	k->print_eps(s, eps);
	s << ' ';
      }
    }

  };
}

#endif
