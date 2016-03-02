#include "core/instrs/all.h"
#include "transformers/feed_changer.h"

namespace gca {
  
  gprog* change_feeds(gprog* p, value* initial_feedrate, value* new_feedrate) {
    gprog* r = gprog::make();
    for (ilist::iterator it = p->begin(); it != p->end(); ++it) {
      instr* i = *it;
      if (i->is_G1()) {
	g1_instr* g1 = static_cast<g1_instr*>(i);
	r->push_back(g1_instr::make(g1->get_x(), g1->get_y(), g1->get_z(), new_feedrate));
      } else {
	r->push_back(i);
      }
    }
    return r;
  }

  gprog* generalize_feeds(gprog* p, value* default_val, value* initial_feedrate, var* new_feedrate) {
    gprog* r = gprog::make();
    r->push_back(assign_instr::make(new_feedrate, default_val));
    for (ilist::iterator it = p->begin(); it != p->end(); ++it) {
      instr* i = *it;
      if (i->is_G1()) {
	g1_instr* g1 = static_cast<g1_instr*>(i);
	r->push_back(g1_instr::make(g1->get_x(), g1->get_y(), g1->get_z(), new_feedrate));
      } else {
	r->push_back(i);
      }
    }
    return r;
  }
  
}
