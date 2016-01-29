#ifndef GCA_EXTRACT_CUTS_H
#define GCA_EXTRACT_CUTS_H

#include "core/basic_states.h"
#include "core/context.h"
#include "core/gprog.h"
#include "core/pass.h"
#include "geometry/point.h"

namespace gca {

  class cut_section {
  public:
    point start;
    gprog* p;

  cut_section(point s, gprog* pp) : start(s), p(pp) {}

    point start_orientation() {
      assert(p->size() > 0);
      point s = start_pos();
      point first_cut_end = static_cast<g1_instr*>((*p)[0])->pos();
      return first_cut_end - s;
    }    
    point end_orientation() {
      if (p->size() == 1) {
	return start_orientation();
      }
      point s = static_cast<g1_instr*>((*p)[p->size() - 2])->pos();
      point e = static_cast<g1_instr*>((*p)[p->size() - 1])->pos();
      return e - s;
    }
  
    point end_pos() {
      assert(p->size() > 0);
      instr* is = (*p)[p->size() - 1];
      assert(is->is_G1());
      g1_instr* mi = static_cast<g1_instr*>(is);
      assert(mi->is_concrete());
      return mi->pos();
    }

    point start_pos() {
      return start;
    }
  
  };

  pass* mk_pos_pass(point start) {
    pass* ps = new (allocate<pass>()) pass();
    orientation_state* orient_s = new (allocate<orientation_state>()) orientation_state(*ps, GCA_ABSOLUTE);
    position_state* pis = new (allocate<position_state>()) position_state(*ps, start);
    ps->add_state(GCA_POSITION_STATE, pis);
    ps->add_state(GCA_ORIENTATION_STATE, orient_s);
    return ps;
  }

  point get_diff(pass* p) {
    position_state* pos_state = p->get_state<position_state>(GCA_POSITION_STATE);
    return pos_state->diff;
  }

  point get_before(pass* p) {
    position_state* pos_state = p->get_state<position_state>(GCA_POSITION_STATE);
    return pos_state->before;
  }

  bool is_cut_G1(pass* p, instr* is) {
    if (!is->is_G1()) {
      return false;
    }
    point diff = get_diff(p);
    return diff.z == 0 && (diff.x != 0 || diff.y != 0);
  }

  void extract_cuts(gprog* p, vector<cut_section>& g1_sections) {
    pass* s = mk_pos_pass(point(0, 0, 0));  
    int i = 0;
    bool last_was_g1 = false;  
    gprog* current = mk_gprog();
    point last_start = point(0, 0, 0);
    while (i < p->size()) {
      instr* ist = (*p)[i];
      s->update(ist);
      if (is_cut_G1(s, ist) && last_was_g1) {
	current->push_back(ist);
      } else if (is_cut_G1(s, ist)) {
	last_was_g1 = true;
	if (current->size() > 0) {
	
	  g1_sections.push_back(cut_section(last_start, current));
	}
	last_start = get_before(s);
	current = mk_gprog();
	current->push_back(ist);
      } else {
	last_was_g1 = false;
      }
      i++;
    }
    if (current->size() > 0) {
      g1_sections.push_back(cut_section(last_start, current));
    }
  }

}
#endif
