#ifndef GCA_EXTRACT_CUTS_H
#define GCA_EXTRACT_CUTS_H

#include "core/basic_states.h"
#include "core/gprog.h"
#include "core/lexer.h"
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

  pass* mk_pos_pass(point start);

  point get_diff(pass* p);

  point get_before(pass* p);

  bool is_cut_G1(pass* p, instr* is);

  void extract_cuts(gprog* p, vector<cut_section>& g1_sections);

  void extract_cuts(gprog* p, vector<cut_section>& g1_sections);  
}

#endif
