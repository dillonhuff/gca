#include <algorithm>

#include "synthesis/schedule_cuts.h"

namespace gca {

  bool cmp_z(const cut* l, const cut* r) {
    assert(within_eps(l->start.z, l->end.z));
    assert(within_eps(r->start.z, r->end.z));
    return l->start.z > r->start.z;
  }

  struct elem_test {
    const vector<cut*>& elems;
    elem_test(const vector<cut*>& pelems) : elems(pelems) {}
    bool operator()(const cut* t) {
      return find(elems.begin(), elems.end(), t) != elems.end();
    }
  };

  bool are_contiguous(const cut* last, const cut* next) {
    if (last->tool_no != next->tool_no) {
      return false;
    }
    return within_eps(last->end, next->start);
  }

  vector<cut*> contiguous_chain(vector<cut*>::const_iterator it,
				vector<cut*>::const_iterator e) {
    cut* last_cut = NULL;
    cut* next_cut = NULL;
    vector<cut*> cont_chain;
    for (; it != e; ++it) {
      next_cut = *it;
      if (last_cut == NULL || are_contiguous(last_cut, next_cut)) {
	cont_chain.push_back(next_cut);
	last_cut = next_cut;
      }
    }
    return cont_chain;
  }


  bool is_hole_punch(const cut* c) {
    return c->is_hole_punch();
  }
  
  void has_tool(const cut* c) {
    assert(c->tool_no == DRILL || c->tool_no == DRAG_KNIFE);
  }
  
  vector<cut*> schedule_cuts(const vector<cut*>& cuts) {
    for_each(cuts.begin(), cuts.end(), has_tool);
    vector<cut*> scheduled_cuts = cuts;
    vector<cut*>::iterator it = scheduled_cuts.begin();
    while (it < scheduled_cuts.end()) {
      vector<cut*> next_chain = contiguous_chain(it, scheduled_cuts.end());
      elem_test rt(next_chain);
      stable_partition(it, scheduled_cuts.end(), rt);
      it += next_chain.size();
    }
    stable_partition(scheduled_cuts.begin(), scheduled_cuts.end(), is_hole_punch);
    return scheduled_cuts;
  }

}
