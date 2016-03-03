#include <algorithm>

#include "core/arena_allocator.h"
#include "synthesis/schedule_cuts.h"

namespace gca {

  typedef vector<cut*> cut_group;

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
    // TODO: Tune this magic number to the actual drag knife value
    double max_orientation_change = 15;
    if (last->tool_no != next->tool_no) {
      return false;
    }
    if (next->tool_no == DRAG_KNIFE &&
	!within_eps(angle_between(last->final_orient(),
				  next->initial_orient()),
		    0,
		    max_orientation_change)) {
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

  void schedule_cut_groups(vector<cut_group*>& groups) {
  }

  vector<cut_group*> group_cuts(const vector<cut*>& cuts) {
    vector<cut_group*> groups;
    vector<cut*>::const_iterator it = cuts.begin();
    while (it < cuts.end()) {
      cut_group next_chain = contiguous_chain(it, cuts.end());
      elem_test rt(next_chain);
      groups.push_back(new (allocate<cut_group>()) cut_group());
      groups.back()->insert(groups.back()->end(),
			    next_chain.begin(),
			    next_chain.end());
      it += next_chain.size();
    }
    return groups;
  }

  vector<cut*> concat_cut_groups(const vector<cut_group*>& groups) {
    vector<cut*> cuts;
    return cuts;
  }

  vector<cut*> schedule_cuts(const vector<cut*>& cuts) {
    for_each(cuts.begin(), cuts.end(), has_tool);
    vector<cut_group*> groups = group_cuts(cuts);
    schedule_cut_groups(groups);
    vector<cut*> scheduled_cuts = concat_cut_groups(groups);
    stable_partition(scheduled_cuts.begin(), scheduled_cuts.end(), is_hole_punch);
    return scheduled_cuts;
  }

}
