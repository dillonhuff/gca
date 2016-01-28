#include <cassert>
#include <iostream>

#include "core/basic_states.h"
#include "core/callback.h"
#include "core/context.h"
#include "core/parser.h"

using namespace gca;

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

class cut_section {
public:
  point start;
  gprog* p;

  cut_section(point s, gprog* pp) : start(s), p(pp) {}
};

void extract_cuts(gprog* p, vector<cut_section>& g1_sections) {
  pass* s = mk_pos_pass(point(0, 0, 0));  
  int i = 0;
  bool last_was_g1 = false;  
  gprog* current = mk_gprog();
  while (i < p->size()) {
    instr* ist = (*p)[i];
    s->update(ist);
    if (is_cut_G1(s, ist) && last_was_g1) {
      current->push_back(ist);
    } else if (is_cut_G1(s, ist)) {
      last_was_g1 = true;
      if (current->size() > 0) {
	g1_sections.push_back(cut_section(get_before(s), current));
      }
      current = mk_gprog();
      current->push_back(ist);
    } else {
      last_was_g1 = false;
    }
    i++;
  }
  g1_sections.push_back(cut_section(get_before(s), current));
}

bool duplicate_ignoring_z(cut_section& p1, cut_section p2) {
  if (p1.p->size() != p2.p->size()) {
    return false;
  }
  for (int i = 0; i < p1.p->size(); i++) {
    instr* is1 = (*(p1.p))[i];
    assert(is1->is_G1());
    g1_instr* mi1 = static_cast<g1_instr*>(is1);
    assert(mi1->is_concrete());
    point mv1 = mi1->pos();
    mv1.z = 0.0;
    instr* is2 = (*(p2.p))[i];
    assert(is2->is_G1());
    g1_instr* mi2 = static_cast<g1_instr*>(is2);
    assert(mi2->is_concrete());
    point mv2 = mi2->pos();
    mv2.z = 0.0;
    if (!within_eps(mv1, mv2)) {
      return false;
    }
  }
  return true;
}

bool no_duplicates(cut_section& section, vector<cut_section>& already_merged) {
  for (int i = 0; i < already_merged.size(); i++) {
    if (duplicate_ignoring_z(section, already_merged[i])) {
      return false;
    }
  }
  return true;
}

void merge_cut_sections(vector<cut_section>& g1_sections,
			vector<cut_section>& merged_cuts) {
  for (int i = 0; i < g1_sections.size(); i++) {
    cut_section sec = g1_sections[i];
    if (no_duplicates(sec, merged_cuts)) {
      merged_cuts.push_back(sec);
    }
  }
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: extract-g1-paths <gcode file path>" << endl;
    return 0;
  }
  arena_allocator a;
  set_system_allocator(&a);
  string file = argv[1];
  gprog* p = read_file(file);
  vector<cut_section> g1_sections;
  extract_cuts(p, g1_sections);
  vector<cut_section> merged_cuts;
  merge_cut_sections(g1_sections, merged_cuts);
  for (int i = 0; i < merged_cuts.size(); i++) {
    cut_section sec = merged_cuts[i];
    cout << "-- section of size " << (sec.p)->size() << endl;
    for (int j = 0; j < (sec.p)->size(); j++) {
      cout << *((*(sec.p))[j]) << endl;
    }
  }
  return 0;
}
