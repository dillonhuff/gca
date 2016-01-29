#include <cassert>
#include <iostream>

#include "core/basic_states.h"
#include "core/callback.h"
#include "core/context.h"
#include "core/parser.h"
#include "synthesis/align_blade.h"

using namespace gca;
using namespace std;

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

  point start_orientation() {
    assert(p->size() > 0);
    point s = start_pos();
    cout << "Start orientation s = " << s << endl;
    point first_cut_end = static_cast<g1_instr*>((*p)[0])->pos();
    cout << "Start orientation first_end_cut " << first_cut_end << endl;
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
  g1_sections.push_back(cut_section(last_start, current));
}

bool duplicate_ignoring_z(cut_section& p1, cut_section& p2) {
  point p1s = p1.start;
  p1s.z = 0;
  point p2s = p1.start;
  p2s.z = 0;  
  if (!within_eps(p1s, p2s)) {
    return false;
  }
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

double angle_between(point p1, point p2) {
  assert(false);
}

void from_to_with_G0_drag_knife(double safe_height,
				double align_depth,
				gprog* p,
				point last_pos,
				point last_orient,
				point next_pos,
				point next_orient) {
  instr* pull_up_instr = mk_G0(point(last_pos.x, last_pos.y, safe_height));
  // TODO: Set this to realistic value
  double r = 0.5;
  point c_pos;
  point circle_center_offset;
  point next_pos_xy = next_pos;
  next_pos_xy.z = 0;
  cout << "last orient = " << last_orient << endl;
  cout << "next orient = " << next_orient << endl;
  align_coords(next_orient, next_pos_xy, last_orient, r, c_pos, circle_center_offset);
  instr* move_to_c_pos_instr = mk_G0(c_pos.x, c_pos.y, safe_height);
  // TODO: Make this a parameter;
  instr* push_down_instr = mk_G1(c_pos.x, c_pos.y, align_depth, mk_omitted());
  instr* circle_move_instr = mk_G3(mk_lit(next_pos.x), mk_lit(next_pos.y), mk_omitted(),
				   mk_lit(circle_center_offset.x), mk_lit(circle_center_offset.y), mk_omitted(),
				   mk_omitted());
  instr* final_push_down_instr = mk_G1(next_pos.x, next_pos.y, next_pos.z, mk_omitted());
  p->push_back(pull_up_instr);
  p->push_back(move_to_c_pos_instr);
  p->push_back(push_down_instr);
  p->push_back(circle_move_instr);
  p->push_back(final_push_down_instr);
}

  //  double safe_height = 3.6;
gprog* generate_drag_knife_code(double safe_height,
				double align_depth,
				point start_pos,
				point start_orient,
				vector<cut_section>& sections) {
  // TODO: Change these to what the machine actually wants
  point last_section_end_pos = start_pos; //point(14.7, 5.7, 6.6);
  point last_section_end_orientation = start_orient; //point(1, 0, 0);
  gprog* res = mk_gprog();
  for (int i = 0; i < sections.size(); i++) {
    cut_section sec = sections[i];    
    cout << "[ section starting at " << sec.start << " ]" << endl;
    cout << "[ section of size " << (sec.p)->size() << " ]" << endl;
    cout << "last section orientation " << last_section_end_orientation << endl;
    from_to_with_G0_drag_knife(safe_height,
			       align_depth,
			       res,
			       last_section_end_pos,
			       last_section_end_orientation,
			       sec.start,
			       sec.start_orientation());
    for (int j = 0; j < (sec.p)->size(); j++) {
      res->push_back((*(sec.p))[j]);
    }
    last_section_end_pos = sec.end_pos();
    last_section_end_orientation = sec.end_orientation();
  }
  return res;
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
  double safe_height = 0.5;
  double align_depth = 0.087;
  point start_loc = point(15.1, 7.1, 0.83);
  point start_orient = point(1, 0, 0);
  gprog* res = generate_drag_knife_code(safe_height,
					align_depth,
					start_loc,
					start_orient,
					g1_sections);
  cout << "[ Reconstructed program ]" << endl;
  cout << *res;
  return 0;
}
