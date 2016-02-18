#include <cassert>
#include <iostream>

#include "analysis/extract_cuts.h"
#include "core/basic_states.h"
#include "core/callback.h"
#include "core/context.h"
#include "core/parser.h"
#include "synthesis/align_blade.h"
#include "synthesis/output.h"

using namespace gca;
using namespace std;

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
  for (unsigned i = 0; i < p1.p->size(); i++) {
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
  for (unsigned i = 0; i < already_merged.size(); i++) {
    if (duplicate_ignoring_z(section, already_merged[i])) {
      return false;
    }
  }
  return true;
}

void merge_cut_sections(vector<cut_section>& g1_sections,
			vector<cut_section>& merged_cuts) {
  for (unsigned i = 0; i < g1_sections.size(); i++) {
    cut_section sec = g1_sections[i];
    if (no_duplicates(sec, merged_cuts)) {
      merged_cuts.push_back(sec);
    }
  }
}

gprog* generate_drag_knife_code(double safe_height,
				double align_depth,
				point start_pos,
				point start_orient,
				vector<cut_section>& sections) {
  point last_section_end_pos = start_pos;
  point last_section_end_orientation = start_orient;
  gprog* res = initial_gprog();
  for (unsigned i = 0; i < sections.size(); i++) {
    cut_section sec = sections[i];    
    cout << "[ section starting at " << sec.start << " ]" << endl;
    cout << "[ section of size " << (sec.p)->size() << " ]" << endl;
    from_to_with_G0_drag_knife(safe_height,
			       align_depth,
			       res,
			       last_section_end_pos,
			       last_section_end_orientation,
			       sec.start,
			       sec.start_orientation());
    for (unsigned j = 0; j < (sec.p)->size(); j++) {
      res->push_back((*(sec.p))[j]);
    }
    last_section_end_pos = sec.end_pos();
    last_section_end_orientation = sec.end_orientation();
  }
  res = append_footer(res);
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
					merged_cuts);
  cout << "[ Reconstructed program ]" << endl;
  cout << *res;
  return 0;
}
