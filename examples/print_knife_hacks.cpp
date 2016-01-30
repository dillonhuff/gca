#include <cassert>
#include <dirent.h>
#include <iostream>

#include "analysis/extract_cuts.h"
#include "core/basic_states.h"
#include "core/callback.h"
#include "core/context.h"
#include "core/parser.h"
#include "synthesis/align_blade.h"

using namespace gca;
using namespace std;

inline bool ends_with(string const& value, string const& ending) {
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

unsigned hack_length = 8;

bool is_align_hack(const vector<instr*>& window) {
  if (window.size() != hack_length) { return false; }
  if (!window[0]->is_G1()) { return false; }
  if (!window[1]->is_G1()) { return false; }
  if (!window[2]->is_G0()) { return false; }
  if (!window[3]->is_G0()) { return false; }
  if (!window[4]->is_G1()) { return false; }
  if (!(window[5]->is_G2() || window[3]->is_G3())) { return false; }
  if (!window[6]->is_G1()) { return false; }
  if (!window[7]->is_G1()) { return false; }
  return true;
}

point get_offset(move_instr* is) {
  if (is->is_G2()) {
    g2_instr* mi = static_cast<g2_instr*>(is);
    assert(mi->get_k()->is_omitted());
    return point(mi->get_i_val(), mi->get_j_val(), 0);
  } else {
    assert(is->is_G3());
    g3_instr* mi = static_cast<g3_instr*>(is);
    assert(mi->get_k()->is_omitted());
    return point(mi->get_i_val(), mi->get_j_val(), 0);
  }
  return point(0, 0, 0);
}

void print_window_info(const vector<instr*>& window) {
  cout << endl << "-- window " << endl;
  point start_pos = static_cast<g1_instr*>(window[1])->pos();
  point start_cut = static_cast<g1_instr*>(window[0])->pos();
  point current_orient = start_pos - start_cut;
  point end_pos = static_cast<g1_instr*>(window[6])->pos();
  point end_cut = static_cast<g1_instr*>(window[7])->pos();
  point desired_orient = end_cut - end_pos;
  cout << "theta = " << angle_between(desired_orient, current_orient) << endl;
  point circle_start = static_cast<g1_instr*>(window[4])->pos();
  move_instr* ci = static_cast<move_instr*>(window[5]);
  point circle_end(ci->x_with_default(0),
		   ci->y_with_default(0),
		   ci->z_with_default(circle_start.z));
  cout << "circle start = " << circle_start << endl;
  cout << "cicle end = " << circle_end << endl;
  point circle_offset = get_offset(static_cast<move_instr*>(window[5]));
  cout << "circle radius = " << circle_offset.len() << endl;
  point circle_center = circle_start + circle_offset;
  cout << "circle center = " << circle_center << endl;;
  point ev = circle_end - circle_center;
  point sv = circle_start - circle_center;
  cout << "ev = " << ev << endl;
  cout << "sv = " << sv << endl;
  cout << "angle between start and end = " << angle_between(sv, ev) << endl;
  cout << "Instructions: " << endl;
  for (unsigned j = 0; j < window.size(); j++) {
    cout << *(window[j]) << endl;
  }
  assert(within_eps(ev.len(), sv.len(), 0.0001));
}
void print_knife_hacks(gprog* prog) {
  vector<instr*> window;
  for (unsigned i = 0; i < prog->size(); i++) {
    instr* is = (*prog)[i];
    if (window.size() >= hack_length) {
      window.erase(window.begin());
    }
    window.push_back(is);
    if (is_align_hack(window)) {
      print_window_info(window);
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

  string dir_name = argv[1];
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir(dir_name.c_str())) != NULL) {
    while ((ent = readdir(dir)) != NULL) {
      string fname = ent->d_name;
      if (ends_with(fname, ".tap")) {
	print_knife_hacks(read_file(dir_name + fname));
      }
    }
    closedir(dir);
  } else {
    cout << "Could not open directory: " << dir_name << endl;
    return EXIT_FAILURE;
  }
}
