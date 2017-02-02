#include <cassert>
#include <dirent.h>
#include <iostream>

#include "analysis/extract_cuts.h"
#include "gcode/basic_states.h"
#include "gcode/callback.h"

#include "gcode/parser.h"
#include "backend/align_blade.h"

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
  if (!(window[5]->is_G2() || window[5]->is_G3())) { return false; }
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

circular_arc extract_arc(const vector<instr*>& window) {
  point circle_start = static_cast<g1_instr*>(window[4])->pos();
  move_instr* ci = static_cast<move_instr*>(window[5]);
  point circle_end(ci->x_with_default(0),
		   ci->y_with_default(0),
		   ci->z_with_default(circle_start.z));
  point circle_offset = get_offset(static_cast<move_instr*>(window[5]));
  return circular_arc(circle_start, circle_end, circle_offset);
}

circular_arc compute_arc(const vector<instr*>& window) {
  point start_pos = static_cast<g1_instr*>(window[1])->pos();
  point start_cut = static_cast<g1_instr*>(window[0])->pos();
  point current_orient = start_pos - start_cut;
  point end_pos = static_cast<g1_instr*>(window[6])->pos();
  point end_cut = static_cast<g1_instr*>(window[7])->pos();  
  point desired_orient = end_cut - end_pos;
  point circle_start;
  point circle_start_off;
  double rad = 0.16;
  point circle_end = end_pos;
  circle_end.z = 0.093;
  
  circular_arc ca = align_coords(desired_orient,
				 circle_end,
				 current_orient,
				 rad);
  
  return ca; //circular_arc(circle_start, circle_end, circle_start_off);
}

ostream& operator<<(ostream& s, const circular_arc& a) {
  s << a.get_start() << " " << a.get_end() << " " << a.start_offset;
  return s;
}

void print_window_info(const vector<instr*>& window) {
  cout << endl << "-- window " << endl;
  circular_arc ca = extract_arc(window);
  cout << "Extracted: " << ca << endl;
  cout << "Extracted circle radius = " << ca.start_offset.len() << endl;
  point circle_center = ca.get_start() + ca.start_offset;
  cout << "Extracted circle center = " << circle_center << endl;;
  point ev = ca.get_end() - circle_center;
  point sv = ca.get_start() - circle_center;
  cout << "ev = " << ev << endl;
  cout << "sv = " << sv << endl;
  cout << "angle between start and end = " << angle_between(sv, ev) << endl;
  cout << "Extracted angle between ev and end = " << angle_between(ca.get_end(), ev) << endl;
  circular_arc my_a = compute_arc(window);
  cout << "Computed: " << my_a << endl;
  cout << "Instructions: " << endl;
  for (unsigned j = 0; j < window.size(); j++) {
    cout << *(window[j]) << endl;
  }
  //  assert(within_eps(ev.len(), sv.len(), 0.0001));
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
    cout << "Usage: print-knife-hacks <gcode file path>" << endl;
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
