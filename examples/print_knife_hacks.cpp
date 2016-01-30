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

class circular_arc {
public:
  point start;
  point end;
  point start_offset;
  circular_arc(point sp, point ep, point so) : start(sp), end(ep), start_offset(so) {}
};

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
  double theta = angle_between(desired_orient, current_orient);
  cout << "angle between desired and current orient = " << theta << endl;
  double rad = 0.16;
  point circle_end = end_pos;
  circle_end.z = 0.093;
  point ef = rad * circle_end.normalize();
  cout << "Computed ef " << ef << endl;
  point circle_center = circle_end - rad*desired_orient.normalize();
  cout << "Computed circle center: " << circle_center << endl;
  return circular_arc(point(0, 0, 0), circle_end, point(0, 0, 0));
}

ostream& operator<<(ostream& s, const circular_arc& a) {
  s << a.start << " " << a.end << " " << a.start_offset;
  return s;
}
void print_window_info(const vector<instr*>& window) {
  cout << endl << "-- window " << endl;
  circular_arc ca = extract_arc(window);
  cout << "Extracted: " << ca << endl;
  cout << "Extracted circle radius = " << ca.start_offset.len() << endl;
  point circle_center = ca.start + ca.start_offset;
  cout << "Extracted circle center = " << circle_center << endl;;
  point ev = ca.end - circle_center;
  point sv = ca.start - circle_center;
  cout << "ev = " << ev << endl;
  cout << "sv = " << sv << endl;
  cout << "angle between start and end = " << angle_between(sv, ev) << endl;
  cout << "Extracted angle between ev and end = " << angle_between(ca.end, ev) << endl;
  //cout << "angle between desired orient and circle rad = " << angle_between(desired_orient, ev) << endl;  
  circular_arc my_a = compute_arc(window);
  cout << "Computed: " << my_a << endl;
  // // Align coords code
  // point c_pos;
  // point center_off;
  // gprog prog;
  // point end_pos_xy = ca.end;
  // end_pos_xy.z = 0;
  // align_coords(desired_orient, end_pos_xy, current_orient,
  // 	       0.016, c_pos, center_off);
  // cout << "computed center offset in align: " << center_off << endl;
  // cout << "actual center offset: " << circle_offset << endl;
  // assert(within_eps(circle_offset, center_off));
  // // End align coords code
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
