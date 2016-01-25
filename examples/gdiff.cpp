#include <cassert>
#include <iostream>

#include "core/context.h"
#include "core/parser.h"
#include "diffs/all.h"

using namespace gca;
using namespace std;

class section {
public:
  gprog& p;
  int start;
  int end;
  vector<diff*> diffs;
};

void summarize_diffs(vector<diff*>& old_diffs, vector<diff*>& new_diffs) {
  if (old_diffs.size() == 0) {
    return;
  }
  diff* cd = old_diffs[0];
  new_diffs.push_back(cd);
  for (int i = 1; i < old_diffs.size(); i++) {
    diff* nd = old_diffs[i];
    if (!cd->same_effect(*nd)) {
      cd = nd;
      new_diffs.push_back(nd);
    }
  }
}

diff* explain_diff(instr* i1, instr* i2) {
  if ((i1->is_G0() && i2->is_G0()) ||
      (i1->is_G1() && i2->is_G1())) {
    move_instr* m1 = static_cast<move_instr*>(i1);
    move_instr* m2 = static_cast<move_instr*>(i2);
    if (*(m1->feed_rate) == *(m2->feed_rate) &&
	m1->is_concrete() && m2->is_concrete()) {
      return new gca::shift_xyz(m2->pos() - m1->pos());
    }
  }
  return new gca::swap(i1, i2);
}

void diff_gprogs(vector<diff*>& diffs, gprog* p1, gprog* p2) {
  int i;
  for (i = 0; i < p1->size() && i < p2->size(); i++) {
    instr* is1 = (*p1)[i];
    instr* is2 = (*p2)[i];
    if (*is1 != *is2) {
      diff* d = explain_diff(is1, is2);
      diffs.push_back(d);
    }
  }
  if (p1->size() == p2->size()) {
    return;
  }
  bool p1_larger = p1->size() > p2->size();
  if (p1_larger) {
    for (int j = i; j < p1->size(); j++ ) {
      diffs.push_back(new gca::remove((*p1)[j]));
    }
  } else {
    for (int j = i; j < p2->size(); j++ ) {
      diffs.push_back(new gca::append((*p2)[j]));
    }    
  }
  return;
}

bool is_toolpath_start(instr* i) {
  if (i->is_comment()) {
    comment* c = static_cast<comment*>(i);
    return c->text.find("TOOLPATH NAME: ") == 0;
  }
  return false;
}

void split_toolpaths(context& c, vector<gprog*>& tps, gprog* p) {
  gprog* t = c.mk_gprog();
  for (int i = 0; i < p->size(); i++) {
    instr* is = (*p)[i];
    if (is_toolpath_start(is)) {
      if (t->size() > 0) {
	tps.push_back(t);
      }
      t = c.mk_gprog();
    }
    t->push_back(is);    
  }
  if (t->size() > 0) {
    tps.push_back(t);
  }  
}

void compute_diff_summary(vector<diff*>& diff_summary, gprog* tp1, gprog* tp2) {
  vector<diff*> diffs;
  diff_gprogs(diffs, tp1, tp2);
  summarize_diffs(diffs, diff_summary);
}

void show_diff_summary(int section_num, vector<diff*>& diff_summary) {
  cout << "========== Section " << section_num << endl;
  for (int j = 0; j < diff_summary.size(); j++) {
    cout << "\t" << *diff_summary[j] << endl;
  }
}

void gdiff_programs(context& c, gprog* p1, gprog* p2) {
  vector<gprog*> toolpaths1;
  split_toolpaths(c, toolpaths1, p1);
  vector<gprog*> toolpaths2;
  split_toolpaths(c, toolpaths2, p2);
  assert(toolpaths1.size() == toolpaths2.size());
  for (int i = 0; i < toolpaths1.size(); i++) {
    vector<diff*> diff_summary;
    compute_diff_summary(diff_summary, toolpaths1[i], toolpaths2[i]);
    show_diff_summary(i, diff_summary);
  }
}

int main(int argc, char** argv) {
  if (argc != 3) {
    cout << "Usage: gdiff <gcode_file_path> <gcode_file_path>" << endl;
    return 0;
  }
  string file1 = argv[1];
  string file2 = argv[2];
  context c;
  gprog* p1 = read_file(c, file1);
  gprog* p2 = read_file(c, file2);
  gdiff_programs(c, p1, p2);
  return 0;
}
