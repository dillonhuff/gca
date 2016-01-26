#include <cassert>
#include <iostream>
#include <map>
#include <set>

#include "core/context.h"
#include "core/parser.h"

using namespace gca;

double get_z(move_instr* mi) {
  value* z = mi->get_z();
  assert(z->is_lit());
  lit* z_lit = static_cast<lit*>(z);
  return z_lit->v;
}

double deepen_z(double z, double push_depth, double old_depth, double new_depth) {
  assert(new_depth > old_depth);
  double diff = new_depth - old_depth;
  double deeper_z;
  if (z > old_depth || within_eps(z, old_depth - push_depth)) {
    deeper_z = z + diff;
  } else if (old_depth >= z && z > 0) {
    double k = old_depth / z;
    deeper_z = z + diff/k;
  } else if (z == 0.0) {
    deeper_z = z;
  } else {
    cout << "Unsupported z = " << z << endl;
    assert(false);
  }
  double z_max = 2.0;
  assert(z_max >= deeper_z && deeper_z >= 0.0);
  return deeper_z;
}

gprog* deepen(context& c, gprog* p, double push_depth, double old_depth, double new_depth) {
  gprog* r = c.mk_gprog();
  for (int i = 0; i < p->size(); i++) {
    instr* is = (*p)[i];
    if (is->is_G0()) {
      move_instr* mi = static_cast<move_instr*>(is);
      if (!mi->get_z()->is_omitted()) {
	double z = get_z(mi);
	double deeper_z = deepen_z(z, push_depth, old_depth, new_depth);
	r->push_back(c.mk_G0(mi->get_x(), mi->get_y(), c.mk_lit(deeper_z)));
      } else {
	r->push_back(mi);
      }
    } else if (is->is_G1()) {
      move_instr* mi = static_cast<move_instr*>(is);
      if (!mi->get_z()->is_omitted()) {
	double z = get_z(mi);
	double deeper_z = deepen_z(z, push_depth, old_depth, new_depth);
	r->push_back(c.mk_G1(mi->get_x(), mi->get_y(), c.mk_lit(deeper_z), mi->feed_rate));
      } else {
	r->push_back(mi);
      }      
    } else if (is->is_g2_instr() || is->is_g3_instr()) {
      move_instr* mi = static_cast<move_instr*>(is);
      assert(mi->get_z()->is_omitted());
      r->push_back(mi);
    } else {
      r->push_back(is);
    }
  }
  return r;
}

int main(int argc, char** argv) {
  if (argc != 5) {
    cout << "Usage: gdiff <old depth> <new depth> <drag knife push depth> <gcode file path>" << endl;
    return 0;
  }
  double old_depth = stod(argv[1]);
  double new_depth = stod(argv[2]);
  double push_depth = stod(argv[3]);
  string file = argv[4];
  context c;
  gprog* p = read_file(c, file);
  gprog* r = deepen(c, p, push_depth, old_depth, new_depth);
  cout << *r;
  return 0;
}
