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

double deepen_z(double z) {
  if (within_eps(z, 0.35) || within_eps(z, 0.143)) {
    return z + 0.015;
  } else if (within_eps(z, 0.075)) {
    return z + 0.0075;
  } else if (within_eps(z, 0.8) || within_eps(z, 0.0)) {
    return z;
  } else {
    cout << "Unsupported z = " << z << endl;
    assert(false);
  }
}

gprog* deepen(context& c, gprog* p) {
  gprog* r = c.mk_gprog();
  for (int i = 0; i < p->size(); i++) {
    instr* is = (*p)[i];
    if (is->is_G0()) {
      move_instr* mi = static_cast<move_instr*>(is);
      if (!mi->get_z()->is_omitted()) {
	double z = get_z(mi);
	double deeper_z = deepen_z(z);
	r->push_back(c.mk_G0(mi->get_x(), mi->get_y(), c.mk_lit(deeper_z)));
      } else {
	r->push_back(mi);
      }
    } else if (is->is_G1()) {
      move_instr* mi = static_cast<move_instr*>(is);
      if (!mi->get_z()->is_omitted()) {
	double z = get_z(mi);
	double deeper_z = deepen_z(z);
	r->push_back(c.mk_G1(mi->get_x(), mi->get_y(), c.mk_lit(deeper_z), mi->feed_rate));
      } else {
	r->push_back(mi);
      }      
    } else if (is->is_g2_instr()) {
      g2_instr* mi = static_cast<g2_instr*>(is);
      assert(mi->get_z()->is_omitted());
      r->push_back(mi);
    } else if (is->is_g3_instr()) {
      g3_instr* mi = static_cast<g3_instr*>(is);
      assert(mi->get_z()->is_omitted());
      r->push_back(mi);
    } else {
      r->push_back(is);
    }
  }
  return r;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: gdiff <gcode_file_path>" << endl;
    return 0;
  }
  string file = argv[1];
  context c;
  gprog* p = read_file(c, file);
  gprog* r = deepen(c, p);
  cout << *r;
  return 0;
}
