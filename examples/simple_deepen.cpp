#include <cassert>
#include <iostream>

#include "core/callback.h"
#include "core/context.h"
#include "core/parser.h"

using namespace gca;

// Use for dragknife, this scales all vertical moves to fit the new depth,
// unless it is a short vertical move used to align the knife
double deepen_z(double z, double old_depth, double new_depth) {
  assert(new_depth > old_depth);
  // This is the depth of the cuts used to align the dragknife
  double push_depth = 0.007;
  double diff = new_depth - old_depth;
  double deeper_z;
  if (z >= old_depth - push_depth) {
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

double get_z(move_instr* mi) {
  value* z = mi->get_z();
  assert(z->is_lit());
  lit* z_lit = static_cast<lit*>(z);
  return z_lit->v;
}

class deepen_callback : public per_instr_callback<instr*> {
public:
  double old_depth, new_depth;

  deepen_callback(double old_dp, double new_dp) :
    old_depth(old_dp), new_depth(new_dp) {}

  instr* call_default(gprog* p, int i, instr* is) {
    return is;
  }

  instr* call_G0(gprog* p, int i, move_instr* mi) {
    return deepen_linear_move(mi);
  }

  instr* call_G1(gprog* p, int i, move_instr* mi) {
    return deepen_linear_move(mi);
  }

  instr* call_G2(gprog* p, int i, g2_instr* is) {
    assert(is->get_z()->is_omitted());
    return is;
  }

  instr* call_G3(gprog* p, int i, g3_instr* is) {
    assert(is->get_z()->is_omitted());
    return is;
  }

  instr* deepen_linear_move(move_instr* mi) {
    if (mi->get_z()->is_omitted()) {
      return mi;
    } else {
      double z = get_z(mi);
      double deeper_z = deepen_z(z, old_depth, new_depth);
      move_instr* mi_cpy = static_cast<move_instr*>(mk_instr_cpy(mi));
      mi_cpy->set_z(mk_lit(deeper_z));
      return mi_cpy;
    }
  }
  
};

// Main driver function for deepening
gprog* deepen(gprog* p, deepen_callback& f) {
  gprog* r = new (allocate<gprog>()) gprog();
  for (int i = 0; i < p->size(); i++) {
    instr* is = (*p)[i];
    instr* deepened_is = f.call(p, i, is);
    r->push_back(deepened_is);
  }
  return r;
}

int main(int argc, char** argv) {
  if (argc != 4) {
    cout << "Usage: gdiff <old depth> <new depth> <gcode file path>" << endl;
    return 0;
  }
  arena_allocator a;
  set_system_allocator(&a);
  double old_depth = stod(argv[1]);
  double new_depth = stod(argv[2]);
  deepen_callback call(old_depth, new_depth);
  string file = argv[3];
  
  gprog* p = read_file(file);
  gprog* r = deepen(p, call);
  cout << *r;
  return 0;
}
