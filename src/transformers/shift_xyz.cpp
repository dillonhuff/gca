#include "transformers/shift_xyz.h"

#include "core/context.h"
#include "transformers/scale_xyz.h"

namespace gca {

  class shift_callback : public per_instr_callback<instr*> {
  public:
    double x_s, y_s, z_s;

    shift_callback(double x, double y, double z) : x_s(x), y_s(y), z_s(z) {}

    value* shift_value(double s, value* v) {
      if (v->is_omitted()) {
	return v;
      }
      assert(v->is_lit());
      lit* l = static_cast<lit*>(v);
      return lit::make(s + l->v);
    }

    instr* shift_move(move_instr* is) {
      value* new_x = shift_value(x_s, is->get_x());
      value* new_y = shift_value(y_s, is->get_y());
      value* new_z = shift_value(z_s, is->get_z());
      move_instr* new_is = static_cast<move_instr*>(is->copy());
      new_is->set_x(new_x);
      new_is->set_y(new_y);
      new_is->set_z(new_z);
      return new_is;
    }
    
    instr* call_G0(gprog* p, int i, g0_instr* is) { return shift_move(is); }
    instr* call_G1(gprog* p, int i, g1_instr* is) { return shift_move(is); }
    instr* call_G90(gprog* p, int i, g90_instr* is) { return is; }
    instr* call_M2(gprog* p, int i, m2_instr* is) { return is; }
    instr* call_default(gprog* p, int i, instr* is) { assert(false); }
  };

  gprog* shift_xyz(double x_s, double y_s, double z_s, gprog& p) {
    gprog* r = gprog::make();
    shift_callback c(x_s, y_s, z_s);
    for (unsigned i = 0; i < p.size(); i++) {
      r->push_back(c.call(&p, i, p[i]));
    }
    return r;
  }
  
}

