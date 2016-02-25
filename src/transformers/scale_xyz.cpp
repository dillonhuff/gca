#include "core/context.h"
#include "transformers/scale_xyz.h"

namespace gca {

  class scale_callback : public per_instr_callback<instr*> {
  public:
    double x_s, y_s, z_s;

    scale_callback(double x, double y, double z) : x_s(x), y_s(y), z_s(z) {}

    value* scale_value(double s, value* v) {
      if (v->is_omitted()) {
	return v;
      }
      assert(v->is_lit());
      lit* l = static_cast<lit*>(v);
      return lit::make(s*l->v);
    }

    instr* scale_move(move_instr* is) {
      value* new_x = scale_value(x_s, is->get_x());
      value* new_y = scale_value(y_s, is->get_y());
      value* new_z = scale_value(z_s, is->get_z());
      move_instr* new_is = static_cast<move_instr*>(is->copy());
      new_is->set_x(new_x);
      new_is->set_y(new_y);
      new_is->set_z(new_z);
      return new_is;
    }
    
    instr* call_G0(gprog* p, int i, g0_instr* is) { return scale_move(is); }
    instr* call_G1(gprog* p, int i, g1_instr* is) { return scale_move(is); }
    instr* call_G90(gprog* p, int i, g90_instr* is) { return is; }
    instr* call_M2(gprog* p, int i, m2_instr* is) { return is; }
    instr* call_default(gprog* p, int i, instr* is) { assert(false); }
  };

  gprog* scale_xyz(double x_s, double y_s, double z_s, gprog& p) {
    gprog* r = mk_gprog();
    scale_callback c(x_s, y_s, z_s);
    for (unsigned i = 0; i < p.size(); i++) {
      r->push_back(c.call(&p, i, p[i]));
    }
    return r;
  }
  
}
