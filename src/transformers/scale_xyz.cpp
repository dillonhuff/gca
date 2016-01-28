#include "core/context.h"
#include "transformers/scale_xyz.h"

namespace gca {

  class scale_callback : public per_instr_callback<instr*> {
  public:

    instr* call_M2(gprog* p, int i, instr* is) { return is; }
    instr* call_default(gprog* p, int i, instr* is) { assert(false); }
  };

  gprog* scale_xyz(double x_s, double y_s, double z_s, gprog& p) {
    gprog* r = mk_gprog();
    scale_callback c;
    for (int i = 0; i < p.size(); i++) {
      r->push_back(c.call(&p, i, p[i]));
    }
    return r;
  }
  
}
