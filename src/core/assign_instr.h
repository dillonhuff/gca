#include "core/instr.h"
#include "core/value.h"

namespace gca {

  class assign_instr : public instr {
  public:
    var* v;
    value* e;

  assign_instr(var* vp, value* ep) :
    v(vp), e(ep) {}

  };
}
