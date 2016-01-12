#include "core/instr.h"
#include "core/value.h"

namespace gca {

  class assign_instr : public instr {
  public:
    var* v;
    value* e;

  assign_instr(var* vp, value* ep) :
    v(vp), e(ep) {}

    virtual inline bool is_assign_instr() const { return true; }

    virtual bool operator==(const instr& other) const {
      if (other.is_assign_instr()) {
	const assign_instr& a_other = static_cast<const assign_instr&>(other);
	return *(a_other.v) == *v && *(a_other.e) == *e;
      }
      return false;
    }

    virtual void print(ostream& s) const {
      s << *v << "=" << *e;
    }

  };
}
