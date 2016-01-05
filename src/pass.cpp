#include "pass.h"
namespace gca {
  
  state* state::get_state(state_name n)  {
    assert(t != NULL); return t->get_state(n);
  }
}
