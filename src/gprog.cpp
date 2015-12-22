#include <iostream>

#include "gprog.h"

namespace gca {

  point gprog::last_position() {
    for (ilist::reverse_iterator rit = instrs.rbegin();
	 rit != instrs.rend(); ++rit) {
      if ((*rit)->is_G()) {
	return (*rit)->pos();
      }
    }
    assert(false);
  }

  void gprog::print(ostream& s) {
    for (ilist::iterator it = begin();
	 it != end(); ++it) {
      s << **it << endl;
    }
  }

  ostream& operator<<(ostream& stream, gprog& p) {
    p.print(stream);
    return stream;
  }
  
}
