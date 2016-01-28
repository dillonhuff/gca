#ifndef GCA_CALLBACK_H
#define GCA_CALLBACK_H

#include "core/gprog.h"

namespace gca {
  
  template<typename T>
    class callback {
  public:
    virtual T call(gprog* p, int i, instr* is) = 0;
  };

  template<typename T>
    class per_instr_callback : callback<T> {
  public:

    virtual T call_G0(gprog* p, int i, move_instr* is) {
      return call_default(p, i, is);
    }
    
    virtual T call_G1(gprog* p, int i, move_instr* is) {
      return call_default(p, i, is);
    }

    virtual T call_G53(gprog* p, int i, move_instr* is) {
      return call_default(p, i, is);
    }
    
    virtual T call_G2(gprog* p, int i, g2_instr* is) {
      return call_default(p, i, is);
    }
    
    virtual T call_G3(gprog* p, int i, g3_instr* is) {
      return call_default(p, i, is);
    }
    
    virtual T call_T(gprog* p, int i, instr* is) {
      return call_default(p, i, is);
    }

    virtual T call_S(gprog* p, int i, instr* is) {
      return call_default(p, i, is);
    }
    
    virtual T call_M2(gprog* p, int i, instr* is) {
      return call_default(p, i, is);
    }

    virtual T call_M5(gprog* p, int i, instr* is) {
      return call_default(p, i, is);
    }

    virtual T call_M30(gprog* p, int i, instr* is) {
      return call_default(p, i, is);
    }
    
    virtual T call_comment(gprog* p, int i, comment* is) {
      return call_default(p, i, is);
    }

    virtual T call_G90(gprog* p, int i, instr* is) {
      return call_default(p, i, is);
    }

    virtual T call_default(gprog* p, int i, instr* is) = 0;

    virtual T call(gprog* p, int i, instr* is) {
      if (is->is_G0()) {
	move_instr* mi = static_cast<move_instr*>(is);
	return call_G0(p, i, mi);
      } else if (is->is_G1()) {
	move_instr* mi = static_cast<move_instr*>(is);
	return call_G1(p, i, mi);	
      } else if (is->is_g2_instr()) {
	g2_instr* mi = static_cast<g2_instr*>(is);
	return call_G2(p, i, mi);		
      } else if (is->is_g3_instr()) {
	g3_instr* mi = static_cast<g3_instr*>(is);
	return call_G3(p, i, mi);
      } else if (is->is_comment()) {
	comment* ci = static_cast<comment*>(is);
	return call_comment(p, i, ci);
      } else if (is->is_G90()) {
	return call_G90(p, i, is);
      } else if (is->is_M2()) {
	return call_M2(p, i, is);
      } else if (is->is_M5()) {
	return call_M5(p, i, is);
      } else if (is->is_M30()) {
	return call_M30(p, i, is);
      } else if (is->is_G53()) {
	move_instr* mi = static_cast<move_instr*>(is);
	return call_G53(p, i, mi);
      } else if (is->is_T()) {
	return call_T(p, i, is);
      } else if (is->is_S()) {
	return call_S(p, i, is);
      } else {
	cout << "Unsupported instruction: " << *is << endl;
	assert(false);
      }
    }
  };
 
}


#endif
