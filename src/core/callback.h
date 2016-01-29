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

    virtual T call_G0(gprog* p, int i, g0_instr* is) {
      return call_default(p, i, is);
    }
    
    virtual T call_G1(gprog* p, int i, g1_instr* is) {
      return call_default(p, i, is);
    }

    virtual T call_G53(gprog* p, int i, g53_instr* is) {
      return call_default(p, i, is);
    }
    
    virtual T call_G2(gprog* p, int i, g2_instr* is) {
      return call_default(p, i, is);
    }
    
    virtual T call_G3(gprog* p, int i, g3_instr* is) {
      return call_default(p, i, is);
    }
    
    virtual T call_T(gprog* p, int i, t_instr* is) {
      return call_default(p, i, is);
    }

    virtual T call_S(gprog* p, int i, s_instr* is) {
      return call_default(p, i, is);
    }
    
    virtual T call_M2(gprog* p, int i, m2_instr* is) {
      return call_default(p, i, is);
    }

    virtual T call_M5(gprog* p, int i, m5_instr* is) {
      return call_default(p, i, is);
    }

    virtual T call_M30(gprog* p, int i, m30_instr* is) {
      return call_default(p, i, is);
    }

    virtual T call_M3(gprog* p, int i, m3_instr* is) {
      return call_default(p, i, is);
    }
    
    virtual T call_comment(gprog* p, int i, comment* is) {
      return call_default(p, i, is);
    }

    virtual T call_G90(gprog* p, int i, g90_instr* is) {
      return call_default(p, i, is);
    }

    virtual T call_default(gprog* p, int i, instr* is) = 0;

    virtual T call(gprog* p, int i, instr* is) {
      if (is->is_G0()) {
	g0_instr* mi = static_cast<g0_instr*>(is);
	return call_G0(p, i, mi);
      } else if (is->is_G1()) {
	g1_instr* mi = static_cast<g1_instr*>(is);
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
	g90_instr* mi = static_cast<g90_instr*>(is);
	return call_G90(p, i, mi);
      } else if (is->is_M2()) {
	m2_instr* mi = static_cast<m2_instr*>(is);
	return call_M2(p, i, mi);
      } else if (is->is_M3()) {
	m3_instr* mi = static_cast<m3_instr*>(is);
	return call_M3(p, i, mi);
      } else if (is->is_M5()) {
	m5_instr* mi = static_cast<m5_instr*>(is);
	return call_M5(p, i, mi);
      } else if (is->is_M30()) {
	m30_instr* mi = static_cast<m30_instr*>(is);
	return call_M30(p, i, mi);
      } else if (is->is_G53()) {
	g53_instr* mi = static_cast<g53_instr*>(is);
	return call_G53(p, i, mi);
      } else if (is->is_T()) {
	t_instr* mi = static_cast<t_instr*>(is);
	return call_T(p, i, mi);
      } else if (is->is_S()) {
	s_instr* mi = static_cast<s_instr*>(is);
	return call_S(p, i, mi);
      } else {
	cout << "Unsupported instruction: " << *is << endl;
	assert(false);
      }
    }
  };
 
}


#endif
