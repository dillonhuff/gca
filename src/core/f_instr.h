#ifndef GCA_F_INSTR_H
#define GCA_F_INSTR_H

#include "core/instr.h"

namespace gca {

  class f_instr : public instr {
  public:
    int rate;
    string axes;
  f_instr(int val, string s) : rate(val) {
      axes = s;
    }

    inline bool is_f_instr() const { return true; }
    virtual bool operator==(const instr& i) const {
      if (i.is_f_instr()) {
	const f_instr& fi = static_cast<const f_instr&>(i);
	return rate == fi.rate && axes == fi.axes;
      }
      return false;
    }

    virtual void print(ostream& s) const {
      s << 'F' << rate << ' ' << axes;
    }
    
  };
}

#endif
