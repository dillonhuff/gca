#ifndef GCA_INSTR_H
#define GCA_INSTR_H

#include <cassert>
#include <iostream>

#include "geometry/point.h"

using namespace std;

namespace gca {

  enum orientation {
    GCA_ABSOLUTE = 0,
    GCA_RELATIVE = 1,
    GCA_NONE
  };

  class instr {
  public:
    virtual instr* copy() const { assert(false); }

    virtual inline bool is_g3_instr() const { return false; }
    virtual inline bool is_g2_instr() const { return false; }
    virtual inline bool is_f_instr() const { return false; }
    virtual inline bool is_comment() const { return false; }
    virtual inline bool is_assign_instr() const { return false; }
    virtual inline bool is_move_instr() const { return false; }
    virtual inline bool is_end_instr() const { return false; }
    
    virtual bool operator==(const instr& other) const = 0;

    bool operator!=(const instr& other) {
      return !(*this == other);
    }

    virtual void print(ostream& s) const = 0;
    virtual void print_nc_output(ostream& s) const {
      print(s);
    }

    virtual inline bool is_G() const { return is_G0() || is_G1() || is_G2() || is_G53() || is_G90() || is_G91(); }
    virtual inline bool is_M() const { return is_M3() || is_M5() || is_M2() || is_M30(); }

    virtual inline bool is_M2() const { return false; }    
    virtual inline bool is_M3() const { return false; }
    virtual inline bool is_M4() const { return false; }
    virtual inline bool is_M5() const { return false; }
    virtual inline bool is_M7() const { return false; }
    virtual inline bool is_M8() const { return false; }
    virtual inline bool is_M9() const { return false; }
    virtual inline bool is_M30() const { return false; }    
    virtual inline bool is_F() const { return false; }
    virtual inline bool is_S() const { return false; }
    virtual inline bool is_T() const { return false; }
    virtual inline bool is_G0() const { return false; }
    virtual inline bool is_G1() const { return false; }
    virtual inline bool is_G2() const { return false; }
    virtual inline bool is_G3() const { return false; }
    virtual inline bool is_G18() const { return false; }    
    virtual inline bool is_G20() const { return false; }
    virtual inline bool is_G21() const { return false; }
    virtual inline bool is_G43() const { return false; }    
    virtual inline bool is_G53() const { return false; }    
    virtual inline bool is_G64() const { return false; }
    virtual inline bool is_G91() const { return false; }
    virtual inline bool is_G90() const { return false; }

  };

  ostream& operator<<(ostream& stream, const instr& i);

}

#endif
