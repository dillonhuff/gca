#ifndef GCA_INSTR_MISC_H
#define GCA_INSTR_MISC_H

#include "core/instrs/instr.h"

namespace gca {

  class t_instr : public instr {
  public:
    int num;

  t_instr(int n) : num(n) {}

    inline bool is_T() const { return true; }
    void print(ostream& s) const { s << "T" << num; }
  };

  class m5_instr : public instr {
  public:
    inline bool is_M5() const { return true; }
    void print(ostream& s) const { s << "M5"; }
  };

  class m2_instr : public instr {
  public:
    inline bool is_M2() const { return true; }
    void print(ostream& s) const { s << "M2"; }
    inline bool is_end_instr() const { return true; }
  };

  class m30_instr : public instr {
  public:
    inline bool is_M30() const { return true; }
    void print(ostream& s) const { s << "M30"; }
    inline bool is_end_instr() const { return true; }
  };

  class m3_instr : public instr {
  public:
    inline bool is_M3() const { return true; }
    void print(ostream& s) const { s << "M3"; }
  };
  
  class g90_instr : public instr {
  public:
    inline bool is_G90() const { return true; }
    void print(ostream& s) const { s << "G90"; }
  };

  class g91_instr : public instr {
  public:
    inline bool is_G91() const { return true; }
    void print(ostream& s) const { s << "G91"; }
  };
  
}

#endif
