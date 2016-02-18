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
    bool operator==(const instr& other) const {
      return other.is_T() &&
      static_cast<const t_instr&>(other).num == num;
    }

  };

  class s_instr : public instr {
  public:
    int num;

  s_instr(int n) : num(n) {}

    inline bool is_S() const { return true; }
    void print(ostream& s) const { s << "S" << num; }
    bool operator==(const instr& other) const {
      return other.is_S() &&
      static_cast<const s_instr&>(other).num == num;
    }
  };
  
  class m5_instr : public instr {
  public:
    inline bool is_M5() const { return true; }
    void print(ostream& s) const { s << "M5"; }
    bool operator==(const instr& other) const { return other.is_M5(); }
  };

  class m2_instr : public instr {
  public:
    inline bool is_M2() const { return true; }
    void print(ostream& s) const { s << "M2"; }
    inline bool is_end_instr() const { return true; }
    bool operator==(const instr& other) const { return other.is_M2(); }
  };

  class m30_instr : public instr {
  public:
    inline bool is_M30() const { return true; }
    void print(ostream& s) const { s << "M30"; }
    inline bool is_end_instr() const { return true; }
    bool operator==(const instr& other) const { return other.is_M30(); }
  };

  class m3_instr : public instr {
  public:
    inline bool is_M3() const { return true; }
    void print(ostream& s) const { s << "M3"; }
    bool operator==(const instr& other) const { return other.is_M3(); }
  };

  class g20_instr : public instr {
  public:
    inline bool is_G20() const { return true; }
    void print(ostream& s) const { s << "G20"; }
    bool operator==(const instr& other) const { return other.is_G20(); }
  };

  class g21_instr : public instr {
  public:
    inline bool is_G21() const { return true; }
    void print(ostream& s) const { s << "G21"; }
    bool operator==(const instr& other) const { return other.is_G21(); }
  };

  class g64_instr : public instr {
  public:
    inline bool is_G64() const { return true; }
    void print(ostream& s) const { s << "G64"; }
    bool operator==(const instr& other) const { return other.is_G64(); }
  };
  
  class g90_instr : public instr {
  public:
    inline bool is_G90() const { return true; }
    void print(ostream& s) const { s << "G90"; }
    bool operator==(const instr& other) const { return other.is_G90(); }
  };

  class g91_instr : public instr {
  public:
    inline bool is_G91() const { return true; }
    void print(ostream& s) const { s << "G91"; }
    bool operator==(const instr& other) const { return other.is_G91(); }
  };
  
}

#endif
