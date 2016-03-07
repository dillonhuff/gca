#ifndef GCA_INSTR_MISC_H
#define GCA_INSTR_MISC_H

#include "core/instrs/instr.h"

namespace gca {

  class t_instr : public instr {
  public:
    int num;

  t_instr(int n) : num(n) {}
    static t_instr* make(int n) { return new (allocate<t_instr>()) t_instr(n); }
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
    static s_instr* make(int n) { return new (allocate<s_instr>()) s_instr(n); }
    inline bool is_S() const { return true; }
    void print(ostream& s) const { s << "S" << num; }
    bool operator==(const instr& other) const {
      return other.is_S() &&
      static_cast<const s_instr&>(other).num == num;
    }
  };
  
  class m5_instr : public instr {
  public:
    static m5_instr* make() { return new (allocate<m5_instr>()) m5_instr(); }
    inline bool is_M5() const { return true; }
    void print(ostream& s) const { s << "M5"; }
    bool operator==(const instr& other) const { return other.is_M5(); }
  };

  class m2_instr : public instr {
  public:
    static m2_instr* make() { return new (allocate<m2_instr>()) m2_instr(); }
    inline bool is_M2() const { return true; }
    void print(ostream& s) const { s << "M2"; }
    inline bool is_end_instr() const { return true; }
    bool operator==(const instr& other) const { return other.is_M2(); }
  };

  class m3_instr : public instr {
  public:
    static m3_instr* make() { return new (allocate<m3_instr>()) m3_instr(); }
    inline bool is_M3() const { return true; }
    void print(ostream& s) const { s << "M3"; }
    bool operator==(const instr& other) const { return other.is_M3(); }
  };

  class m4_instr : public instr {
  public:
    static m4_instr* make() { return new (allocate<m4_instr>()) m4_instr(); }
    inline bool is_M4() const { return true; }
    void print(ostream& s) const { s << "M4"; }
    bool operator==(const instr& other) const { return other.is_M4(); }
  };

  class m7_instr : public instr {
  public:
    static m7_instr* make() { return new (allocate<m7_instr>()) m7_instr(); }
    inline bool is_M7() const { return true; }
    void print(ostream& s) const { s << "M7"; }
    bool operator==(const instr& other) const { return other.is_M7(); }
  };

  class m8_instr : public instr {
  public:
    static m8_instr* make() { return new (allocate<m8_instr>()) m8_instr(); }
    inline bool is_M8() const { return true; }
    void print(ostream& s) const { s << "M8"; }
    bool operator==(const instr& other) const { return other.is_M8(); }
  };

  class m9_instr : public instr {
  public:
    static m9_instr* make() { return new (allocate<m9_instr>()) m9_instr(); }
    inline bool is_M9() const { return true; }
    void print(ostream& s) const { s << "M9"; }
    bool operator==(const instr& other) const { return other.is_M9(); }
  };

  class m30_instr : public instr {
  public:
    static m30_instr* make() { return new (allocate<m30_instr>()) m30_instr(); }
    inline bool is_M30() const { return true; }
    void print(ostream& s) const { s << "M30"; }
    inline bool is_end_instr() const { return true; }
    bool operator==(const instr& other) const { return other.is_M30(); }
  };

  class g20_instr : public instr {
  public:
    static g20_instr* make() { return new (allocate<g20_instr>()) g20_instr(); }
    inline bool is_G20() const { return true; }
    void print(ostream& s) const { s << "G20"; }
    bool operator==(const instr& other) const { return other.is_G20(); }
  };

  class g21_instr : public instr {
  public:
    static g21_instr* make() { return new (allocate<g21_instr>()) g21_instr(); }
    inline bool is_G21() const { return true; }
    void print(ostream& s) const { s << "G21"; }
    bool operator==(const instr& other) const { return other.is_G21(); }
  };

  class g64_instr : public instr {
  public:
    value* p;
  g64_instr() : p(omitted::make()) {}
  g64_instr(value* pp) : p(pp) {}
    static g64_instr* make() { return new (allocate<g64_instr>()) g64_instr(); }
    static g64_instr* make(value* pp) { return new (allocate<g64_instr>()) g64_instr(pp); }
    inline bool is_G64() const { return true; }
    void print(ostream& s) const { s << "G64"; }
    bool operator==(const instr& other) const { return other.is_G64(); }
  };

  class g43_instr : public instr {
  public:
    value* p;
  g43_instr() : p(omitted::make()) {}
  g43_instr(value* pp) : p(pp) {}
    static g43_instr* make(value* pp) { return new (allocate<g43_instr>()) g43_instr(pp); }
    inline bool is_G43() const { return true; }
    void print(ostream& s) const { s << "G43"; }
    bool operator==(const instr& other) const { return other.is_G43(); }
  };
  
  class g90_instr : public instr {
  public:
    static g90_instr* make() { return new (allocate<g90_instr>()) g90_instr(); }
    inline bool is_G90() const { return true; }
    void print(ostream& s) const { s << "G90"; }
    bool operator==(const instr& other) const { return other.is_G90(); }
  };

  class g91_instr : public instr {
  public:
    static g91_instr* make() { return new (allocate<g91_instr>()) g91_instr(); }
    inline bool is_G91() const { return true; }
    void print(ostream& s) const { s << "G91"; }
    bool operator==(const instr& other) const { return other.is_G91(); }
  };
  
}

#endif
