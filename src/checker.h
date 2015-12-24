#ifndef GCA_CHECKER_H
#define GCA_CHECKER_H

namespace gca {

  class checker {
  public:
    virtual bool check(ostream& s, gprog* i) const {
      assert(false);
    }
  };
  
}
#endif
