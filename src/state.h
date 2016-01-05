#ifndef GCA_STATE_H
#define GCA_STATE_H

namespace gca {
  
  class state {
  public:
    virtual void update() { assert(false); }
  };

}

#endif
