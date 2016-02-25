#ifndef GCA_SAFE_MOVE_H
#define GCA_SAFE_MOVE_H

#include "synthesis/cut.h"

namespace gca {

  class safe_move : public cut {
  public:
  safe_move(point sp, point ep) :
    cut(sp, ep) {}

    static safe_move* make(point sp, point ep) {
      safe_move* mem = allocate<safe_move>();
      return new (mem) safe_move(sp, ep);
    }

    point final_orient() const {
      return end - start;
    }
    
    point initial_orient() const {
      return end - start;
    }

    bool operator==(const cut& other) const {
      if (other.is_safe_move()) {
	bool res = within_eps(start, other.start) && within_eps(end, other.end);
	return res;
      }
      return false;
    }

    cut* shift(point sh) const {
      safe_move* mem = allocate<safe_move>();
      safe_move* m = new (mem) safe_move(start + sh, end + sh);
      m->tool_no = tool_no;
      return m;
    }

    cut* scale(double s) const {
      safe_move* mem = allocate<safe_move>();
      safe_move* m = new (mem) safe_move(s*start, s*end);
      m->tool_no = tool_no;
      return m;
    }
    
    inline bool is_safe_move() const { return true; }
    
  };
  
}

#endif
