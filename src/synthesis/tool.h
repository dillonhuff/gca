#ifndef GCA_TOOL_H
#define GCA_TOOL_H

#include <cmath>

#include "synthesis/material.h"

namespace gca {

  enum tool_type { FLAT_NOSE, BALL_NOSE };

  class tool {
  protected:
    double diam, len;
    unsigned flutes;
    material mat;
    int tool_num;

  public:
    tool(double p_diameter,
	 double p_length,
	 unsigned p_flutes,
	 material p_mat,
	 tool_type t) :
      diam(p_diameter), len(p_length), flutes(p_flutes), mat(p_mat), tool_num(-1) {
      assert(mat == CARBIDE || mat == HSS);
    }

    tool(double p_diameter,
	 double p_length,
	 unsigned p_flutes,
	 material p_mat,
	 tool_type t,
	 const int p_tool_num) :
      diam(p_diameter), len(p_length), flutes(p_flutes), mat(p_mat), tool_num(p_tool_num) {
      assert(mat == CARBIDE || mat == HSS);
    }
    
    inline double radius() const { return diam / 2.0; }
    inline double diameter() const { return diam; }
    inline double length() const { return len; }
    inline unsigned num_flutes() const { return flutes; }
    inline material material() const { return mat; }
    inline double cross_section_area() const
    { return M_PI*radius()*radius(); }
    inline int tool_number() const { return tool_num; }
  };

}

#endif
