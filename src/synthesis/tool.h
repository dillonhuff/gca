#ifndef GCA_TOOL_H
#define GCA_TOOL_H

#include <cmath>
#include <iostream>

#include "synthesis/material.h"
#include "utils/check.h"

namespace gca {

  enum tool_type { FLAT_NOSE, BALL_NOSE };

  class tool {
  protected:
    double diam, len;
    unsigned flutes;
    material mat;
    int tool_num;
    double cut_len, shank_diam, shank_len, holder_diam, holder_len;

  public:
    tool(double p_diameter,
	 double p_length,
	 unsigned p_flutes,
	 material p_mat,
	 tool_type t) :
      diam(p_diameter), len(p_length), flutes(p_flutes), mat(p_mat), tool_num(-1) {
      DBG_ASSERT(mat == CARBIDE || mat == HSS);
    }

    tool(double p_diameter,
	 double p_length,
	 unsigned p_flutes,
	 material p_mat,
	 tool_type t,
	 const int p_tool_num) :
      diam(p_diameter), len(p_length), flutes(p_flutes), mat(p_mat), tool_num(p_tool_num) {
      DBG_ASSERT(mat == CARBIDE || mat == HSS);
    }
    
    inline double radius() const { return diam / 2.0; }
    inline double diameter() const { return diam; }
    inline double length() const { return len; }
    inline unsigned num_flutes() const { return flutes; }
    inline material material() const { return mat; }
    inline double cross_section_area() const
    { return M_PI*radius()*radius(); }
    inline int tool_number() const { return tool_num; }

    inline double cut_length() const { return cut_len; }
    inline double cut_diameter() const { return diam; }

    inline double shank_length() const { return shank_len; }
    inline double shank_diameter() const { return shank_diam; }

    inline double holder_length() const { return holder_len; }
    inline double holder_diameter() const { return holder_diam; }
    
    inline void set_cut_diameter(const double n)
    { diam = n; }

    inline void set_cut_length(const double n)
    { cut_len = n; }

    inline void set_shank_diameter(const double n)
    { shank_diam = n; }

    inline void set_shank_length(const double n)
    { shank_len = n; }

    inline void set_holder_diameter(const double n)
    { holder_diam = n; }

    inline void set_holder_length(const double n)
    { holder_len = n; }
    
  };

  std::ostream& operator<<(std::ostream& out, const tool& t);

  double chip_load_per_tooth(const tool& t,
			     const double feed_ipm,
			     const double rpm);
}

#endif
