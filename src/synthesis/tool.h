#ifndef GCA_TOOL_H
#define GCA_TOOL_H

namespace gca {

  enum tool_type { FLAT_NOSE, BALL_NOSE };
  enum material { CARBIDE, HSS, ACETAL, ALUMINUM, BRASS };

  class tool {
  protected:
    double diam, len;
    unsigned flutes;
    material mat;

  public:
    tool(double p_diameter,
	 double p_length,
	 unsigned p_flutes,
	 material p_mat,
	 tool_type t) :
      diam(p_diameter), len(p_length), flutes(p_flutes), mat(p_mat) {
      assert(mat == CARBIDE || mat == HSS);
    }

    inline double radius() const { return diam / 2.0; }
    inline double diameter() const { return diam; }
    inline double length() const { return len; }
    inline unsigned num_flutes() const { return flutes; }
    inline material material() const { return mat; }
  };

}

#endif
