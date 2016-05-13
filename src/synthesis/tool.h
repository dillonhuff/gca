#ifndef GCA_TOOL_H
#define GCA_TOOL_H

namespace gca {

  enum tool_type { FLAT_NOSE, BALL_NOSE };

  class tool {
  protected:
    double diam;

  public:
    tool(double p_diameter, tool_type t) :
      diam(p_diameter) {}

    inline double radius() const { return diam / 2.0; }
    inline double diameter() const { return diam; }
  };

}

#endif
