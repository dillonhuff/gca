#ifndef GCA_MILL_TOOL_H
#define GCA_MILL_TOOL_H

namespace gca {

  class mill_tool {};

  class cylindrical_bit : public mill_tool {
  public:
    double diameter;
  cylindrical_bit(double d) : diameter(d) {}
  };
}

#endif
