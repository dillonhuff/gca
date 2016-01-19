#ifndef GCA_MILL_TOOL_H
#define GCA_MILL_TOOL_H

namespace gca {

  typedef pair<int, int> column;

  class mill_tool {
  public:
    virtual void columns_to_update(point p,
				   double resolution,
				   vector<column> to_update) const { assert(false); }
  };

  class cylindrical_bit : public mill_tool {
  public:
    double diameter;
  cylindrical_bit(double d) : diameter(d) {}

    virtual void columns_to_update(point p,
				   double resolution,
				   vector<column> to_update) const;
  };
}

#endif
