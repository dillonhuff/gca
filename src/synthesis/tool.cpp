#include "synthesis/tool.h"

namespace gca {

  std::ostream& operator<<(std::ostream& out, const tool& t) {
    out << "Tool Number         = " << t.tool_number() << std::endl;
    out << "Tool cut length     = " << t.cut_length() << std::endl;
    out << "Tool cut diameter   = " << t.cut_diameter() << std::endl;
    out << "Tool shank length   = " << t.shank_length() << std::endl;
    return out;
  }

  double chip_load_per_tooth(const tool& t,
			     const double feed_ipm,
			     const double rpm) {
    return feed_ipm / (rpm * t.num_flutes());
  }
  
  
}
