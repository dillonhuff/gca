#include "synthesis/tool.h"

namespace gca {

  std::ostream& operator<<(std::ostream& out, const tool& t) {
    out << "Tool Number         = " << t.tool_number() << std::endl;
    out << "Tool cut length     = " << t.cut_length() << std::endl;
    out << "Tool cut diameter   = " << t.cut_diameter() << std::endl;
    out << "Tool shank length   = " << t.shank_length() << std::endl;
    return out;
  }

}
