#ifndef GCA_GCODE_PROGRAM_H
#define GCA_GCODE_PROGRAM_H

#include "gcode/lexer.h"

namespace gca {

  class gcode_program {
  public:
    std::string name;
    std::vector<block> blocks;

    gcode_program(const std::string& p_name,
		  const std::vector<block>& p_blocks) :
      name(p_name), blocks(p_blocks) {}
  };

}

#endif
