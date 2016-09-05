#ifndef GCA_GCODE_GENERATION_H
#define GCA_GCODE_GENERATION_H

#include "gcode/cut.h"
#include "gcode/gcode_program.h"
#include "geometry/polyline.h"
#include "synthesis/toolpath.h"
#include "utils/algorithm.h"

namespace gca {

  std::vector<cut*> polyline_cuts(const polyline& p);

  std::vector<block> emco_f1_code(const toolpath& pocket_lines);

  std::vector<block> emco_f1_code_G10_TLC(const toolpath& tp);

  template<typename ToolpathGCODE>
  gcode_program
  build_gcode_program(std::string program_name,
		      const std::vector<toolpath>& toolpaths,
		      ToolpathGCODE f) {
    vector<block> blocks;
    for (auto t : toolpaths) {
      concat(blocks, f(t));
    }
    token end('M', 2);
    vector<block> end_blks{{end}};
    concat(blocks, end_blks);
    return gcode_program(program_name, blocks);
  }

}

#endif
