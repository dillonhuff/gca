#ifndef GCA_GCODE_GENERATION_H
#define GCA_GCODE_GENERATION_H

#include "gcode/cut.h"
#include "gcode/gcode_program.h"
#include "geometry/polyline.h"
#include "synthesis/toolpath.h"

namespace gca {

  std::vector<cut*> polyline_cuts(const polyline& p);

  std::vector<block> emco_f1_code(const toolpath& pocket_lines);


  template<typename F>
  gcode_program
  build_gcode_program(std::string program_name,
		      const std::vector<toolpath>& toolpaths,
		      F f) {
    vector<block> blocks;
    for (auto t : toolpaths) {
      concat(blocks, f(t));
    }
    return gcode_program(program_name, blocks);
  }

}

#endif
