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
  std::vector<block> emco_f1_code_no_TLC(const toolpath& pocket_lines);

  std::vector<block> camaster_engraving(const toolpath& last,
					const toolpath& current);
  std::vector<block> camaster_prefix_blocks(const toolpath& initial);
  std::vector<block> camaster_suffix_blocks();

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

  template<typename ToolpathGCODE,
	   typename InitialToolpath,
	   typename SuffixBlocks>
  gcode_program
  build_gcode_program(std::string program_name,
		      const std::vector<toolpath>& toolpaths,
		      InitialToolpath initial,
		      SuffixBlocks suffix,
		      ToolpathGCODE f) {
    if (toolpaths.size() == 0) { return gcode_program{program_name, {}}; }

    vector<block> blocks = initial(toolpaths.front());

    for (unsigned i = 1; i < toolpaths.size(); i++) {
      const toolpath& last = toolpaths[i - 1];
      const toolpath& current = toolpaths[i];
      concat(blocks, f(last, current));
    }

    concat(blocks, suffix());
    return gcode_program(program_name, blocks);
  }

}

#endif
