#include "analysis/gcode_to_cuts.h"
#include "catch.hpp"
#include "core/lexer.h"
#include "synthesis/cut.h"
#include "synthesis/shapes_to_gcode.h"
#include "system/file.h"
#include "transformers/retarget.h"

namespace gca {

  TEST_CASE("HAAS VF1 to Emco F1 minimill") {
    arena_allocator a;
    set_system_allocator(&a);

    string dir_name = "/Users/dillon/CppWorkspace/gca/test/nc-files/BottomALBottom2.NCF";
    vector<block> p = lex_file(dir_name);
    vector<vector<cut*>> paths;
    
    SECTION("One program per toolpath") {
      auto res = haas_to_minimill(p);
      auto r = gcode_to_cuts(p, paths);
      assert(r == GCODE_TO_CUTS_SUCCESS);
      assert(res.size() == paths.size());
    }
  }
}
