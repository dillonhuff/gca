#include "catch.hpp"
#include "synthesis/dxf_to_gcode.h"

namespace gca {

  unsigned num_cuts(const vector<cut_group>& cut_groups) {
    unsigned nc = 0;
    for (unsigned i = 0; i < cut_groups.size(); i++) {
      nc += cut_groups[i].size();
    }
    return nc;
  }

  TEST_CASE("Read rectangle file") {
    arena_allocator a;
    set_system_allocator(&a);
    
    string file_name = "/Users/dillon/CppWorkspace/gca/test/dxf-files/rect-2inx3in.DXF";
    shape_layout l = read_dxf(file_name.c_str());
    
    SECTION("All cuts parsed") {
      REQUIRE(l.lines.size() == 6);
    }

    SECTION("No cuts lost in cut group creation") {
      vector<cut_group> cgs;
      group_adjacent_cuts(l.lines, cgs, 30.0);
      unsigned nc = num_cuts(cgs);
      unsigned correct = l.lines.size();
      REQUIRE(nc == correct);
    }

    SECTION("Each rectangle cut is its own cut group") {
      vector<cut_group> cgs;
      group_adjacent_cuts(l.lines, cgs, 30.0);
      unsigned correct = l.lines.size();
      REQUIRE(cgs.size() == correct);
    }
  }
}
