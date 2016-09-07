#include "analysis/gcode_to_cuts.h"
#include "catch.hpp"
#include "gcode/lexer.h"
#include "simulators/sim_mill.h"
#include "gcode/cut.h"
#include "synthesis/shapes_to_gcode.h"
#include "system/file.h"
#include "transformers/retarget.h"

namespace gca {

  double conservative_simulation(const vector<vector<cut*>>& paths,
				 tool_table& tt) {
    double volume_removed = 0.0;
    double max_tool_diameter = 1.5;
    auto r = set_up_region_conservative(paths, max_tool_diameter);
    for (auto path : paths) {
      int tn = get_active_tool_no(path);
      auto ti = tt[tn];
      double tool_diameter = ti.diameter;
      cylindrical_bit t = (tool_diameter);
      for (auto c : path) {
	volume_removed += update_cut(*c, r, t);
      }
    }
    return volume_removed;
  }

  // TEST_CASE("HAAS VF1 to Emco F1 minimill single cut program") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   string dir_name = "/Users/dillon/CppWorkspace/gca/test/nc-files/fake_HAAS_VF1_program_one_cut.NCF";
  //   vector<block> p = lex_file(dir_name);
    
  //   vector<vector<cut*>> paths;
  //   auto r = gcode_to_cuts(p, paths);
  //   assert(r == GCODE_TO_CUTS_SUCCESS);

  //   tool_table tt;
  //   tt[1] = tool_info(1.5, 0.25);

  //   auto res_progs = haas_to_minimill(p, tt, tt);
  //   auto res_paths = haas_to_minimill(paths, tt, tt);

  //   SECTION("One program per toolpath") {
  //     REQUIRE(res_paths.size() == paths.size());
  //   }

  //   SECTION("Material removed is the same") {
  //     double original_volume_removed = conservative_simulation(res_paths, tt);
  //     double retargeted_volume_removed = conservative_simulation(paths, tt);
  //     REQUIRE(within_eps(original_volume_removed, retargeted_volume_removed));
  //   }

  //   SECTION("Z values adjusted to for tool length") {
  //     box original = bound_paths(paths);
  //     box retargeted = bound_paths(res_paths);
  //     REQUIRE(within_eps(retargeted.z_min, original.z_min + 1.5));
  //   }

  //   SECTION("Tool height comp setting is turned off") {
  //     bool height_comp_off = true;
  //     for (auto path : res_paths) {
  // 	for (auto c : path) {
  // 	  if (c->settings.tool_height_comp != TOOL_HEIGHT_COMP_OFF)
  // 	    { height_comp_off = false; }
  // 	}
  //     }
  //     REQUIRE(height_comp_off);
  //   }

  //   SECTION("Resulting GCODE retains old properties") {
  //     vector<vector<cut*>> output_cuts;
  //     for (auto prog : res_progs) {
  //   	vector<vector<cut*>> pcuts;
  //   	r = gcode_to_cuts(prog, pcuts);
  //   	output_cuts.insert(output_cuts.end(), pcuts.begin(), pcuts.end());
  //     }

  //     REQUIRE(output_cuts.size() == 1);

  //     SECTION("Spindle speed saved") {
  //   	auto spindle_speed = get_spindle_speed(output_cuts.front());
  //   	REQUIRE(spindle_speed == 1500);
  //     }
  //   }
  // }  

  // TEST_CASE("HAAS VF1 to Emco F1 minimill small program") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   string dir_name = "/Users/dillon/CppWorkspace/gca/test/nc-files/fake_HAAS_VF1_program.NCF";
  //   vector<block> p = lex_file(dir_name);
  //   vector<vector<cut*>> paths;
    
  //   SECTION("One program per toolpath") {
  //     auto res = haas_to_minimill(p);
  //     auto r = gcode_to_cuts(p, paths);
  //     assert(r == GCODE_TO_CUTS_SUCCESS);
  //     assert(res.size() == paths.size());
  //   }

  //   SECTION("") {
  //   }
  // }
  
  // TEST_CASE("HAAS VF1 to Emco F1 minimill ") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   string dir_name = "/Users/dillon/CppWorkspace/gca/test/nc-files/BottomALBottom2.NCF";
  //   vector<block> p = lex_file(dir_name);
  //   vector<vector<cut*>> paths;
    
  //   SECTION("One program per toolpath") {
  //     auto res = haas_to_minimill(p);
  //     auto r = gcode_to_cuts(p, paths);
  //     assert(r == GCODE_TO_CUTS_SUCCESS);
  //     assert(res.size() == paths.size());
  //   }

  //   SECTION("") {
  //   }
  // }
}
