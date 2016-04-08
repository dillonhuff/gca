#include "analysis/gcode_to_cuts.h"
#include "catch.hpp"
#include "core/lexer.h"
#include "simulators/sim_mill.h"
#include "synthesis/cut.h"
#include "synthesis/shapes_to_gcode.h"
#include "system/file.h"
#include "transformers/retarget.h"

namespace gca {

  int get_active_tool_no(const vector<cut*>& path) {
    auto c = *find_if(path.begin(), path.end(),
		      [](const cut* c) { return !c->is_safe_move(); });
    auto tn = c->settings.active_tool; //path.front()->settings.active_tool;
    if (!(tn->is_ilit())) {
      cout << "ERROR" << endl;
      cout << *c << endl;
      cout << "Active tool = " << *(c->settings.active_tool) << endl;
      assert(false);
    }
    auto tl = static_cast<ilit*>(tn);
    int current_tool_no = tl->v;
    cout << "current_tool_no = " << current_tool_no << endl;
    return current_tool_no;
  }

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

  TEST_CASE("HAAS VF1 to Emco F1 minimill single cut program") {
    arena_allocator a;
    set_system_allocator(&a);

    string dir_name = "/Users/dillon/CppWorkspace/gca/test/nc-files/fake_HAAS_VF1_program_one_cut.NCF";
    vector<block> p = lex_file(dir_name);
    cout << "Original program: " << endl;
    cout << p << endl;
    
    vector<vector<cut*>> paths;
    auto r = gcode_to_cuts(p, paths);
    assert(r == GCODE_TO_CUTS_SUCCESS);

    tool_table tt;
    tt[1] = tool_info(1.5, 0.25);

    auto res_paths = haas_to_minimill(paths, tt, tt);

    SECTION("One program per toolpath") {
      REQUIRE(res_paths.size() == paths.size());
    }

    SECTION("Material removed is the same") {
      double original_volume_removed = conservative_simulation(res_paths, tt);
      double retargeted_volume_removed = conservative_simulation(paths, tt);
      REQUIRE(within_eps(original_volume_removed, retargeted_volume_removed));
    }
  }  

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
