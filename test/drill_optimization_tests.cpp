#include "catch.hpp"

#include <fstream>

#include "analysis/gcode_to_cuts.h"
#include "gcode/visual_debug.h"
#include "utils/algorithm.h"

using namespace std;

namespace gca {

  std::vector<cut*> extract_drill_downs(const std::vector<cut*>& drill_downs) {
    vector<cut*> drills = drill_downs;
    delete_if(drills, [](const cut* c) { return !c->is_linear_cut() || !is_vertical(c); });

    return drills;
  }

  double length(const std::vector<cut*>& cuts) {
    double len = 0;
    for (auto c : cuts) {
      len += c->length();
    }
    return len;
  }

  TEST_CASE("Counting needed drilling actions") {
    arena_allocator a;
    set_system_allocator(&a);
    
    string file_name = "/Users/dillon/CppWorkspace/gca/test/nc-files/strip=3,holes=.5,h=.2,len=60,drill.tap";

    std::ifstream t(file_name);
    std::string str((std::istreambuf_iterator<char>(t)),
		    std::istreambuf_iterator<char>());
    vector<block> p = lex_gprog(str);
    vector<vector<cut*>> paths;
    auto r = gcode_to_cuts(p, paths);

    cout << "# of paths = " << paths.size() << endl;

    int num_drills = 417;

    REQUIRE(paths.size() == 1);

    vtk_debug_cuts(paths[0]);

    double len = length(paths[0]);
    cout << "total length = " << len << endl;
    
    vector<cut*> drill_cuts =
      extract_drill_downs(paths[0]);

    cout << "drill length = " << length(drill_cuts) << endl;

    REQUIRE(drill_cuts.size() == num_drills);
  }

}
