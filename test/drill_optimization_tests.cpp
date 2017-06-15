#include "catch.hpp"

#include <fstream>

#include "analysis/gcode_to_cuts.h"
#include "utils/algorithm.h"

using namespace std;

namespace gca {

  std::vector<cut*> extract_drill_downs(const std::vector<cut*>& drill_downs) {
    vector<cut*> drills = drill_downs;
    delete_if(drills, [](const cut* c) { return !c->is_linear_cut() || !is_vertical(c); });

    return drills;
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

    vector<cut*> drill_cuts =
      extract_drill_downs(paths[0]);

    REQUIRE(drill_cuts.size() == num_drills);
  }

}
