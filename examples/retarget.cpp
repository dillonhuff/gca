// #include "analysis/gcode_to_cuts.h"
// #include "gcode/lexer.h"
// #include "gcode/cut.h"
// #include "synthesis/shapes_to_gcode.h"
// #include "system/file.h"
// #include "transformers/retarget.h"

// using namespace gca;
// using namespace std;

// void check_retargeting(vector<block>& res) {
//   vector<vector<cut*>> res_cuts;
//   auto r = gcode_to_cuts(res, res_cuts);
//   assert(r == GCODE_TO_CUTS_SUCCESS);
//   for (auto p : res_cuts) {
//     for (auto c : p) {
//       cout << *c << endl;
//       assert(c->settings.tool_radius_comp == TOOL_RADIUS_COMP_OFF);
//       assert(c->settings.tool_height_comp == TOOL_HEIGHT_COMP_OFF);
//     }
//   }
// }

// int main(int argc, char** argv) {
//   arena_allocator a;
//   set_system_allocator(&a);

//   string dir_name = "/Users/dillon/CppWorkspace/gca/test/nc-files/BottomALBottom2.NCF";
//   std::ifstream t(dir_name);
//   std::string str((std::istreambuf_iterator<char>(t)),
// 		  std::istreambuf_iterator<char>());
//   vector<block> p = lex_gprog(str);
//   vector<block> res = haas_to_minimill(p);
//   check_retargeting(res);
//   cout << res << endl;
// }
