#include "geometry/vtk_debug.h"
#include "synthesis/axis_3.h"
#include "synthesis/contour_planning.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/gcode_generation.h"
#include "system/parse_stl.h"

using namespace gca;

int main(int argc, char* argv[]) {
  DBG_ASSERT(argc == 2);

  arena_allocator a;
  set_system_allocator(&a);

  std::string target_file = argv[1];
  
  auto mesh = parse_stl(target_file, 0.001);

  auto decomp = compute_contour_surfaces(mesh);

  DBG_ASSERT(decomp);
  DBG_ASSERT(decomp->rest.size() == 0);

  surface s = merge_surfaces(decomp->visible_from_minus_n);

  vector<index_t> si = s.index_list();
  subtract(si, merge_surfaces(decomp->visible_from_n).index_list());

  DBG_ASSERT(si.size() == 0);

  vtk_debug_highlight_inds(decomp->outline);

  vector<surface> sfs = decomp->visible_from_n;

  vector<pocket> pockets =
    make_surface_pockets(surfaces_to_inds(sfs),
			 mesh,
			 diameter(decomp->n, mesh));

  tool t1(0.1, 3.0, 4, HSS, FLAT_NOSE);
  vector<toolpath> toolpaths = mill_pockets(pockets, {t1}, ALUMINUM);

  gcode_program p =
    build_gcode_program(target_file, toolpaths, emco_f1_code);

  cout << p.name << endl;
  cout << p.blocks << endl;
}
