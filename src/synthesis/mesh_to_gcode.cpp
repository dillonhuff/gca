#include "synthesis/axis_3.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/workpiece_clipping.h"
#include "utils/algorithm.h"

namespace gca {

  ostream& operator<<(ostream& out, const workpiece& w) {
    out << "WORKPIECE" << endl;
    out << w.sides[0] << endl;
    out << w.sides[1] << endl;
    out << w.sides[2] << endl;
    return out;
  }

  std::vector<pocket> make_surface_pockets(const triangular_mesh& mesh,
					   std::vector<std::vector<index_t>>& surfaces) {
					   
    double h = max_in_dir(mesh, point(0, 0, 1));
    vector<pocket> pockets =
      make_surface_pockets(surfaces, mesh, h);
    return pockets;
  }

  gcode_program cut_secured_mesh(vector<pocket>& pockets,
  				 const std::vector<tool>& tools,
				 const material& stock_material) {
    assert(tools.size() > 0);

    double h = (*(max_element(begin(pockets), end(pockets),
			      [](const pocket& l, const pocket& r)
      { return l.get_start_depth() < r.get_start_depth(); }))).get_start_depth();

    tool t = *(min_element(begin(tools), end(tools),
  			   [](const tool& l, const tool& r)
      { return l.diameter() < r.diameter(); }));

    double cut_depth = 0.2;
    vector<polyline> lines = mill_pockets(pockets, t, cut_depth);

    double safe_z = h + t.length() + 0.1;
    return gcode_program("Surface cut", emco_f1_code(lines, safe_z, stock_material));
  }

  std::vector<gcode_program> mesh_to_gcode(const triangular_mesh& part_mesh,
					   const fixtures& f,
					   const vector<tool>& tools,
					   const workpiece w) {
    fabrication_plan plan = make_fabrication_plan(part_mesh, f, tools, w);
    vector<gcode_program> progs;
    for (auto f : plan.steps()) {
      progs.push_back(f.prog);
    }
    return progs;
  }

  fabrication_plan make_fabrication_plan(const triangular_mesh& part_mesh,
					 const fixtures& f,
					 const vector<tool>& tools,
					 const workpiece w) {
    fixture_plan plan = make_fixture_plan(part_mesh, f, tools, w);

    vector<fabrication_setup> setups;
    for (auto setup : plan.fixtures()) {
      gcode_program gprog =
	cut_secured_mesh(setup.pockets, tools, w.stock_material);
      setups.push_back(fabrication_setup(*(setup.m), setup.fix.v, gprog));
    }

    return fabrication_plan(setups);
  }

}
