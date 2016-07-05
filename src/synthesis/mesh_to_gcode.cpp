#include "geometry/matrix.h"
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
  				 const std::vector<tool>& tools) {
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
    return gcode_program("Surface cut", emco_f1_code(lines, safe_z));
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
    // TODO: Create actual mesh and vice for clipping
    // box b(0, 1, 0, 1, 0, 1);
    // triangular_mesh mesh = make_mesh(box_triangles(b), 0.001);
    // triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(mesh);
    for (auto p : workpiece_clipping_programs(plan.aligned_workpiece(), part_mesh, tools, f)) {
      gcode_program gprog = cut_secured_mesh(p.pockets, tools);
      setups.push_back(fabrication_setup(*(p.m), p.v, gprog));
    }

    for (auto setup : plan.fixtures()) {
      //      auto m = oriented_part_mesh(setup.fix.orient, v);
      gcode_program gprog = cut_secured_mesh(setup.pockets, tools);
      setups.push_back(fabrication_setup(*(setup.m), setup.v, gprog));
    }

    return fabrication_plan(setups);
  }

}
