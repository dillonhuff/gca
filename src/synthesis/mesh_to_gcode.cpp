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
  
  triangular_mesh orient_mesh(const triangular_mesh& mesh,
			      const stock_orientation& orient) {
    point normal = orient.top_normal();
    matrix<3, 3> top_rotation_mat = rotate_onto(normal, point(0, 0, 1));
    auto m = top_rotation_mat * mesh;
    return m;
  }

  triangular_mesh shift_mesh(const triangular_mesh& mesh,
			     const vice v) {
    double x_f = v.x_max();
    double y_f = v.fixed_clamp_y();
    double z_f = v.base_z();
    point shift(x_f - max_in_dir(mesh, point(1, 0, 0)),
		y_f - max_in_dir(mesh, point(0, 1, 0)),
		z_f - min_in_dir(mesh, point(0, 0, 1)));
    auto m = mesh.apply_to_vertices([shift](const point p)
		      { return p + point(shift.x, shift.y, shift.z); });
    return m;
  }

  std::pair<triangular_mesh, std::vector<std::vector<index_t>>>
  oriented_part_mesh(const stock_orientation& orient,
		     const surface_list& surfaces,
		     const vice v) {
    auto mesh = orient.get_mesh();
    auto oriented_mesh = orient_mesh(mesh, orient);
    return mk_pair(shift_mesh(oriented_mesh, v), surfaces);
  }

  std::vector<std::pair<triangular_mesh, surface_list>>
  part_arrangements(const triangular_mesh& part_mesh,
  		    const vector<surface>& part_ss,
  		    const vice v) {
    fixtures fixes(v);
    fixture_list orients =
      orientations_to_cut(part_mesh, part_ss, fixes);
    vector<pair<triangular_mesh, surface_list>> meshes;
    for (auto orient_surfaces_pair : orients) {
      cout << "Top normal " << orient_surfaces_pair.first.orient.top_normal() << endl;
      meshes.push_back(oriented_part_mesh(orient_surfaces_pair.first.orient,
  					  orient_surfaces_pair.second,
  					  orient_surfaces_pair.first.v));
    }
    return meshes;
  }

  gcode_program cut_secured_mesh(const std::pair<triangular_mesh,
				 std::vector<std::vector<index_t>>>& mesh_surfaces_pair,
				 const std::vector<tool>& tools) {
    assert(tools.size() > 0);
    tool t = *(min_element(begin(tools), end(tools),
			   [](const tool& l, const tool& r)
      { return l.diameter() < r.diameter(); }));
    double cut_depth = 0.2;
    double h = max_in_dir(mesh_surfaces_pair.first, point(0, 0, 1));
    vector<polyline> lines = mill_surfaces(mesh_surfaces_pair.second,
					   mesh_surfaces_pair.first,
					   t,
					   cut_depth,
					   h);
    double safe_z = h + t.length() + 0.1;
    return gcode_program("Surface cut", emco_f1_code(lines, safe_z));
  }

  // TODO: Eliminate vice v parameter
  void cut_secured_meshes(const std::vector<std::pair<triangular_mesh, surface_list>>& meshes,
			  std::vector<gcode_program>& progs,
			  const std::vector<tool>& tools) {
    for (auto mesh_surface_pair : meshes) {
      progs.push_back(cut_secured_mesh(mesh_surface_pair, tools));
    }
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
    auto part_ss = outer_surfaces(part_mesh);
    fixture_plan plan = make_fixture_plan(part_mesh, part_ss, f, tools, w);
    vector<fabrication_setup> setups;
    // TODO: Create actual mesh and vice for clipping
    box b(0, 1, 0, 1, 0, 1);
    triangular_mesh m = make_mesh(box_triangles(b), 0.001);
    for (auto p : workpiece_clipping_programs(plan.aligned_workpiece(), part_mesh, tools, f)) {
      setups.push_back(fabrication_setup(m, f.get_vice(), p));
    }

    for (auto orient_surfaces_pair : plan.fixtures()) {
      cout << "Top normal " << orient_surfaces_pair.first.orient.top_normal() << endl;
      auto v = orient_surfaces_pair.first.v;
      auto m = oriented_part_mesh(orient_surfaces_pair.first.orient,
				  orient_surfaces_pair.second,
				  v);
      gcode_program gprog = cut_secured_mesh(m, tools);
      setups.push_back(fabrication_setup(m.first, v, gprog));
    }

    return fabrication_plan(setups);
  }


}
