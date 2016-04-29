#include <set>

#include "synthesis/mesh_to_gcode.h"
#include "system/algorithm.h"

namespace gca {

  void classify_part_surfaces(std::vector<surface>& part_surfaces,
			      const triangular_mesh& workpiece_mesh) {
  }

  // TODO: Change to actually align instead of just making surfaces
  // orthogonal to axes
  triangular_mesh align_workpiece(const std::vector<surface>& part_surfaces,
				  const workpiece_dimensions w_dims) {
    point p0(0, 0, 0);
    point p1(w_dims.x, 0, 0);
    point p2(w_dims.x, w_dims.y, 0);
    point p3(0, w_dims.y, 0);
    point p4(0, 0, w_dims.z);
    point p5(w_dims.x, 0, w_dims.z);
    point p6(w_dims.x, w_dims.y, w_dims.z);
    point p7(0, w_dims.y, w_dims.z);

    point n0(1, 0, 0);
    point n1(-1, 0, 0);

    point n2(0, 1, 0);
    point n3(0, -1, 0);

    point n4(0, 0, 1);
    point n5(0, 0, -1);

    vector<triangle> ts;
    ts.push_back(triangle(n4, p4, p7, p6));
    ts.push_back(triangle(n4, p5, p4, p6));
    ts.push_back(triangle(n0, p1, p5, p6));
    ts.push_back(triangle(n0, p5, p1, p4));
    return make_mesh(ts, 0.001);
  }

  bool any_sa_surface_contains(index_t i,
			       const std::vector<surface>& surfaces) {
    return true;
  }

  void remove_sa_surfaces(const std::vector<surface>& surfaces,
			  std::vector<index_t>& indices) {
    delete_if(indices,
	      [&surfaces](index_t i)
	      { return any_sa_surface_contains(i, surfaces); });
  }

  std::vector<surface> part_stable_surfaces(const triangular_mesh& m) {
    vector<surface> surfaces;
    return surfaces;
  }

  std::vector<gcode_program>
  workpiece_clipping_programs(const triangular_mesh& workpiece_mesh,
			      const triangular_mesh& part_mesh) {
    vector<gcode_program> clip_progs;
    for (int i = 0; i < 3; i++) {
      clip_progs.push_back(gcode_program());
      clip_progs.push_back(gcode_program());
    }
    return clip_progs;
  }

  std::vector<gcode_program> mesh_to_gcode(const triangular_mesh& part_mesh,
					   const vice v,
					   const vector<tool>& tools,
					   const workpiece_dimensions w_dims) {
    auto part_ss = part_stable_surfaces(part_mesh);
    auto workpiece_mesh = align_workpiece(part_ss, w_dims);
    classify_part_surfaces(part_ss, workpiece_mesh);
    vector<index_t> face_inds = part_mesh.face_indexes();
    remove_sa_surfaces(part_ss, face_inds);
    assert(face_inds.size() == 0);
    vector<gcode_program> ps =
      workpiece_clipping_programs(workpiece_mesh, part_mesh);
    return ps;
  }
}
