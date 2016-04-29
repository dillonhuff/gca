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
    for (auto surface : surfaces) {
      if (surface.contains(i)) { return true; }
    }
    return false;
  }

  void remove_sa_surfaces(const std::vector<surface>& surfaces,
			  std::vector<index_t>& indices) {
    delete_if(indices,
	      [&surfaces](index_t i)
	      { return any_sa_surface_contains(i, surfaces); });
  }

  std::vector<std::vector<index_t>>
  const_orientation_regions(const triangular_mesh& part) {
    auto faces = part.face_indexes();
    stable_sort(begin(faces), end(faces),
		[&part](index_t l, index_t r)
		{ return part.face_orientation(l).x < part.face_orientation(r).x; });
    stable_sort(begin(faces), end(faces),
		[&part](index_t l, index_t r)
		{ return part.face_orientation(l).y < part.face_orientation(r).y; });
    stable_sort(begin(faces), end(faces),
		[&part](index_t l, index_t r)
		{ return part.face_orientation(l).z < part.face_orientation(r).z; });
    vector<vector<index_t>> const_orient_face_indices;
    split_by(faces,
	     const_orient_face_indices,
	     [&part](index_t l, index_t r)
	     { return within_eps(part.face_orientation(l), part.face_orientation(r)); });
    return const_orient_face_indices;
  }

  double distance_along(point normal, const triangle t) {
    point p = t.v1;
    point dir = normal.normalize();
    return ((p.dot(dir))*dir).len();
  }

  std::vector<index_t> outermost_by(point normal,
				    const std::vector<index_t> faces,
				    const triangular_mesh& part,
				    double tolerance) {
    assert(faces.size() > 0);
    auto f = faces;
    sort(begin(f), end(f),
	 [&part, normal](index_t l, index_t r)
	 { return distance_along(normal, part.face_triangle(l)) >
	   distance_along(normal, part.face_triangle(r)); });
    double outer_dist = distance_along(normal, part.face_triangle(f.front()));
    take_while(f,
	       [&part, normal, outer_dist, tolerance](index_t i)
	       { return within_eps(outer_dist,
				   distance_along(normal, part.face_triangle(i)),
				   tolerance); });
    assert(f.size() > 0);
    return f;
  }

  std::vector<surface> part_stable_surfaces(const triangular_mesh& part) {
    auto const_orient_face_indices = const_orientation_regions(part);
    vector<vector<index_t>> stable_face_indices;
    for (auto f : const_orient_face_indices) {
      assert(f.size() > 0);
      point face_normal = part.face_orientation(f.front());
      vector<index_t> outer_face = outermost_by(face_normal, f, part, 0.001);
      stable_face_indices.push_back(outer_face);
    }
    vector<surface> surfaces;
    for (auto f : stable_face_indices) {
      surfaces.push_back(surface(&part, f));
    }
    return surfaces;
  }

  // TODO: Replace this dummy
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

  bool face_is_millable_from(index_t i,
			     const stock_orientation& orient,
			     const triangular_mesh& part_mesh) {
    return true;
  }

  std::vector<stock_orientation>
  all_stable_orientations(const std::vector<surface>& surfaces,
			  const triangular_mesh& part_mesh) {
    vector<stock_orientation> orients{stock_orientation()};
    return orients;
  }

  std::vector<stock_orientation>
  orientations_to_cut(const triangular_mesh& part_mesh,
		      const std::vector<surface>& surfaces,
		      std::vector<index_t>& faces_to_cut) {
    vector<stock_orientation> all_orients =
      all_stable_orientations(surfaces, part_mesh);
    vector<stock_orientation> orients;
    while (faces_to_cut.size() > 0) {
      assert(all_orients.size() > 0);
      auto next_orient = all_orients.front();
      delete_if(faces_to_cut,
		[&part_mesh, &next_orient](index_t i)
		{ return face_is_millable_from(i, next_orient, part_mesh); });
      orients.push_back(next_orient);
    }
    return orients;
  }

  std::vector<gcode_program> mesh_to_gcode(const triangular_mesh& part_mesh,
					   const vice v,
					   const vector<tool>& tools,
					   const workpiece_dimensions w_dims) {
    auto part_ss = part_stable_surfaces(part_mesh);
    auto workpiece_mesh = align_workpiece(part_ss, w_dims);
    classify_part_surfaces(part_ss, workpiece_mesh);
    vector<index_t> face_inds = part_mesh.face_indexes();
    cout << "# initial faces = " << face_inds.size() << endl;
    remove_sa_surfaces(part_ss, face_inds);
    vector<gcode_program> ps =
      workpiece_clipping_programs(workpiece_mesh, part_mesh);
    cout << "# faces left = " << face_inds.size() << endl;
    vector<stock_orientation> orients =
      orientations_to_cut(part_mesh, part_ss, face_inds);
    for (auto orient : orients) {
      ps.push_back(gcode_program());
    }
    return ps;
  }
}
