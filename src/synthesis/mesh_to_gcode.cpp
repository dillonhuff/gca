#include "geometry/matrix.h"
#include "synthesis/axis_3.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/millability.h"
#include "synthesis/toolpath_generation.h"
#include "system/algorithm.h"

namespace gca {

  ostream& operator<<(ostream& out, const workpiece& w) {
    out << "WORKPIECE" << endl;
    out << w.sides[0] << endl;
    out << w.sides[1] << endl;
    out << w.sides[2] << endl;
    return out;
  }

  void classify_part_surfaces(std::vector<surface>& part_surfaces,
			      const workpiece workpiece_mesh) {
    vector<point> normals;
    normals.push_back(point(1, 0, 0));
    normals.push_back(point(-1, 0, 0));
    normals.push_back(point(0, 1, 0));
    normals.push_back(point(0, -1, 0));
    normals.push_back(point(0, 0, 1));
    normals.push_back(point(0, 0, -1));

    for (auto& sf : part_surfaces) {
      for (auto n : normals) {
	if (within_eps(angle_between(n, sf.face_orientation(sf.front())), 0, 0.001)) {
	  sf.set_SA();
	}
      }
    }
  }

  // TODO: Change to actually align instead of just making surfaces
  // orthogonal to axes
  workpiece align_workpiece(const std::vector<surface>& part_surfaces,
			    const workpiece w) {
    return w;
  }

  bool any_SA_surface_contains(index_t i,
			       const std::vector<surface>& surfaces) {
    for (auto surface : surfaces) {
      if (surface.is_SA() && surface.contains(i)) { return true; }
    }
    return false;
  }

  void remove_SA_surfaces(const std::vector<surface>& surfaces,
			  std::vector<index_t>& indices) {
    delete_if(indices,
	      [&surfaces](index_t i)
	      { return any_SA_surface_contains(i, surfaces); });
  }

  std::vector<surface> outer_surfaces(const triangular_mesh& part) {
    auto const_orient_face_indices = const_orientation_regions(part);
    vector<surface> surfaces;
    for (auto f : const_orient_face_indices) {
      assert(f.size() > 0);
      if (is_outer_surface(f, part)) {
	surfaces.push_back(surface(&part, f));
      }
    }
    return surfaces;
  }

  workpiece clipped_workpiece(const workpiece aligned_workpiece,
			      const triangular_mesh& part_mesh) {
    point x_n = aligned_workpiece.sides[0].normalize();
    point y_n = aligned_workpiece.sides[1].normalize();
    point z_n = aligned_workpiece.sides[2].normalize();
    
    point x_d = diameter(aligned_workpiece.sides[0], part_mesh) * x_n;
    point y_d = diameter(aligned_workpiece.sides[1], part_mesh) * y_n;
    point z_d = diameter(aligned_workpiece.sides[2], part_mesh) * z_n;

    return workpiece(x_d, y_d, z_d);
  }

  std::vector<polyline> shift_lines_xy(const std::vector<polyline>& lines,
				       const vice v) {
    double x_f = v.x_max();
    double y_f = v.fixed_clamp_y();
    point shift(x_f - max_in_dir(lines, point(1, 0, 0)),
		y_f - max_in_dir(lines, point(0, 1, 0)),
		0);
    return shift_lines(lines, shift);
  }

  std::pair<std::vector<block>,
	    std::vector<block> >
  clip_axis(double workpiece_x,
	    double workpiece_y,
	    double workpiece_height,
	    double eps,
	    double part_height,
	    const tool& t,
	    double cut_depth,
	    const vice v) {
    assert(workpiece_height > part_height);
    double z_max = workpiece_height + 0.01;

    box b = box(0, workpiece_x,
		0, workpiece_y,
		z_max - eps, z_max);

    double safe_height = workpiece_height + t.length() + 0.1;

    vector<polyline> blk_lines = shift_lines(rough_box(b, t.radius(), cut_depth),
					     point(0, 0, t.length()));
    vector<block> blks =
      emco_f1_code(shift_lines_xy(blk_lines, v), safe_height);

    box b2 = box(0, workpiece_x,
		 0, workpiece_y,
		 part_height, z_max);
    vector<polyline> lines = shift_lines(rough_box(b2, t.radius(), cut_depth),
					 point(0, 0, t.length()));
    vector<block> clip_blocks =
      emco_f1_code(shift_lines_xy(lines, v), safe_height);
    return pair<vector<block>, vector<block> >(blks, clip_blocks);
  }

  void
  append_clip_programs(const string& axis,
		       const int axis_number,
		       const workpiece aligned_workpiece,
		       const workpiece clipped,
		       const double eps,
		       const tool& t,
		       const double cut_depth,
		       const vice v,
		       std::vector<gcode_program>& clip_progs) {

    double a1 = aligned_workpiece.sides[(axis_number + 1) % 3].len();
    double a2 = aligned_workpiece.sides[(axis_number + 2) % 3].len();

    double workpiece_x = max(a1, a2);
    double workpiece_y = min(a1, a2);

    double workpiece_height = aligned_workpiece.sides[axis_number].len() + v.base_z();
    double part_height = clipped.sides[axis_number].len() + v.base_z();

    auto clip_x = clip_axis(workpiece_x,
			    workpiece_y,
			    workpiece_height,
			    eps,
			    part_height,
			    t,
			    cut_depth,
			    v);

    gcode_program x_face(axis + "_Face", clip_x.first);
    gcode_program x_clip(axis + "_Clip", clip_x.second);
    clip_progs.push_back(x_face);
    clip_progs.push_back(x_clip);
  }

  // TODO: Clean up and add vice height test
  std::vector<gcode_program>
  workpiece_clipping_programs(const workpiece aligned_workpiece,
			      const triangular_mesh& part_mesh,
			      const std::vector<tool>& tools,
			      const vice v) {
    workpiece clipped = clipped_workpiece(aligned_workpiece, part_mesh);
    vector<gcode_program> clip_progs;

    // TODO: Turn these magic numbers into parameters
    double cut_depth = 0.2;
    double eps = 0.05;

    tool t = *(max_element(begin(tools), end(tools),
			   [](const tool& l, const tool& r)
      { return l.diameter() < r.diameter(); }));

    append_clip_programs("X", 0, aligned_workpiece, clipped, eps, t, cut_depth, v, clip_progs);
    append_clip_programs("Y", 1, aligned_workpiece, clipped, eps, t, cut_depth, v, clip_progs);
    append_clip_programs("Z", 2, aligned_workpiece, clipped, eps, t, cut_depth, v, clip_progs);

    return clip_progs;
  }

  bool orthogonal_flat_surfaces(const surface* l, const surface* r) {
    point l_orient = l->face_orientation(l->front());
    point r_orient = r->face_orientation(r->front());
    double theta = angle_between(l_orient, r_orient);
    return within_eps(theta, 90, 0.1);
  }

  bool parallel_flat_surfaces(const surface* l, const surface* r) {
    point l_orient = l->face_orientation(l->front());
    point r_orient = r->face_orientation(r->front());
    double theta = angle_between(l_orient, r_orient);
    return within_eps(theta, 180, 0.1);
  }
  
  std::vector<stock_orientation>
  all_stable_orientations(const std::vector<surface>& surfaces) {
    vector<stock_orientation> orients;
    for (unsigned j = 0; j < surfaces.size(); j++) {
      const surface* next_left = &(surfaces[j]);
      for (unsigned k = 0; k < surfaces.size(); k++) {
	const surface* next_right = &(surfaces[k]);
	if (parallel_flat_surfaces(next_right, next_left)) {
	  for (unsigned l = 0; l < surfaces.size(); l++) {
	    const surface* next_bottom = &(surfaces[l]);
	    if (orthogonal_flat_surfaces(next_bottom, next_left)) {
	      orients.push_back(stock_orientation(next_left,
						  next_right,
						  next_bottom));
	    }
	  }
	}
      }
    }
    assert(orients.size() > 0);
    return orients;
  }

  void remove_millable_surfaces(const stock_orientation& orient,
				std::vector<index_t>& faces_left) {
    std::vector<index_t> millable = millable_faces(orient.top_normal(), orient.get_mesh());
    sort(begin(millable), end(millable));
    delete_if(faces_left,
	      [&millable](const index_t i)
	      { return binary_search(begin(millable), end(millable), i); });
  }

  std::vector<stock_orientation>
  orientations_to_cut(const triangular_mesh& part_mesh,
		      const std::vector<surface>& surfaces,
		      std::vector<index_t>& faces_to_cut) {
    vector<stock_orientation> all_orients = all_stable_orientations(surfaces);
    sort(begin(all_orients), end(all_orients),
	 [](const stock_orientation l, const stock_orientation r)
	 { return l.top_normal().x < r.top_normal().x; });
    sort(begin(all_orients), end(all_orients),
	 [](const stock_orientation l, const stock_orientation r)
	 { return l.top_normal().z < r.top_normal().z; });
    vector<stock_orientation> orients;
    while (faces_to_cut.size() > 0) {
      assert(all_orients.size() > 0);
      auto next_orient = all_orients.back();
      all_orients.pop_back();
      unsigned old_size = faces_to_cut.size();
      remove_millable_surfaces(next_orient, faces_to_cut);
      if (faces_to_cut.size() != old_size) {
	cout << "Faces left = " << faces_to_cut.size() << endl;
	if (within_eps(point(0, -1, 0), next_orient.top_normal())) {
	  for (auto f : faces_to_cut) {
	    cout << part_mesh.face_triangle(f) << endl;
	  }
	}
	orients.push_back(next_orient);
      }
    }
    return orients;
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

  triangular_mesh oriented_part_mesh(const stock_orientation& orient,
				     const vice v) {
    auto mesh = orient.get_mesh();
    auto oriented_mesh = orient_mesh(mesh, orient);
    return shift_mesh(oriented_mesh, v);
  }

  gcode_program cut_secured_mesh(const triangular_mesh& mesh,
				 const vice v,
				 const std::vector<tool>& tools) {
    tool t = *(min_element(begin(tools), end(tools),
			   [](const tool& l, const tool& r)
      { return l.diameter() < r.diameter(); }));
    double cut_depth = 0.2;
    double h = max_in_dir(mesh, point(0, 0, 1));
    vector<polyline> lines = mill_surface_lines(mesh, t, cut_depth, h);
    double safe_z = h + t.length() + 0.1;
    return gcode_program("Surface cut", emco_f1_code(lines, safe_z));
  }

  void cut_secured_meshes(const std::vector<triangular_mesh>& meshes,
			  std::vector<gcode_program>& progs,
			  const vice v,
			  const std::vector<tool>& tools) {
    for (auto mesh : meshes) {
      progs.push_back(cut_secured_mesh(mesh, v, tools));
    }
  }

  std::vector<triangular_mesh>
  part_arrangements(const triangular_mesh& part_mesh,
		    const vector<surface>& part_ss,
		    const vice v) {
    vector<index_t> face_inds = part_mesh.face_indexes();
    cout << "# initial faces = " << face_inds.size() << endl;
    remove_SA_surfaces(part_ss, face_inds);
    cout << "# faces left = " << face_inds.size() << endl;
    vector<stock_orientation> orients =
      orientations_to_cut(part_mesh, part_ss, face_inds);
    vector<triangular_mesh> meshes;
    for (auto orient : orients) {
      cout << "Top normal " << orient.top_normal() << endl;
      meshes.push_back(oriented_part_mesh(orient, v));
    }
    return meshes;
  }

  std::vector<gcode_program> mesh_to_gcode(const triangular_mesh& part_mesh,
					   const vice v,
					   const vector<tool>& tools,
					   const workpiece w) {
    auto part_ss = outer_surfaces(part_mesh);
    auto aligned_workpiece = align_workpiece(part_ss, w);
    vector<gcode_program> ps =
      workpiece_clipping_programs(aligned_workpiece, part_mesh, tools, v);
    classify_part_surfaces(part_ss, aligned_workpiece);
    vector<triangular_mesh> meshes = part_arrangements(part_mesh, part_ss, v);
    cut_secured_meshes(meshes, ps, v, tools);
    return ps;
  }
}
