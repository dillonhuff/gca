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

  double distance_along(point normal, const triangle t) {
    point p = t.v1;
    point dir = normal.normalize();
    return ((p.dot(dir))*dir).len();
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

  double diameter(const point normal, const triangular_mesh& m) {
    return diameter(normal, m.vertex_list());
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

  double min_in_dir(const std::vector<polyline>& lines, const point dir) {
    return min_distance_along(points(lines), dir);
  }

  double max_in_dir(const std::vector<polyline>& lines, const point dir) {
    return max_distance_along(points(lines), dir);
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
  clip_axis(double workpiece_width,
	    double workpiece_length,
	    double workpiece_height,
	    double eps,
	    double part_height,
	    double tool_radius,
	    double cut_depth,
	    const vice v) {
    assert(workpiece_height > part_height);
    double z_max = workpiece_height + 0.01;

    box b = box(0, workpiece_width,
		0, workpiece_length,
		z_max - eps, z_max);

    vector<polyline> blk_lines = rough_box(b, tool_radius, cut_depth);
    vector<block> blks =
      emco_f1_code(shift_lines_xy(blk_lines, v), workpiece_height + 0.1);

    box b2 = box(0, workpiece_width,
		 0, workpiece_length,
		 part_height, z_max);
    vector<polyline> lines = rough_box(b2, tool_radius, cut_depth);
    vector<block> clip_blocks =
      emco_f1_code(shift_lines_xy(lines, v), workpiece_height + 0.1);
    return pair<vector<block>, vector<block> >(blks, clip_blocks);
  }

  // TODO: Clean up and add vice height test
  std::vector<gcode_program>
  workpiece_clipping_programs(const workpiece aligned_workpiece,
			      const triangular_mesh& part_mesh,
			      const std::vector<tool>& tools,
			      const vice v) {
    workpiece clipped = clipped_workpiece(aligned_workpiece, part_mesh);
    vector<gcode_program> clip_progs;

    double tool_radius = tools.front().radius();

    cout << "Workpiece: " << aligned_workpiece << endl;
    cout << "Clipped: " << clipped << endl;

    // TODO: Turn these magic numbers into parameters
    double cut_depth = 0.15;
    double eps = 0.05;

    auto s = aligned_workpiece.sides;
    const point m_e = *(max_element(s, s + 3,
		  [](const point& l, const point& r)
      { return l.len() < r.len(); }));

    double l = m_e.len();
    double workpiece_width = l;//aligned_workpiece.sides[1].len();
    double workpiece_length = l; //aligned_workpiece.sides[2].len();

    double workpiece_height = aligned_workpiece.sides[0].len() + v.base_z();
    double part_height = clipped.sides[0].len() + v.base_z();

    auto clip_x = clip_axis(workpiece_width,
			    workpiece_length,
			    workpiece_height,
			    eps,
			    part_height,
			    tool_radius,
			    cut_depth,
			    v);

    gcode_program x_face("X_Face", clip_x.first);
    gcode_program x_clip("X_Clip", clip_x.second);
    clip_progs.push_back(x_face);
    clip_progs.push_back(x_clip);

    workpiece_height = aligned_workpiece.sides[1].len() + v.base_z();
    part_height = clipped.sides[1].len() + v.base_z();
    
    auto clip_y = clip_axis(workpiece_width,
			    workpiece_length,
			    workpiece_height,
			    eps,
			    part_height,
			    tool_radius,
			    cut_depth,
			    v);
    
    gcode_program y_face("Y_Face", clip_y.first);
    gcode_program y_clip("Y_Clip", clip_y.second);
    clip_progs.push_back(y_face);
    clip_progs.push_back(y_clip);

    workpiece_height = aligned_workpiece.sides[2].len() + v.base_z();
    part_height = clipped.sides[2].len() + v.base_z();
    
    auto clip_z = clip_axis(workpiece_width,
			    workpiece_length,
			    workpiece_height,
			    eps,
			    part_height,
			    tool_radius,
			    cut_depth,
			    v);
    
    gcode_program z_face("Z_Face", clip_z.first);
    gcode_program z_clip("Z_Clip", clip_z.second);
    clip_progs.push_back(z_face);
    clip_progs.push_back(z_clip);

    return clip_progs;
  }

  bool face_is_millable_from(index_t i,
			     const point top_normal,
			     const triangular_mesh& part_mesh) {
    point i_normal = part_mesh.face_orientation(i);
    return within_eps(angle_between(top_normal, i_normal), 0, 90);
  }

  bool face_is_millable_from(index_t i,
			     const stock_orientation& orient,
			     const triangular_mesh& part_mesh) {
    point top_normal = orient.top_normal();
    point i_normal = part_mesh.face_orientation(i);
    return within_eps(angle_between(top_normal, i_normal), 0, 90);
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

  double max_in_dir(const triangular_mesh& mesh,
		    const point dir) {
    return max_distance_along(mesh.vertex_list(), dir);
  }

  double min_in_dir(const triangular_mesh& mesh,
		    const point dir) {
    return min_distance_along(mesh.vertex_list(), dir);
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
    std::vector<index_t> millable = millable_faces(point(0, 0, 1), mesh);
    std::vector<triangle> tris;
    for (auto i : millable) {
      tris.push_back(mesh.face_triangle(i));
    }
    double tool_diameter = tools.front().diameter();
    vector<polyline> lines = drop_sample(tris, tool_diameter / 2.0);
    double safe_z = max_in_dir(mesh, point(0, 0, 1));
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

  std::vector<triangular_mesh>
  part_arrangements(const triangular_mesh& part_mesh,
		    const vice v) {
    auto part_ss = outer_surfaces(part_mesh);
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
