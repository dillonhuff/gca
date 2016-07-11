#include "gcode/gcode_program.h"
#include "gcode/lexer.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/tool.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/vice.h"
#include "synthesis/workpiece_clipping.h"

namespace gca {

  box workpiece_box(const workpiece& w) {
    double x_len = w.sides[0].len();
    double y_len = w.sides[1].len();
    double z_len = w.sides[2].len();
    return box(0, x_len, 0, y_len, 0, z_len);
  }

  // TODO: Dont just make a box
  triangular_mesh stock_mesh(const workpiece& w) {
    box b = workpiece_box(w);
    auto tris = box_triangles(b);
    auto m = make_mesh(tris, 0.001);
    assert(m.is_connected());
    return m;
  }

  void remove_contained_surfaces(const std::vector<surface>& stable_surfaces,
				 std::vector<surface>& surfaces_to_cut) {
    vector<index_t> stable_surface_inds;
    for (auto s : stable_surfaces) {
      concat(stable_surface_inds, s.index_list());
    }
    sort(begin(stable_surface_inds), end(stable_surface_inds));

    delete_if(surfaces_to_cut,
	      [&stable_surface_inds](const surface& s)
	      { return s.contained_by_sorted(stable_surface_inds); });
  }

  void remove_clipped_surfaces(const std::vector<surface>& stable_surfaces,
			       std::vector<surface>& surfaces_to_cut) {
    vector<surface> sa_surfs;
    for (auto s : stable_surfaces) {
      if (s.is_SA()) {
	sa_surfs.push_back(s);
      }
    }
    remove_contained_surfaces(sa_surfs, surfaces_to_cut);
  }
  
  workpiece clipped_workpiece(const workpiece aligned_workpiece,
			      const triangular_mesh& part_mesh) {
    point x_n = aligned_workpiece.sides[0].normalize();
    point y_n = aligned_workpiece.sides[1].normalize();
    point z_n = aligned_workpiece.sides[2].normalize();
    
    point x_d = diameter(aligned_workpiece.sides[0], part_mesh) * x_n;
    point y_d = diameter(aligned_workpiece.sides[1], part_mesh) * y_n;
    point z_d = diameter(aligned_workpiece.sides[2], part_mesh) * z_n;

    return workpiece(x_d, y_d, z_d, aligned_workpiece.stock_material);
  }

  std::vector<polyline> shift_lines_xy(const std::vector<polyline>& lines,
				       const vice v) {
    if (lines.size() == 0) {
      return lines;
    }
    double x_f = v.x_max();
    double y_f = v.fixed_clamp_y();
    point shift(x_f - max_in_dir(lines, point(1, 0, 0)),
		y_f - max_in_dir(lines, point(0, 1, 0)),
		0);
    return shift_lines(lines, shift);
  }

  // TODO: Correctly place boxes in space relative to the vices
  std::pair<fixture_setup, fixture_setup>
  clip_axis(double workpiece_x,
	    double workpiece_y,
	    double workpiece_height,
	    double eps,
	    double part_height,
	    const vice v) {
    assert(workpiece_height > part_height);

    double z_max = workpiece_height + 0.01;

    // TODO: Use actual workpiece mesh
    box bx(0, 1, 0, 1, z_max - 5.0, z_max);
    triangular_mesh mesh = make_mesh(box_triangles(bx), 0.001);
    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(mesh);
    
    box b = box(0, workpiece_x,
		0, workpiece_y,
		z_max - eps, z_max);

    pocket p = box_pocket(b);
    vector<pocket> ps{p};

    box b2 = box(0, workpiece_x,
		 0, workpiece_y,
		 part_height, z_max);
    pocket q = box_pocket(b2);
    vector<pocket> qs{q};

    return std::make_pair(fixture_setup(m, v, ps), fixture_setup(m, v, qs));
  }

  void
  append_clip_setups(const string& axis,
		       const int axis_number,
		       const workpiece aligned_workpiece,
		       const workpiece clipped,
		       const double eps,
		       const vice v,
		       std::vector<fixture_setup>& clip_progs) {
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
    			    v);

    clip_progs.push_back(clip_x.first);
    clip_progs.push_back(clip_x.second);
  }

  // TODO: Need to add normal vectors, how to match this with
  // the code in make_fixture_plan?
  oriented_polygon
  part_outline(std::vector<surface>* surfaces_to_cut) {
    point n(0, 0, 1);

    vector<surface> vertical_surfs =
      select(*surfaces_to_cut,
    	     [n](const surface& s)
    	     { return s.orthogonal_to(n, 0.01); });

    vector<vector<unsigned>> merge_groups =
      connected_components_by(vertical_surfs, [](const surface& l, const surface& r)
    			      { return surfaces_share_edge(l, r); });

    // TODO: Add tests of exterior / interior, etc
    if (merge_groups.size() == 1) {
      surface m = merge_surfaces(vertical_surfs);
      vector<oriented_polygon> outlines =
	mesh_bounds(m.index_list(), m.get_parent_mesh());
      if (outlines.size() == 2) {
	remove_contained_surfaces(vertical_surfs, *surfaces_to_cut);
	return outlines.front();
      }
    }

    oriented_polygon empty(point(0, 0, 1), {});
    return empty;
  }

  fixture_setup
  clip_top_and_sides(const workpiece& aligned,
		     const point clipped_dims,
		     const oriented_polygon& outline,
		     const vice& v,
		     const double plate_height) {
    double aligned_x = aligned.sides[0].len();
    double clipped_x = clipped_dims.x;

    double aligned_y = aligned.sides[1].len();
    double clipped_y = clipped_dims.y;

    double aligned_z_height = aligned.sides[2].len();
    double clipped_z_height = clipped_dims.z;

    double adjusted_jaw_height = v.jaw_height() - plate_height;
    assert(adjusted_jaw_height > 0);
    double leftover = aligned_z_height - clipped_z_height - adjusted_jaw_height;
    double alpha = leftover / 2.0;

    double z_max = v.base_z() + plate_height + aligned_z_height;
    double z_min = z_max - alpha;

    // Top facing operation
    box b1 = box(v.x_max() - aligned.sides[0].len(), v.x_max(),
    		 v.y_max() - aligned.sides[1].len(), v.y_max(),
    		 z_min, z_max);

    vector<pocket> pockets;
    pockets.push_back(box_pocket(b1));

    oriented_polygon exterior = base(b1);

    z_max = z_min;
    z_min = z_min - clipped_z_height;

    double x1 = v.x_max();
    double x2 = x1 - (aligned_x - clipped_x) / 2.0;
    double x3 = x2 - clipped_x;

    double y1 = v.y_max();
    double y2 = y1 - (aligned_y - clipped_y) / 2.0;
    double y3 = y2 - clipped_y;

    box b2 = box(x3, x2, y3, y2, z_min, z_max);

    if (outline.vertices().size() > 0) {
      pockets.push_back(contour_pocket(b2.z_max, b2.z_min, outline, exterior));
    } else {
      oriented_polygon interior = base(b2);
      pockets.push_back(contour_pocket(b2.z_max, b2.z_min, interior, exterior));
    }

    // TODO: Use actual workpiece mesh
    box bx(0, 1, 0, 1, z_max - 3.0, z_max);
    triangular_mesh mesh = make_mesh(box_triangles(bx), 0.001);
    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(mesh);
    
    return fixture_setup(m, v, pockets);
  }

  fixture_setup
  clip_base(const workpiece& aligned,
	    const double clipped_z_height,
	    const vice& v,
	    const double plate_height) {
    double aligned_z_height = aligned.sides[2].len();

    double adjusted_jaw_height = v.jaw_height() - plate_height;
    assert(adjusted_jaw_height > 0);
    double leftover = aligned_z_height - clipped_z_height - adjusted_jaw_height;
    assert(leftover > 0);
    double alpha = leftover / 2.0;

    // box b = box(v.x_max() - aligned.sides[0].len(), v.x_max(),
    // 		v.y_max() - aligned.sides[1].len(), v.y_max(),
    // 		v.base_z() + plate_height + clipped_z_height, v.base_z() + plate_height + clipped_z_height + alpha);

    // Align with vice
    box b = box(0, aligned.sides[0].len(),
    		0, aligned.sides[1].len(),
    		v.base_z() + plate_height + clipped_z_height, v.base_z() + plate_height + clipped_z_height + alpha);

    pocket p = box_pocket(b);
    vector<pocket> pockets{p};

    // TODO: Use actual workpiece mesh
    box bx(0, 1, 0, 1, 0, 1);
    triangular_mesh mesh = make_mesh(box_triangles(bx), 0.001);
    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(mesh);
    return fixture_setup(m, v, pockets);
  }

  point dims(const workpiece& w) {
    return point(w.sides[0].len(), w.sides[1].len(), w.sides[2].len());
  }

  std::vector<fixture_setup>
  parallel_clipping_programs(const workpiece& aligned,
			     const point clipped_dims,
			     std::vector<surface>& surfaces_to_cut,
			     const vice& v,
			     const double plate_height) {
    oriented_polygon outline =
      part_outline(&surfaces_to_cut);

    std::vector<fixture_setup> progs;
    progs.push_back(clip_top_and_sides(aligned, clipped_dims, outline, v, plate_height));
    progs.push_back(clip_base(aligned, clipped_dims.z, v, plate_height));
    return progs;
  }

  std::vector<plate_height>
  find_viable_parallel_plates(const double aligned_z_height,
			      const double clipped_z_height,
			      const fixtures& f) {
    const vice& v = f.get_vice();
    assert(!v.has_protective_base_plate());

    vector<plate_height> plates;
    for (auto p : f.parallel_plates()) {
      double adjusted_jaw_height = v.jaw_height() - p;
      assert(adjusted_jaw_height > 0);
      double leftover = aligned_z_height - clipped_z_height - adjusted_jaw_height;
      // TODO: Compute this magic number via friction analysis?
      if (leftover > 0.01 && (clipped_z_height - 0.01) > adjusted_jaw_height) {
	plates.push_back(p);
      }
    }
    return plates;
  }

  std::vector<fixture_setup>
  parallel_plate_clipping(const workpiece& aligned,
			  const point clipped_dims,
			  std::vector<surface>& surfaces_to_cut,
			  const fixtures& f) {
    double aligned_z_height = aligned.sides[2].len();
    double clipped_z_height = clipped_dims.z;
    vector<plate_height> viable_plates =
      find_viable_parallel_plates(aligned_z_height, clipped_z_height, f);

    if (viable_plates.size() > 0) {
      return parallel_clipping_programs(aligned,
					clipped_dims,
					surfaces_to_cut,
					f.get_vice(),
					viable_plates.front());
    }

    vector<fixture_setup> setups;
    return setups;
  }

  // TODO: Clean up and add vice height test
  // TODO: Use tool lengths in can_clip_parallel test
  std::pair<triangular_mesh, std::vector<fixture_setup> >
  try_parallel_plate_clipping(const workpiece w, 
			      const triangular_mesh& part_mesh,
			      std::vector<surface>& surfaces_to_cut,
			      const std::vector<tool>& tools,
			      const fixtures& f) {
    auto stable_surfaces = outer_surfaces(part_mesh);
    auto aligned_workpiece = align_workpiece(stable_surfaces, w);
    triangular_mesh wp_mesh = stock_mesh(aligned_workpiece);

    workpiece clipped = clipped_workpiece(aligned_workpiece, part_mesh);
    point clipped_dims = dims(clipped);

    vector<fixture_setup> clip_setups =
      parallel_plate_clipping(aligned_workpiece, clipped_dims, surfaces_to_cut, f);

    classify_part_surfaces(stable_surfaces, wp_mesh);
    remove_clipped_surfaces(stable_surfaces, surfaces_to_cut);
    return std::make_pair(wp_mesh, clip_setups);
  }

  std::vector<fixture_setup>
  axis_by_axis_clipping(std::vector<surface>& stable_surfaces,
			std::vector<surface>& surfaces_to_cut,
			const workpiece& aligned_workpiece,
			const workpiece& clipped,
			const fixtures& f) {
    triangular_mesh wp_mesh = stock_mesh(aligned_workpiece);

    classify_part_surfaces(stable_surfaces, wp_mesh);
    remove_clipped_surfaces(stable_surfaces, surfaces_to_cut);

    vector<fixture_setup> clip_setups;
    double eps = 0.05;
    append_clip_setups("X", 0, aligned_workpiece, clipped, eps, f.get_vice(), clip_setups);
    append_clip_setups("Y", 1, aligned_workpiece, clipped, eps, f.get_vice(), clip_setups);
    append_clip_setups("Z", 2, aligned_workpiece, clipped, eps, f.get_vice(), clip_setups);
    return clip_setups;
  }

  std::pair<triangular_mesh, std::vector<fixture_setup> >
  axis_by_axis_clipping(const workpiece w, 
			const triangular_mesh& part_mesh,
			std::vector<surface>& surfaces_to_cut,
			const std::vector<tool>& tools,
			const fixtures& f) {
    auto stable_surfaces = outer_surfaces(part_mesh);
    auto aligned_workpiece = align_workpiece(stable_surfaces, w);
    triangular_mesh wp_mesh = stock_mesh(aligned_workpiece);

    workpiece clipped = clipped_workpiece(aligned_workpiece, part_mesh);

    auto clip_setups = axis_by_axis_clipping(stable_surfaces, surfaces_to_cut, aligned_workpiece, clipped, f);

    classify_part_surfaces(stable_surfaces, wp_mesh);
    remove_clipped_surfaces(stable_surfaces, surfaces_to_cut);
    return std::make_pair(wp_mesh, clip_setups);
  }
  
  std::pair<triangular_mesh, std::vector<fixture_setup> >
  workpiece_clipping_programs(const workpiece w, 
			      const triangular_mesh& part_mesh,
			      std::vector<surface>& surfaces_to_cut,
			      const std::vector<tool>& tools,
			      const fixtures& f) {
    auto contour_clip = try_parallel_plate_clipping(w, part_mesh, surfaces_to_cut, tools, f);

    if (contour_clip.second.size() == 0) {
      return axis_by_axis_clipping(w, part_mesh, surfaces_to_cut, tools, f);
    } else {
      return contour_clip;
    }
  }

}
