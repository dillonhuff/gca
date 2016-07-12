#include <boost/optional.hpp>

#include "gcode/gcode_program.h"
#include "gcode/lexer.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/face_clipping.h"
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

  boost::optional<surface>
  part_outline_surface(std::vector<surface>* surfaces_to_cut,
		       const point n) {
    vector<surface> vertical_surfs =
      select(*surfaces_to_cut,
    	     [n](const surface& s)
    	     { return s.orthogonal_to(n, 0.01); });

    vector<vector<unsigned>> merge_groups =
      connected_components_by(vertical_surfs, [](const surface& l, const surface& r)
    			      { return surfaces_share_edge(l, r); });

    if (merge_groups.size() == 1) {
      return merge_surfaces(vertical_surfs);
    }

    return boost::none;
  }

  // TODO: Need to add normal vectors, how to match this with
  // the code in make_fixture_plan?
  boost::optional<oriented_polygon>
  part_outline(std::vector<surface>* surfaces_to_cut) {
    point n(0, 0, 1);

    auto m = part_outline_surface(surfaces_to_cut, n);
    if (m) {
      vector<oriented_polygon> outlines =
	mesh_bounds((*m).index_list(), (*m).get_parent_mesh());
      if (outlines.size() == 2) {
	vector<surface> vertical_surfs{*m};
	remove_contained_surfaces(vertical_surfs, *surfaces_to_cut);
	return outlines.front();
      }
    }

    return boost::none;
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

  boost::optional<std::vector<fixture_setup>>
  parallel_clipping_programs(const workpiece& aligned,
			     const point clipped_dims,
			     std::vector<surface>& surfaces_to_cut,
			     const vice& v,
			     const double plate_height) {
    boost::optional<oriented_polygon> outline =
      part_outline(&surfaces_to_cut);

    if (outline) {
      std::vector<fixture_setup> progs;
      progs.push_back(clip_top_and_sides(aligned, clipped_dims, *outline, v, plate_height));
      progs.push_back(clip_base(aligned, clipped_dims.z, v, plate_height));
      return progs;
    }
    return boost::none;
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

  boost::optional<std::vector<fixture_setup> >
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
    return boost::none;
  }

  // TODO: Clean up and add vice height test
  // TODO: Use tool lengths in can_clip_parallel test
  boost::optional<std::pair<triangular_mesh, std::vector<fixture_setup> > >
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

    boost::optional<vector<fixture_setup>> clip_setups =
      parallel_plate_clipping(aligned_workpiece, clipped_dims, surfaces_to_cut, f);

    classify_part_surfaces(stable_surfaces, wp_mesh);
    remove_clipped_surfaces(stable_surfaces, surfaces_to_cut);
    if (clip_setups) {
      return std::make_pair(wp_mesh, *clip_setups);
    } else {
      return boost::none;
    }
  }
  
  std::pair<triangular_mesh, std::vector<fixture_setup> >
  workpiece_clipping_programs(const workpiece w, 
			      const triangular_mesh& part_mesh,
			      std::vector<surface>& surfaces_to_cut,
			      const std::vector<tool>& tools,
			      const fixtures& f) {
    auto contour_clip = try_parallel_plate_clipping(w, part_mesh, surfaces_to_cut, tools, f);

    if (contour_clip) {
      return *contour_clip;
    } else {
      return axis_by_axis_clipping(w, part_mesh, surfaces_to_cut, tools, f);
    }
  }

}
