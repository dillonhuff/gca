#include "gcode/gcode_program.h"
#include "gcode/lexer.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/tool.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/vice.h"
#include "synthesis/workpiece_clipping.h"

namespace gca {

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

    // vector<polyline> blk_lines = shift_lines(pocket_2P5D_interior(p, t, cut_depth),
    // 					     point(0, 0, t.length()));

    // vector<block> blks;
    // if (blk_lines.size() > 0) {
    //   blks =
    // 	emco_f1_code(shift_lines_xy(blk_lines, v), safe_height);
    // }

    // vector<polyline> lines = shift_lines(pocket_2P5D_interior(q, t, cut_depth),
    // 					 point(0, 0, t.length()));

    // vector<block> clip_blocks;
    // if (lines.size() > 0) {
    //   clip_blocks =
    // 	emco_f1_code(shift_lines_xy(lines, v), safe_height);
    // }
    //    return pair<vector<block>, vector<block> >(blks, clip_blocks);
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

  fixture_setup
  clip_top_and_sides(const workpiece& aligned,
		     const workpiece& clipped,
		     std::vector<surface>& surfaces_to_cut,
		     const vice& v,
		     const double plate_height) {
    double aligned_x = aligned.sides[0].len();
    double clipped_x = clipped.sides[0].len();

    double aligned_y = aligned.sides[1].len();
    double clipped_y = clipped.sides[1].len();

    double aligned_z_height = aligned.sides[2].len();
    double clipped_z_height = clipped.sides[2].len();
    
    
    double adjusted_jaw_height = v.jaw_height() - plate_height;
    assert(adjusted_jaw_height > 0);
    double leftover = aligned_z_height - clipped_z_height - adjusted_jaw_height;
    double alpha = leftover / 2.0;

    double z_max = v.base_z() + plate_height + aligned_z_height;
    double z_min = z_max - alpha;

    // TODO: Use actual workpiece mesh
    box bx(0, 1, 0, 1, z_max - 3.0, z_max);
    triangular_mesh mesh = make_mesh(box_triangles(bx), 0.001);
    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(mesh);
    
    // Top facing operation
    box b1 = box(v.x_max() - aligned.sides[0].len(), v.x_max(),
    		 v.y_max() - aligned.sides[1].len(), v.y_max(),
    		 z_min, z_max);

    vector<pocket> pockets;
    pockets.push_back(box_pocket(b1));

    z_max = z_min;
    z_min = z_min - clipped_z_height;

    double x1 = v.x_max();
    double x2 = x1 - (aligned_x - clipped_x) / 2.0;
    double x3 = x2 - clipped_x;
    //    double x4 = x3 - (aligned_x - clipped_x) / 2.0;

    double y1 = v.y_max();
    double y2 = y1 - (aligned_y - clipped_y) / 2.0;
    double y3 = y2 - clipped_y;
    // double y4 = y3 - (aligned_y - clipped_y) / 2.0;

    box b2 = box(x3, x2, y3, y2, z_min, z_max);

    point p1(b2.x_min, b2.y_min, b2.z_min);
    point p2(b2.x_min, b2.y_max, b2.z_min);
    point p3(b2.x_max, b2.y_max, b2.z_min);
    point p4(b2.x_max, b2.y_min, b2.z_min);
    vector<point> verts{p1, p2, p3, p4};
    oriented_polygon interior(point(0, 0, 1), verts);

    point p5(b1.x_min, b1.y_min, b1.z_min);
    point p6(b1.x_min, b1.y_max, b1.z_min);
    point p7(b1.x_max, b1.y_max, b1.z_min);
    point p8(b1.x_max, b1.y_min, b1.z_min);
    vector<point> vs{p5, p6, p7, p8};
    oriented_polygon exterior(point(0, 0, 1), vs);

    pockets.push_back(contour_pocket(b2.z_max, b2.z_min, interior, exterior));

    // box b2 = box(x4, x1, y2, y1, z_min, z_max);
    // box b3 = box(x4, x1, y4, y3, z_min, z_max);
    // box b4 = box(x4, x3, y3, y2, z_min, z_max);
    // box b5 = box(x2, x1, y3, y2, z_min, z_max);

    // pockets.push_back(box_pocket(b2));
    // pockets.push_back(box_pocket(b3));
    // pockets.push_back(box_pocket(b4));
    // pockets.push_back(box_pocket(b5));

    return fixture_setup(m, v, pockets);
  }

  fixture_setup
  clip_base(const workpiece& aligned,
	    const workpiece& clipped,
	    const vice& v,
	    const double plate_height) {
    double aligned_z_height = aligned.sides[2].len();
    double clipped_z_height = clipped.sides[2].len();

    double adjusted_jaw_height = v.jaw_height() - plate_height;
    assert(adjusted_jaw_height > 0);
    double leftover = aligned_z_height - clipped_z_height - adjusted_jaw_height;
    double alpha = leftover / 2.0;

    box b = box(v.x_max() - aligned.sides[0].len(), v.x_max(),
    		v.y_max() - aligned.sides[1].len(), v.y_max(),
    		v.base_z() + plate_height + clipped_z_height, v.base_z() + plate_height + clipped_z_height + alpha);

    pocket p = box_pocket(b);
    vector<pocket> pockets{p};

    // TODO: Use actual workpiece mesh
    box bx(0, 1, 0, 1, 0, 1);
    triangular_mesh mesh = make_mesh(box_triangles(bx), 0.001);
    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(mesh);
    return fixture_setup(m, v, pockets);
  }

  std::vector<fixture_setup>
  parallel_clipping_programs(const workpiece& aligned,
			     const workpiece& clipped,
			     std::vector<surface>& surfaces_to_cut,
			     const vice& v,
			     const double plate_height) {
    std::vector<fixture_setup> progs;
    progs.push_back(clip_top_and_sides(aligned, clipped, surfaces_to_cut, v, plate_height));
    progs.push_back(clip_base(aligned, clipped, v, plate_height));
    return progs;
  }
  
  // TODO: Reduce duplication between here and can_clip_parallel
  std::vector<fixture_setup>
  parallel_plate_clipping(const workpiece& aligned,
			  const workpiece& clipped,
			  std::vector<surface>& surfaces_to_cut,
			  const fixtures& f) {
    double aligned_z_height = aligned.sides[2].len();
    double clipped_z_height = clipped.sides[2].len();

    const vice& v = f.get_vice();
    assert(!v.has_protective_base_plate());
    
    for (auto p : f.parallel_plates()) {
      double adjusted_jaw_height = v.jaw_height() - p;
      assert(adjusted_jaw_height > 0);
      double leftover = aligned_z_height - clipped_z_height - adjusted_jaw_height;
      // TODO: Compute this magic number via friction analysis?
      if (leftover > 0.01 && (clipped_z_height - 0.01) > adjusted_jaw_height) {
    	return parallel_clipping_programs(aligned,
    					  clipped,
					  surfaces_to_cut,
    					  f.get_vice(),
    					  p);
      }
    }
    assert(false);
  }

  bool can_clip_parallel(const workpiece& aligned,
			 const workpiece& clipped,
			 const fixtures& f) {
    double aligned_z_height = aligned.sides[2].len();
    double clipped_z_height = clipped.sides[2].len();

    const vice& v = f.get_vice();
    assert(!v.has_protective_base_plate());
    
    for (auto p : f.parallel_plates()) {
      double adjusted_jaw_height = v.jaw_height() - p;
      assert(adjusted_jaw_height > 0);
      double leftover = aligned_z_height - clipped_z_height - adjusted_jaw_height;
      // TODO: Compute this magic number via friction analysis?
      if (leftover > 0.01 && (clipped_z_height - 0.01) > adjusted_jaw_height) {
	return true;
      }
    }
    return false;
  }

  void remove_clipped_surfaces(const workpiece& aligned_workpiece,
			       const triangular_mesh& part_mesh,
			       std::vector<surface>& surfaces_to_cut) {
    auto stable_surfaces = outer_surfaces(part_mesh);
    classify_part_surfaces(stable_surfaces, aligned_workpiece);

    vector<index_t> stable_surface_inds;
    for (auto s : stable_surfaces) {
      if (s.is_SA()) {
    	concat(stable_surface_inds, s.index_list());
      }
    }
    sort(begin(stable_surface_inds), end(stable_surface_inds));
    delete_if(surfaces_to_cut,
	      [&stable_surface_inds](const surface& s)
	      { return s.contained_by_sorted(stable_surface_inds); });
  }

  // TODO: Clean up and add vice height test
  // TODO: Use tool lengths in can_clip_parallel test
  std::vector<fixture_setup>
  workpiece_clipping_programs(const workpiece aligned_workpiece,
			      const triangular_mesh& part_mesh,
			      std::vector<surface>& surfaces_to_cut,
			      const std::vector<tool>& tools,
			      const fixtures& f) {
    workpiece clipped = clipped_workpiece(aligned_workpiece, part_mesh);

    remove_clipped_surfaces(aligned_workpiece, part_mesh, surfaces_to_cut);

    if (can_clip_parallel(aligned_workpiece, clipped, f)) {
      return parallel_plate_clipping(aligned_workpiece, clipped, surfaces_to_cut, f);
    } else {
      double eps = 0.05;
      vector<fixture_setup> clip_setups;
      append_clip_setups("X", 0, aligned_workpiece, clipped, eps, f.get_vice(), clip_setups);
      append_clip_setups("Y", 1, aligned_workpiece, clipped, eps, f.get_vice(), clip_setups);
      append_clip_setups("Z", 2, aligned_workpiece, clipped, eps, f.get_vice(), clip_setups);
      return clip_setups;
    }
  }

}
