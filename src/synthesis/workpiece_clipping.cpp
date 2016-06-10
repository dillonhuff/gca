#include "gcode/gcode_program.h"
#include "gcode/lexer.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/tool.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/vice.h"
#include "synthesis/workpiece.h"

namespace gca {

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

}