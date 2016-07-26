#include "gcode/linear_cut.h"
#include "synthesis/gcode_generation.h"
#include "synthesis/shape_layout.h"
#include "synthesis/shapes_to_gcode.h"
#include "utils/algorithm.h"

namespace gca {

  std::vector<polyline> reflect_y(const std::vector<polyline>& pocket_lines) {
    vector<polyline> reflected;
    for (auto p : pocket_lines) {
      reflected.push_back(p.apply([](const point p)
				  { return point(p.x, -p.y, p.z); }));
    }
    return reflected;
  }
  
  // TODO: Make the spindle_speed and feedrate parameters explicit
  cut* mk_cut(const point l,
	      const point r,
	      const double spindle_speed,
	      const double feedrate) {
    auto c = linear_cut::make(l, r);
    c->set_spindle_speed(lit::make(spindle_speed));
    c->set_feedrate(lit::make(feedrate));
    return c;
  }

  vector<cut*> polyline_cuts(const polyline& p,
			     const double spindle_speed,
			     const double feedrate) {
    auto ls = p.lines();
    assert(ls.size() > 0);
    vector<cut*> c;
    for (auto l : ls) {
      c.push_back(mk_cut(l.start, l.end, spindle_speed, feedrate));
    }
    return c;
  }

  vector<block> polylines_cuts(const vector<polyline>& pocket_lines,
			       const cut_params params,
			       const double spindle_speed,
			       const double feedrate) {
    vector<cut*> cuts;
    for (auto p : pocket_lines) {
      auto cs = polyline_cuts(p, spindle_speed, feedrate);
      cuts.insert(cuts.end(), cs.begin(), cs.end());
    }
    return cuts_to_gcode(cuts, params);
  }

  std::vector<block> emco_f1_code(const toolpath& tp) {
    for (auto l : tp.lines) {
      assert(l.num_points() > 0);
    }

    point shift_vector = point(0, 0, tp.t.length());
    vector<polyline> reflected_lines =
      shift_lines(reflect_y(tp.lines), shift_vector);

    cut_params params;
    params.target_machine = EMCO_F1;
    params.safe_height = tp.safe_z_before_tlc + tp.t.length();

    vector<block> blks{{token("Beginning of toolpath")}};
    concat(blks, polylines_cuts(reflected_lines, params, tp.spindle_speed, tp.feedrate));
    return blks;
  }

}
