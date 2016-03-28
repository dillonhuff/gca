#include <algorithm>
#include <numeric>

#include "geometry/polyline.h"
#include "synthesis/circular_arc.h"
#include "synthesis/linear_cut.h"
#include "synthesis/shapes_to_toolpaths.h"
#include "synthesis/spline_sampling.h"
#include "system/algorithm.h"

namespace gca {

  vector<cut*> hole_cuts(const vector<hole_punch*>& holes,
  			 const cut_params& params) {
    vector<cut*> cuts;
    for (vector<hole_punch*>::const_iterator it = holes.begin();
  	 it != holes.end(); ++it) {
      cuts.push_back(*it);
    }
    return cuts;
  }
  
  tool_name select_tool(ToolOptions tools) {
    if (tools == DRILL_AND_DRAG_KNIFE ||
    	tools == DRAG_KNIFE_ONLY) {
      return DRAG_KNIFE;
    } else if (tools == DRILL_ONLY) {
      return DRILL;
    } else {
      assert(false);
    }
  }

  vector<double> cut_depths(const cut_params& params) {
    vector<double> depths;
    double depth = max(0.0, params.material_depth - params.cut_depth);
    while (true) {
      depths.push_back(depth);
      if (depth == 0.0) {
	break;
      }
      depth = max(0.0, depth - params.cut_depth);
    }
    return depths;
  }

  void set_tool_nos(tool_name tool_no,
		    const vector<cut*>& cuts) {
    for (vector<cut*>::const_iterator it = cuts.begin(); it != cuts.end(); ++it) {
      cut* c = *it;
      if (c->is_hole_punch()) {
	c->tool_no = DRILL;
      } else {
	c->tool_no = tool_no;
      }
    }
  }

  void insert_lines(const vector<cut*>& lines,
		    vector<polyline>& polys) {
    for (auto c : lines) {
      vector<point> p{c->get_start(), c->get_end()};
      polys.push_back(polyline(p));
    }
  }

  vector<polyline> deepen_polyline(vector<double> depths, const polyline& p) {
    vector<polyline> ps;
    for (auto depth : depths) {
      vector<point> pts;
      for (auto pt : p)
	{ pts.push_back(point(pt.x, pt.y, depth)); }
      ps.push_back(polyline(pts));
    }
    return ps;
  }

  vector<point> cuts_to_points(vector<cut*> cuts) {
    vector<point> pts;
    for (auto c : cuts) {
      point s = c->get_start();
      point e = c->get_end();
      if (pts.size() == 0) {
	pts.push_back(s);
      }
      pts.push_back(e);      
    }
    return pts;
  }

  vector<point> lines_to_points(vector<line> cuts) {
    vector<point> pts;
    for (auto c : cuts) {
      point s = c.start;
      point e = c.end;
      if (pts.size() == 0) {
	pts.push_back(s);
      }
      pts.push_back(e);
    }
    return pts;
  }
  
  vector<polyline> make_polylines_from(vector<cut*> lines) {
    auto adj_test = [](cut* c, cut* n) { return within_eps(c->get_end(), n->get_start()); };
    auto not_adj_test = [](cut* c, cut* n) {
      bool touching = within_eps(c->get_end(),n->get_start());
      return !touching; // || !small_orientation_diff;
    };
    greedy_adjacent_chains(lines.begin(), lines.end(), adj_test);
    vector<polyline> pls;
    auto it = lines.begin();
    while (it != lines.end()) {
      auto r = find_between(it, lines.end(), not_adj_test);
      vector<point> lts = cuts_to_points(vector<cut*>(it, r.first + 1));
      pls.push_back(polyline(lts));
      it = r.second;
    }
    return pls;
  }

  vector<polyline> polylines_for_shapes(const shape_layout& shapes_to_cut) {
    vector<polyline> polys;
    append_splines(shapes_to_cut.splines, polys);
    auto lines = shapes_to_cut.lines;
    vector<polyline> pls = make_polylines_from(lines);
    polys.insert(polys.end(), pls.begin(), pls.end());
    return polys;
  }

  vector<polyline> shift_polys_for_drag_knife(double d, const vector<polyline>& ps) {
    vector<polyline> polys;
    for (auto pline : ps) {
      vector<point> pts;
      point offset;
      auto it = pline.begin();
      for (; it != pline.end() - 1; ++it) {
	point p = *it;
	point next = *(it + 1);
	offset = d * ((next - p).normalize());
	pts.push_back(p + offset);
      }
      pts.push_back(*it + offset);
      polys.push_back(polyline(pts));
    }
    return polys;
  }

  void split_into_main_and_finish(const polyline& pl,
				  double offset,
				  vector<polyline>& last_main_cuts,
				  vector<polyline>& last_finish_cuts) {
    assert(pl.num_points() > 1);
    vector<point> split_points(pl.num_points());
    adjacent_difference(pl.begin(), pl.end(),
			split_points.begin(),
			[offset](const point r, const point l)
			{ return l + offset*((r - l).normalize()); });
    auto pli = pl.begin();
    auto spi = split_points.begin() + 1;
    for (; pli != pl.end() - 1; ++pli, ++spi) {
      last_finish_cuts.push_back(polyline({*pli, *spi}));
      last_main_cuts.push_back(polyline({*spi, *(pli + 1)}));
    }
  }

  bool all_contiguous(const polyline& p) {
    if (p.num_points() < 3) { return true; }
    vector<point> orientations(p.num_points() - 1);
    apply_between(p.begin(), p.end(),
		  orientations.begin(),
		  [](point p, point n) { return  n - p; });
    return all_between(orientations.begin(), orientations.end(),
		       [](point l, point r)
		       { return within_eps(angle_between(l, r), 0, 15); });
  }

  vector<polyline> deepen_polys(const vector<double> depths,
				const vector<polyline>& ps) {
    vector<polyline> deepened_polys;
    for (auto pl : ps) {
      vector<polyline> dps = deepen_polyline(depths, pl);
      deepened_polys.insert(deepened_polys.end(), dps.begin(), dps.end());
    }
    return deepened_polys;
  }

  vector<polyline> break_adjacent_cuts(double last_depth,
				       const vector<polyline>& dps) {
    vector<polyline> deepened_polys;
    vector<polyline> finish_lines;
    for (auto pl : dps) {
      vector<polyline> last_main_cuts;
      vector<polyline> last_finish_cuts;
      if (within_eps(pl.pt(0).z, last_depth) && !all_contiguous(pl)) {
      	// TODO: Make 0.16 a parameter
      	split_into_main_and_finish(pl, 0.16, last_main_cuts, last_finish_cuts);
      } else {
	last_main_cuts.push_back(pl);
      }
      deepened_polys.insert(deepened_polys.end(),
      			    last_main_cuts.begin(), last_main_cuts.end());
      finish_lines.insert(finish_lines.end(),
      			  last_finish_cuts.begin(), last_finish_cuts.end());
    }
    deepened_polys.insert(deepened_polys.end(),
    			  finish_lines.begin(), finish_lines.end());
    return deepened_polys;
  }

  vector<polyline> shift_polys(double offset,
			       const cut_params& params,
			       const vector<polyline>& ps) {
    vector<polyline> polys;
    if (params.tools == DRAG_KNIFE_ONLY ||
	params.tools == DRILL_AND_DRAG_KNIFE) {
      polys = shift_polys_for_drag_knife(0.16, ps);
    } else {
      polys = ps;
    }
    return polys;
  }

  vector<polyline> break_into_contiguous_cuts(const vector<polyline>& ps) {
    vector<line> lines;
    for (auto p : ps) {
      auto ls = p.lines();
      lines.insert(lines.end(), ls.begin(), ls.end());
    }
    auto not_adj_test = [](line c, line n) {
      bool touching = within_eps(c.end, n.start);
      bool small_orientation_diff =
      within_eps(angle_between(c.end - c.start, n.end - n.start), 0, 15);
      return !touching || !small_orientation_diff;
    };
    vector<polyline> pls;
    auto it = lines.begin();
    while (it != lines.end()) {
      auto r = find_between(it, lines.end(), not_adj_test);
      vector<point> lts = lines_to_points(vector<line>(it, r.first + 1));
      pls.push_back(polyline(lts));
      it = r.second;
    }
    return pls;
  }

  vector<cut*> cuts_from_polylines(const shape_layout& shapes_to_cut,
				   const vector<polyline>& ps,
				   const cut_params& params) {
    vector<double> depths = cut_depths(params);
    vector<polyline> dps = deepen_polys(depths, ps);
    auto broken_pls = break_adjacent_cuts(depths.back(), dps);
    auto pls = break_into_contiguous_cuts(broken_pls);
    auto polys = shift_polys(0.16, params, pls);
    vector<cut*> cuts;
    if (params.tools != DRAG_KNIFE_ONLY) {
      vector<cut*> dcs = hole_cuts(shapes_to_cut.holes, params);
      cuts.insert(cuts.end(), dcs.begin(), dcs.end());
    }
    for (auto dpl : polys) {
      for (auto l : dpl.lines())
	{ cuts.push_back(linear_cut::make(l.start, l.end)); }
    }
    tool_name tool_no = select_tool(params.tools);
    set_tool_nos(tool_no, cuts);
    return cuts;
  }

  template<typename T>
  struct no_op { T operator()(const T t) const { return t; } };
  
  vector<cut*> shape_cuts(const shape_layout& shapes_to_cut,
			  const cut_params& params) {
    return shape_cuts_p(shapes_to_cut, params, no_op<vector<polyline>>());
  }
}
