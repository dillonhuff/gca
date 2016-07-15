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

  point dims(const workpiece& w) {
    return point(w.sides[0].len(), w.sides[1].len(), w.sides[2].len());
  }

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
  
  fixture_setup
  clip_top_and_sides(const triangular_mesh& aligned,
		     const triangular_mesh& part,
		     const vice& v) {
    double stock_top = max_in_dir(aligned, point(0, 0, 1));
    double part_top = max_in_dir(part, point(0, 0, 1));
    double part_bottom = min_in_dir(part, point(0, 0, 1));

    assert(stock_top > part_top);

    auto stock_bound = part_outline_surface(aligned, point(0, 0, 1));
    if (stock_bound) {
    } else {
      assert(false);
    }
    auto stock_outlines =
      mesh_bounds((*stock_bound).index_list(), (*stock_bound).get_parent_mesh());
    assert(stock_outlines.size() == 2);
    oriented_polygon stock_outline = stock_outlines.front();

    auto part_bound = part_outline_surface(part, point(0, 0, 1));
    if (part_bound) {
    } else {
      assert(false);
    }
    auto part_outlines =
      mesh_bounds((*part_bound).index_list(), (*part_bound).get_parent_mesh());
    assert(part_outlines.size() == 2);
    oriented_polygon part_outline = part_outlines.front();

    vector<pocket> pockets{face_pocket(stock_top, part_top, stock_outline)};

    pockets.push_back(contour_pocket(part_top, part_bottom, part_outline, stock_outline));

    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(aligned);
    return fixture_setup(m, v, pockets);
  }

  fixture_setup
  clip_base(const triangular_mesh& aligned,
	    const triangular_mesh& part,
	    const vice& v) {
    double work_height = max_in_dir(aligned, point(0, 0, 1));
    double part_height = max_in_dir(part, point(0, 0, 1));

    assert(work_height > part_height);

    auto bound = part_outline_surface(aligned, point(0, 0, 1));
    if (bound) {
    } else {
      assert(false);
    }
    auto outlines =
      mesh_bounds((*bound).index_list(), (*bound).get_parent_mesh());
    assert(outlines.size() == 2);
    vector<pocket> pockets{face_pocket(work_height, part_height, outlines.front())};

    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(aligned);
    return fixture_setup(m, v, pockets);
  }

  clamp_orientation
  find_orientation_by_normal(const std::vector<clamp_orientation>& orients,
			     const point n) {
    return *(find_if(begin(orients), end(orients),
		     [n](const clamp_orientation& s)
		     { return within_eps(s.top_normal(), n, 0.0001); }));
  }

  clamp_orientation
  largest_upward_orientation(const std::vector<surface>& surfs,
			     const vice& parallel) {
    vector<clamp_orientation> orients =
      all_viable_clamp_orientations(surfs, parallel);

    // TODO: Eventually auto select the direction instead of
    // hard coding (0, 0, 1)
    vector<clamp_orientation> top_orients =
      select(orients, [](const clamp_orientation& s)
	     { return within_eps(s.top_normal(), point(0, 0, 1), 0.0001); });

    assert(orients.size() > 0);

    sort(begin(top_orients), end(top_orients),
	 [](const clamp_orientation& l, const clamp_orientation& r)
	 { return l.surface_area() > r.surface_area(); });
      
    return top_orients.front();
  }
  
  boost::optional<std::vector<fixture_setup> >
  parallel_clipping_programs(const triangular_mesh& aligned,
			     const point clipped_dims,
			     std::vector<surface>& surfaces_to_cut,
			     const vice& v,
			     const double plate_height) {
    boost::optional<oriented_polygon> outline =
      part_outline(&surfaces_to_cut);

    if (outline) {
      vice parallel(v, plate_height);
      const triangular_mesh& m = surfaces_to_cut.front().get_parent_mesh();

      vector<surface> stock_surfs = outer_surfaces(aligned);
      auto stock_top_orient = largest_upward_orientation(stock_surfs, parallel);
      auto s_t = mating_transform(stock_top_orient, parallel);
      
      std::vector<fixture_setup> progs;
      progs.push_back(clip_top_and_sides(apply(s_t, aligned), apply(s_t, m), parallel));

      vector<surface> surfs = outer_surfaces(m);
      
      auto top_orient = largest_upward_orientation(surfs, parallel);
      auto t = mating_transform(top_orient, parallel);

      progs.push_back(clip_base(apply(t, aligned), apply(t, m), parallel));
      return progs;
    }
    return boost::none;
  }

  boost::optional<std::vector<fixture_setup> >
  parallel_plate_clipping(const triangular_mesh& aligned,
			  const point clipped_dims,
			  std::vector<surface>& surfaces_to_cut,
			  const fixtures& f) {
    assert(surfaces_to_cut.size() > 0);
    
    double aligned_z_height = diameter(point(0, 0, 1), aligned);
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
    vector<surface> stable_surfaces = outer_surfaces(part_mesh);
    triangular_mesh wp_mesh = align_workpiece(stable_surfaces, w);

    workpiece clipped = clipped_workpiece(w, part_mesh);
    point clipped_dims = dims(clipped);

    boost::optional<vector<fixture_setup>> clip_setups =
      parallel_plate_clipping(wp_mesh, clipped_dims, surfaces_to_cut, f);

    if (clip_setups) {
      auto clipped_surfs =
	stable_surfaces_after_clipping(part_mesh, wp_mesh);
      remove_contained_surfaces(clipped_surfs, surfaces_to_cut);
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
    auto contour_clip =
      try_parallel_plate_clipping(w, part_mesh, surfaces_to_cut, tools, f);

    if (contour_clip) {
      return *contour_clip;
    } else {
      return axis_by_axis_clipping(w, part_mesh, surfaces_to_cut, tools, f);
    }
  }

}
