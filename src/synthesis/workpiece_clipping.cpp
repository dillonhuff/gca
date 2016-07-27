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

  std::vector<plate_height>
  find_viable_parallel_plates(const double aligned_z_height,
			      const double clipped_z_height,
			      const fixtures& f) {
    const vice& v = f.get_vice();
    assert(!v.has_parallel_plate());

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
		     const fixture& f) {
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
    return fixture_setup(m, f, pockets);
  }

  fixture_setup
  clip_top_and_sides_transform(const triangular_mesh& wp_mesh,
			       const triangular_mesh& part_mesh,
			       const fixture& f) {
    auto s_t = mating_transform(wp_mesh, f.orient, f.v);
    auto aligned = apply(s_t, wp_mesh);
    auto part = apply(s_t, part_mesh);
    return clip_top_and_sides(aligned, part, f);
  }

  fixture_setup
  clip_base_transform(const triangular_mesh& wp_mesh,
		      const triangular_mesh& part_mesh,
		      const fixture& f) {
    auto s_t = mating_transform(part_mesh, f.orient, f.v);
    auto aligned = apply(s_t, wp_mesh);
    auto part = apply(s_t, part_mesh);
    return clip_base(aligned, part, f);
  }

  fixture_setup
  clip_base(const triangular_mesh& aligned,
	    const triangular_mesh& part,
	    const fixture& f) {
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
    return fixture_setup(m, f, pockets);
  }

  clamp_orientation
  largest_upward_orientation(const std::vector<surface>& surfs,
			     const vice& parallel) {
    // TODO: Eventually auto select the direction instead of
    // hard coding (0, 0, 1)
    point n(0, 0, 1);

    vector<clamp_orientation> orients =
      all_viable_clamp_orientations(surfs, parallel);

    cout << "# of orientations = " << orients.size() << endl;

    vector<clamp_orientation> top_orients =
      select(orients, [n](const clamp_orientation& s)
	     { return within_eps(s.top_normal(), n, 0.0001); });

    assert(orients.size() > 0);

    const triangular_mesh& m = surfs.front().get_parent_mesh();
    sort(begin(top_orients), end(top_orients),
	 [m](const clamp_orientation& l, const clamp_orientation& r)
	 { return l.contact_area(m) > r.contact_area(m); });

    assert(top_orients.size() > 0);
      
    return top_orients.front();
  }

  boost::optional<fixture>
  find_top_contour_fixture(const triangular_mesh& aligned,
			   const triangular_mesh& part_mesh,
			   const fixtures& f,
			   const point n) {
      double aligned_z_height = diameter(n, aligned);
      double clipped_z_height = diameter(n, part_mesh);
      vector<plate_height> viable_plates =
      	find_viable_parallel_plates(aligned_z_height, clipped_z_height, f);

      cout << "# of viable parallel plates = " << viable_plates.size() << endl;

      if (viable_plates.size() > 0) {
      	vice parallel(f.get_vice(), viable_plates.front());

      	vector<surface> surfs = outer_surfaces(part_mesh);
      	vector<surface> stock_surfs = outer_surfaces(aligned);

      	auto stock_top_orient = largest_upward_orientation(stock_surfs, parallel);
	return fixture(stock_top_orient, parallel);
      } else {
	return boost::none;
      }
  }


  // TODO: Add code to generate jaws for base fixture
  boost::optional<fixture>
  find_base_contour_fixture(const surface& outline_of_contour,
			    const surface& top_of_contour,
			    const vice& v,
			    const point n) {
    std::vector<surface> const_orient_surfs =
      constant_orientation_subsurfaces(outline_of_contour);
    const_orient_surfs.push_back(top_of_contour);
    std::vector<clamp_orientation> orients =
      all_stable_orientations(const_orient_surfs, v);
    auto cl = find_orientation_by_normal(orients, n);
    return fixture(cl, v);
  }

  boost::optional<surface>
  mesh_top_surface(const triangular_mesh& m, const point n) {
    auto surfs = outer_surfaces(m);
    delete_if(surfs,
	      [n](const surface& s) { return !s.parallel_to(n, 0.001); });
    if (surfs.size() > 0) {
      return merge_surfaces(surfs);
    } else {
      return boost::none;
    }
  }
  
  boost::optional<clipping_plan>
  parallel_plate_clipping(const triangular_mesh& aligned,
			  const triangular_mesh& part_mesh,
			  const fixtures& f) {
    point n(0, 0, 1);
    
    vector<surface> surfs_to_cut = surfaces_to_cut(part_mesh);

    assert(surfs_to_cut.size() > 0);

    boost::optional<surface> outline = part_outline_surface(&surfs_to_cut, n);
    boost::optional<surface> top = mesh_top_surface(part_mesh, n);

    if (outline && top) {
      cout << "Has outline and flat top in " << n << endl;
      
      remove_contained_surfaces({*outline}, surfs_to_cut);

      // TOOD: Remove outline as a parameter?
      boost::optional<fixture> top_fix =
      	find_top_contour_fixture(aligned, part_mesh, f, n);

      if (top_fix) {
	cout << "Has top fix in " << n << endl;

	boost::optional<fixture> base_fix =
	  find_base_contour_fixture(*outline, *top, (*top_fix).v, -1*n);

	if (base_fix) {
	  std::vector<fixture_setup> clip_setups;
	  clip_setups.push_back(clip_top_and_sides_transform(aligned, part_mesh, *top_fix));
	  clip_setups.push_back(clip_base_transform(aligned, part_mesh, *base_fix));

	  auto clipped_surfs =
	    stable_surfaces_after_clipping(part_mesh, aligned);
	  remove_contained_surfaces(clipped_surfs, surfs_to_cut);

	  return clipping_plan(clipped_surfs, surfs_to_cut, clip_setups);
	}

      }
    }

    return boost::none;
  }

  // TODO: Clean up and add vice height test
  // TODO: Use tool lengths in can_clip_parallel test
  boost::optional<clipping_plan>
  try_parallel_plate_clipping(const workpiece w, 
			      const triangular_mesh& part_mesh,
			      const std::vector<tool>& tools,
			      const fixtures& f) {
    vector<surface> stable_surfaces = outer_surfaces(part_mesh);
    triangular_mesh wp_mesh = align_workpiece(stable_surfaces, w);

    return parallel_plate_clipping(wp_mesh, part_mesh, f);
  }

  clipping_plan
  workpiece_clipping_programs(const workpiece w,
			      const triangular_mesh& part_mesh,
			      const std::vector<tool>& tools,
			      const fixtures& f) {
    

    auto contour_clip =
      try_parallel_plate_clipping(w, part_mesh, tools, f);

    if (contour_clip) {
      return *contour_clip;
    } else {
      return axis_by_axis_clipping(w, part_mesh, tools, f);
    }
  }

}
