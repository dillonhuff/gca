#include "synthesis/mesh_to_gcode.h"
#include "geometry/extrusion.h"
#include <boost/optional.hpp>

#include "gcode/gcode_program.h"
#include "gcode/lexer.h"
#include "geometry/vtk_debug.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/axis_3.h"
#include "synthesis/contour_planning.h"
#include "synthesis/face_clipping.h"
#include "synthesis/jaw_cutout.h"
#include "synthesis/millability.h"
#include "synthesis/tool.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/vice.h"
#include "synthesis/workpiece_clipping.h"

namespace gca {

  // TODO: Unify this with the pocket making code in fixture analysis
  std::vector<pocket>
  make_pockets(const triangular_mesh& part,
	       const triangular_mesh& stock,
	       const std::vector<surface>& surfaces) {
    std::vector<std::vector<index_t>> inds;
    for (auto s : surfaces) {
      inds.push_back(s.index_list());
    }
    auto mesh_cpy = new (allocate<triangular_mesh>()) triangular_mesh(part);
    return make_surface_pockets(*mesh_cpy, inds);
  }

  // May not be needed in new feature based system
  void add_surface_pockets(vector<gca::pocket>& pockets,
			   const rigid_arrangement& a,
			   const vector<gca::surface>& surfs) {
    DBG_ASSERT(a.contains_mesh("part"));
    DBG_ASSERT(a.contains_mesh("stock"));
    
    auto m = a.mesh("part");
    auto s = a.mesh("stock");
    unsigned old_size = pockets.size();
    concat(pockets, make_pockets(m, s, surfs));
    unsigned new_size = pockets.size();

    DBG_ASSERT((surfs.size() == 0) || (new_size > old_size));
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
    DBG_ASSERT(m.is_connected());
    return m;
  }

  std::vector<plate_height>
  find_viable_parallel_plates(const double aligned_z_height,
			      const double clipped_z_height,
			      const fixtures& f) {
    const vice& v = f.get_vice();
    DBG_ASSERT(!v.has_parallel_plate());

    vector<plate_height> plates;
    for (auto p : f.parallel_plates()) {
      double adjusted_jaw_height = v.jaw_height() - p;
      DBG_ASSERT(adjusted_jaw_height > 0);
      double leftover = aligned_z_height - clipped_z_height - adjusted_jaw_height;
      // TODO: Compute this magic number via friction analysis?
      if (leftover > 0.01 && (clipped_z_height - 0.01) > adjusted_jaw_height) {
	plates.push_back(p);
      }
    }
    return plates;
  }

  pocket face_down(const triangular_mesh& stock,
		   const triangular_mesh& part,
		   const triangular_mesh& out) {
    double work_height = max_in_dir(stock, point(0, 0, 1));
    double part_height = max_in_dir(part, point(0, 0, 1));

    DBG_ASSERT(work_height > part_height);

    auto bound = contour_outline(out.face_indexes(), out, point(0, 0, 1));
    if (bound) {
    } else {
      DBG_ASSERT(false);
    }
    vector<oriented_polygon> outlines =
      mesh_bounds((*bound).index_list(), (*bound).get_parent_mesh());

    DBG_ASSERT(outlines.size() == 2);

    oriented_polygon outl =
      min_e(outlines, [](const oriented_polygon& p)
	    { return min_z(p); });

    return face_pocket(work_height, part_height, outl);
  }
  
  pocket face_down(const triangular_mesh& stock,
		   const triangular_mesh& part) {
    double work_height = max_in_dir(stock, point(0, 0, 1));
    double part_height = max_in_dir(part, point(0, 0, 1));

    DBG_ASSERT(work_height > part_height);

    auto bound = contour_outline(stock.face_indexes(), stock, point(0, 0, 1));
    if (bound) {
    } else {
      DBG_ASSERT(false);
    }
    auto outlines =
      mesh_bounds((*bound).index_list(), (*bound).get_parent_mesh());
    oriented_polygon outl =
      min_e(outlines, [](const oriented_polygon& p)
	    { return min_z(p); }); // outlines.front();

    DBG_ASSERT(outlines.size() == 2);

    return face_pocket(work_height, part_height, outl);
  }

  
  pocket contour_around(const triangular_mesh& stock,
			const triangular_mesh& part) {
    double stock_top = max_in_dir(stock, point(0, 0, 1));
    double part_top = max_in_dir(part, point(0, 0, 1));

    DBG_ASSERT(stock_top > part_top);

    auto stock_bound =
      contour_outline(stock.face_indexes(), stock, point(0, 0, 1));
    if (stock_bound) {
    } else {
      DBG_ASSERT(false);
    }
    auto stock_outlines =
      mesh_bounds((*stock_bound).index_list(), (*stock_bound).get_parent_mesh());
    DBG_ASSERT(stock_outlines.size() == 2);
    oriented_polygon stock_outline = stock_outlines.front();
    
    auto part_bound = contour_outline(part.face_indexes(), part, point(0, 0, 1));
    if (part_bound) {
    } else {
      DBG_ASSERT(false);
    }
    auto part_outlines =
      mesh_bounds((*part_bound).index_list(), (*part_bound).get_parent_mesh());

    oriented_polygon part_outline =
      min_e(part_outlines, [](const oriented_polygon& p)
	    { return min_z(p); }); // outlines.front();

    double part_bottom = min_in_dir(part, point(0, 0, 1));    
    return contour_pocket(part_top, part_bottom, part_outline, stock_outline);
  }
  
  fixture_setup
  clip_top_and_sides(const triangular_mesh& aligned,
		     const triangular_mesh& part,
		     const fixture& f) {
    vector<pocket> pockets{face_down(aligned, part), contour_around(aligned, part)};

    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(aligned);
    return fixture_setup(m, f, pockets);
  }

  fixture_setup
  clip_top_and_sides_transform(const triangular_mesh& wp_mesh,
			       const triangular_mesh& part_mesh,
			       const std::vector<surface>& surfaces,
			       const fixture& f) {
    auto s_t = mating_transform(wp_mesh, f.orient, f.v);
    auto aligned = apply(s_t, wp_mesh);
    auto part = apply(s_t, part_mesh);
    fixture_setup setup = clip_top_and_sides(aligned, part, f);
    std::vector<pocket>& setup_pockets = setup.pockets;

    unsigned old_size = setup.pockets.size();
    concat(setup_pockets, make_pockets(part, aligned, surfaces));
    unsigned new_size = setup.pockets.size();

    DBG_ASSERT((surfaces.size() == 0) || (new_size > old_size));

    return setup;
  }

  fixture_setup
  clip_base_transform(const triangular_mesh& wp_mesh,
		      const triangular_mesh& part_mesh,
		      const std::vector<surface>& surfaces,
		      const fixture& f) {
    auto s_t = mating_transform(part_mesh, f.orient, f.v);
    auto aligned = apply(s_t, wp_mesh);
    auto part = apply(s_t, part_mesh);

    fixture_setup setup = clip_base(aligned, part, f);
    std::vector<pocket>& setup_pockets = setup.pockets;

    unsigned old_size = setup.pockets.size();
    concat(setup_pockets, make_pockets(part, aligned, surfaces));
    unsigned new_size = setup.pockets.size();

    DBG_ASSERT((surfaces.size() == 0) || (new_size > old_size));

    return setup;
  }

  fixture_setup
  clip_base(const triangular_mesh& aligned,
	    const triangular_mesh& part,
	    const fixture& f) {
    vector<pocket> pockets{face_down(aligned, part)};

    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(aligned);
    return fixture_setup(m, f, pockets);
  }

  clamp_orientation
  largest_upward_orientation(const std::vector<surface>& surfs,
			     const vice& parallel,
			     const point n) {
    vector<clamp_orientation> orients =
      all_viable_clamp_orientations(surfs, parallel);


    DBG_ASSERT(orients.size() > 0);

    vector<clamp_orientation> top_orients =
      select(orients, [n](const clamp_orientation& s)
	     { return within_eps(angle_between(s.top_normal(), n), 0, 1.0); });

    DBG_ASSERT(top_orients.size() > 0);

    const triangular_mesh& m = surfs.front().get_parent_mesh();
    sort(begin(top_orients), end(top_orients),
	 [m](const clamp_orientation& l, const clamp_orientation& r)
	 { return l.contact_area(m) > r.contact_area(m); });

    DBG_ASSERT(top_orients.size() > 0);
      
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

      vector<surface> stock_surfs = outer_surfaces(aligned);

      cout << "n = " << n << endl;
      auto stock_top_orient = largest_upward_orientation(stock_surfs, parallel, n);
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
    double total_area = merge_surfaces(const_orient_surfs).surface_area();
    delete_if(const_orient_surfs,
    	      [total_area](const surface& s)
    	      { return s.surface_area() < (total_area / 20.0); });
    const_orient_surfs.push_back(top_of_contour);
    std::vector<clamp_orientation> orients =
      all_stable_orientations(const_orient_surfs, v);
    if (orients.size() > 0) {
      cout << "NO CUSTOM JAW CUTOUT" << endl;
      auto cl = find_orientation_by_normal(orients, n);
      return fixture(cl, v);
    }

    cout << "Needs custom jaw cutout fixture" << endl;

    return boost::none;

  }

  clipping_plan
  base_fix_clip_plan(const triangular_mesh& aligned,
		     const triangular_mesh& part_mesh,
		     const contour_surface_decomposition& surfs,
		     const fixture& top_fix,
		     const fixture& base_fix) {
    std::vector<fixture_setup> clip_setups;
    clip_setups.push_back(clip_top_and_sides_transform(aligned, part_mesh, surfs.visible_from_n, top_fix));
    clip_setups.push_back(clip_base_transform(aligned, part_mesh, surfs.visible_from_minus_n, base_fix));

    auto clipped_surfs =
      stable_surfaces_after_clipping(part_mesh, aligned);
    auto surfs_to_cut = surfs.rest;

    return clipping_plan(clipped_surfs, surfs_to_cut, clip_setups);
  }

  boost::optional<clipping_plan>
  parallel_plate_clipping(const triangular_mesh& aligned,
			  const triangular_mesh& part_mesh,
			  const fixtures& f,
			  const fabrication_inputs& fab_inputs) {
    cout << "Trying parallel plate clipping" << endl;

    boost::optional<contour_surface_decomposition> surfs =
      compute_contour_surfaces(part_mesh);

    if (surfs) {
      point n = surfs->n;
      const surface& outline = surfs->outline;
      const surface& top = surfs->top;
      
      cout << "Has outline and flat top in " << n << endl;

      boost::optional<fixture> top_fix =
      	find_top_contour_fixture(aligned, part_mesh, f, n);

      if (top_fix) {
	cout << "Has top fix in " << n << endl;

	boost::optional<fixture> base_fix =
	  find_base_contour_fixture(outline, top, (*top_fix).v, -1*n);

	if (base_fix) {
	  return base_fix_clip_plan(aligned, part_mesh, *surfs, *top_fix, *base_fix);
	} else {
	  return custom_jaw_plan(aligned, part_mesh, *surfs, *top_fix, fab_inputs);
	}
      }
    }

    return boost::none;
  }

  clipping_plan
  workpiece_clipping_programs(const workpiece w,
			      const triangular_mesh& part_mesh,
			      const std::vector<tool>& tools,
			      const fixtures& f) {
    vector<surface> stable_surfaces = outer_surfaces(part_mesh);
    triangular_mesh wp_mesh = align_workpiece(stable_surfaces, w);

    auto contour_clip =
      parallel_plate_clipping(wp_mesh, part_mesh, f, fabrication_inputs{f, tools, w});

    if (contour_clip) {
      cout << "Contouring" << endl;
      return *contour_clip;
    } else {
      return axis_by_axis_clipping(w, part_mesh, tools, f);
    }
  }

}
