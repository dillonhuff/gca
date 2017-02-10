#include "process_planning/major_axis_fixturing.h"

#include "process_planning/job_planning.h"
#include "process_planning/tool_access.h"
#include "synthesis/workpiece_clipping.h"
#include "synthesis/visual_debug.h"

namespace gca {

  boost::optional<dir_fixture>
  find_dir_fixture(const fixtures& f,
		   const point n,
		   const triangular_mesh& m) {
    Nef_polyhedron part_nef = trimesh_to_nef_polyhedron(m);

    vice v = f.get_vice();
    if (f.parallel_plates().size() > 0) {
      plate_height ph = f.parallel_plates().front();

      cout << "Selected parallel plate height = " << ph << endl;
      v = vice(f.get_vice(), ph);
    }

    vector<pair<clamp_orientation, homogeneous_transform>> orients =
      all_stable_orientations_with_side_transforms(part_nef, v, n);

    cout << "# of orients in " << n << " = " << orients.size() << endl;

    if (orients.size() > 0) {
      auto orient =
	max_e(orients,
	      [m]
	      (const std::pair<clamp_orientation, homogeneous_transform>& c)
	      { return c.first.contact_area(m); });

      cout << "Found fixture in " << n << endl;

      homogeneous_transform t = orient.second;

      return dir_fixture{orient.first, t, v};
    }

    return boost::none;
    
  }

  axis_fixture build_axis_fixture(const fixtures& f,
				  const major_axis_decomp& d) {
    const triangular_mesh& m = mesh(d);

    boost::optional<dir_fixture> positive =
      find_dir_fixture(f, d.major_axis, m);

    boost::optional<dir_fixture> negative =
      find_dir_fixture(f, -1*(d.major_axis), m);

    return axis_fixture{positive, negative};
  }

  triangular_mesh align_stock(const major_axis_decomp& cut_axis,
			      const dir_fixture& first_dir,
			      const workpiece w) {
    const auto& part = mesh(cut_axis);
    triangular_mesh m = stock_mesh(w);
    vector<surface> sfs = outer_surfaces(m);

    DBG_ASSERT(sfs.size() > 0);

    vector<plane> stock_planes = set_right_handed(max_area_basis(sfs));
    
    vector<plane> part_planes = cut_axis.axes;

    DBG_ASSERT(part_planes.size() == 3);

    vector<double> margin_values;
    for (auto& pl : part_planes) {
      if (angle_eps(pl.normal(), cut_axis.major_axis, 0.0, 1.0)) {
	margin_values.push_back(0.01);
      } else {
	margin_values.push_back(0.5);
      }
    }

    auto t = custom_offset_transform(part_planes,
				     stock_planes,
				     margin_values,
				     part,
				     m);

    DBG_ASSERT(t);

    triangular_mesh aligned_stock = apply(*t, m);
    
    //vtk_debug_meshes({part, aligned_stock});

    box part_bb = part.bounding_box();
    box aligned_bb = aligned_stock.bounding_box();

    DBG_ASSERT(fits_inside(part_bb, aligned_bb));

    return aligned_stock;
  }

  boost::optional<std::pair<fixture, homogeneous_transform> >
  find_next_fixture_side_vice(feature_decomposition* f,
			      const triangular_mesh& stock,
			      const point n,
			      const fixtures& fixes) {
    Nef_polyhedron stock_nef = trimesh_to_nef_polyhedron(stock);
    return find_next_fixture_side_vice(f, stock_nef, stock, n, fixes);
  }


  fixture_setup
  build_second_setup(const triangular_mesh& part,
		     const Nef_polyhedron& stock_nef,
		     const dir_fixture& second_dir,
		     const std::vector<tool>& tools) {
    triangular_mesh stock = nef_to_single_trimesh(stock_nef);

    vector<fixture_setup> setups;

    point n = second_dir.orient.top_normal();

    feature_decomposition* f =
      build_feature_decomposition(stock, part, n);

    tool_access_info tool_info =
      find_accessable_tools(f, tools);

    // Nef_polyhedron stock_nef = trimesh_to_nef_polyhedron(stock);
    // auto maybe_fix = find_next_fixture_side_vice(f, stock_nef, stock, n, fixes);
    
    // DBG_ASSERT(maybe_fix);

    vector<feature*> feats = collect_features(f);
    delete_if(feats, [tool_info](feature* f) {
	return map_find(f, tool_info).size() == 0;
      });

    cout << "# of accessable features = " << feats.size() << endl;
    
    fixture_setup s =
      create_setup(second_dir.placement,
		   stock,
		   part,
		   feats,
		   fixture(second_dir.orient, second_dir.v),
		   tool_info,
		   {},
		   {});

    return s;
  }

  fixture_plan
  axis_fixture_plan(const major_axis_decomp& cut_axis,
		    const axis_fixture& axis_fix,
		    const fixtures& fixes,
		    const workpiece w,
		    const std::vector<tool>& tools) {
    dir_fixture first_dir = *(axis_fix.positive);
    dir_fixture second_dir = *(axis_fix.negative);

    triangular_mesh stock = align_stock(cut_axis, first_dir, w);
    vector<fixture_setup> setups;

    point n = first_dir.orient.top_normal();
    const auto& part = mesh(cut_axis);

    feature_decomposition* f =
      build_feature_decomposition(stock, part, n);

    tool_access_info tool_info =
      find_accessable_tools(f, tools);

    Nef_polyhedron stock_nef = trimesh_to_nef_polyhedron(stock);
    auto maybe_fix = find_next_fixture_side_vice(f, stock_nef, stock, n, fixes);
    
    DBG_ASSERT(maybe_fix);

    vector<feature*> feats = collect_features(f);
    delete_if(feats, [tool_info](feature* f) {
	return map_find(f, tool_info).size() == 0;
      });

    cout << "# of accessable features = " << feats.size() << endl;
    
    fixture_setup s =
      create_setup(maybe_fix->second,
		   stock,
		   part,
		   feats,
		   maybe_fix->first,
		   tool_info,
		   {},
		   {});

    setups.push_back(s);

    stock_nef = subtract_features(stock_nef, feats);

    fixture_setup second =
      build_second_setup(part, stock_nef, second_dir, tools);

    setups.push_back(second);

    return fixture_plan(part, setups, w);
  }

}
