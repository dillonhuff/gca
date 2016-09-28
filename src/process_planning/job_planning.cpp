#include <vtkDelaunay2D.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkLine.h>
#include <vtkPolygon.h>
#include <vtkSmartPointer.h>
#include <vtkLinearExtrusionFilter.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>

#include "geometry/extrusion.h"
#include "geometry/mesh_operations.h"
#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "feature_recognition/visual_debug.h"
#include "process_planning/feature_selection.h"
#include "process_planning/feature_to_pocket.h"
#include "process_planning/job_planning.h"
#include "synthesis/workpiece_clipping.h"
#include "utils/check.h"

namespace gca {

  fixture_setup
  create_setup(const homogeneous_transform& s_t,
	       const triangular_mesh& wp_mesh,
	       const triangular_mesh& part_mesh,
	       const std::vector<feature*>& features,
	       const fixture& f,
	       const tool_access_info& tool_info) {
    auto aligned = apply(s_t, wp_mesh);
    auto part = apply(s_t, part_mesh);

    vector<pocket> pockets = feature_pockets(features, s_t, tool_info);

    triangular_mesh* m = new (allocate<triangular_mesh>()) triangular_mesh(aligned);
    return fixture_setup(m, f, pockets);
  }

  triangular_mesh feature_mesh(const feature& f,
			       const double base_dilation,
			       const double base_extension,
			       const double top_extension) {
    point shift_vec = (-1*top_extension)*f.normal();

    if (base_dilation > 0.0) {
      auto dilated_base = dilate(f.base(), base_dilation);
      dilated_base = shift(shift_vec, dilated_base);
    
      auto m = extrude(dilated_base, (base_extension + f.depth())*f.normal());

      DBG_ASSERT(m.is_connected());

      return m;

    } else {
      auto base = shift(shift_vec, f.base());

      auto m = extrude(base, (base_extension + f.depth())*f.normal());

      DBG_ASSERT(m.is_connected());

      return m;
    }
  }

  triangular_mesh feature_mesh(const feature& f) {
    return feature_mesh(f, 0.0, 0.0001, 0.0000); 
  }
  
  std::vector<triangular_mesh>
  subtract_features(const triangular_mesh& m,
		    const std::vector<feature*>& features) {
    std::vector<triangular_mesh> meshes;
    for (auto f : features) {
      meshes.push_back(feature_mesh(*f));
    }

    auto subtracted = boolean_difference(m, meshes);

    return subtracted;
  }

  double volume(const feature& f) {
    const rotation r = rotate_from_to(f.normal(), point(0, 0, 1));
    auto bp = to_boost_poly_2(apply(r, f.base()));

    double base_area = bg::area(bp);
    
    return base_area * f.depth();
  }

  struct volume_info {
    double volume;
    std::vector<triangular_mesh> meshes;
    triangular_mesh dilated_mesh;
  };

  typedef std::unordered_map<feature*, volume_info> volume_info_map;

  volume_info initial_volume_info(const feature& f) {
    triangular_mesh mesh = feature_mesh(f);
    double vol = volume(mesh);

    //	TODO: Refine these tolerances, it may not matter but
    //	best to be safe
    triangular_mesh dilated_mesh = feature_mesh(f, 0.05, 0.05, 0.05);
    
    return volume_info{vol, {mesh}, dilated_mesh};
  }

  volume_info
  update_volume_info(const volume_info& inf,
		     const std::vector<triangular_mesh>& to_subtract) {
    if (inf.meshes.size() == 0) {

      DBG_ASSERT(inf.volume == 0.0);

      return inf;
    }

    std::vector<triangular_mesh> res =
      boolean_difference(inf.meshes, to_subtract);


    // TODO: Delete the else clause?
    if (res.size() > 0) {

      double new_volume = 0.0;
      for (auto& m : res) {
	new_volume += volume(m);
      }

      cout << "Old volume = " << inf.volume << endl;
      cout << "New volume = " << new_volume << endl;

      return volume_info{new_volume, res, inf.dilated_mesh};

    } else {

      cout << "Old volume = " << inf.volume << endl;
      cout << "New volume = " << 0.0 << endl;

      return volume_info{0.0, {}, inf.dilated_mesh};
    }

  }

  volume_info_map
  initial_volume_info(const std::vector<direction_process_info>& dir_info) {
    volume_info_map m;

    for (auto d : dir_info) {
      auto decomp = d.decomp;
      for (feature* f : collect_features(decomp)) {
	m[f] = initial_volume_info(*f);
      }
    }

    return m;
  }

  void
  clip_volumes(feature_decomposition* decomp,
	       volume_info_map& volume_inf) {
    vector<feature*> feats_to_sub = collect_features(decomp);

    vector<triangular_mesh> to_subtract;
    for (auto f : feats_to_sub) {
      volume_info f_info = map_find(f, volume_inf);

      // If the volume still exists
      if (f_info.meshes.size() > 0) {
	to_subtract.push_back(f_info.dilated_mesh);
      }
    }

    for (auto& info_pair : volume_inf) {
      feature* f = info_pair.first;

      // Do not subtract the selected decomposition, those
      // features are being removed anyway
      if (!elem(f, feats_to_sub)) {
	volume_inf[f] = update_volume_info(info_pair.second, to_subtract);
      }
    }
  }

  double volume(feature* f, const volume_info_map& vol_info) {
    volume_info inf = map_find(f, vol_info);

    return inf.volume;
  }

  double volume(feature_decomposition* f, const volume_info_map& vol_info) {
    auto feats = collect_features(f);

    double vol = 0.0;
    for (auto ft : feats) {
      vol += volume(ft, vol_info);
    }

    return vol;
  }

  direction_process_info
  select_next_dir(std::vector<direction_process_info>& dir_info,
		  const volume_info_map& vol_info) {
    DBG_ASSERT(dir_info.size() > 0);

    auto next = max_element(begin(dir_info), end(dir_info),
			    [vol_info](const direction_process_info& l,
				       const direction_process_info& r) {
			      return volume(l.decomp, vol_info) <
			      volume(r.decomp, vol_info);
			    });

    DBG_ASSERT(next != end(dir_info));

    direction_process_info next_elem = *next;

    dir_info.erase(next);

    return next_elem;
  }

  double volume(feature_decomposition* f,
		const triangular_mesh& m) {
    auto feats = collect_features(f);

    triangular_mesh mesh = m;

    double vol = 0.0;
    for (auto ft : feats) {
      triangular_mesh fm = feature_mesh(*ft);

      vtk_debug_mesh(mesh);
      vtk_debug_mesh(fm);

      boost::optional<triangular_mesh> inter =
	boolean_intersection(fm, mesh);

      if (inter) {
	vol += volume(*inter);
	auto m_res = boolean_difference(mesh, *inter);

	if (!m_res) {
	  return vol;
	} else {
	  mesh = *m_res;
	}
      }
    }

    return vol;
  }
  
  direction_process_info
  select_next_dir(std::vector<direction_process_info>& dir_info,
		  const triangular_mesh& stock) {
    DBG_ASSERT(dir_info.size() > 0);

    auto next = max_element(begin(dir_info), end(dir_info),
			    [stock](const direction_process_info& l,
				       const direction_process_info& r) {
			      return volume(l.decomp, stock) <
			      volume(r.decomp, stock);
			    });

    DBG_ASSERT(next != end(dir_info));

    direction_process_info next_elem = *next;

    dir_info.erase(next);

    return next_elem;
  }

  void clip_top_and_bottom_pairs(std::vector<direction_process_info>& dirs) {
    for (unsigned i = 0; i < dirs.size(); i++) {
      for (unsigned j = i + 1; j < dirs.size(); j++) {
	feature_decomposition* l = dirs[i].decomp;
	feature_decomposition* r = dirs[j].decomp;

	if (angle_eps(normal(l), normal(r), 180.0, 0.5)) {
	  clip_top_and_bottom_features(l, r);
	}
      }
    }
  }

  std::vector<direction_process_info>
  initial_decompositions(const triangular_mesh& stock,
			 const triangular_mesh& part,
			 const std::vector<tool>& tools,
			 const std::vector<point>& norms) {

    vector<direction_process_info> dir_info;
    for (auto n : norms) {
      feature_decomposition* decomp = build_feature_decomposition(stock, part, n);
      tool_access_info info = find_accessable_tools(decomp, tools);
      dir_info.push_back({decomp, info});
    }

    for (auto info : dir_info) {
      feature_decomposition* decomp = info.decomp;
      tool_access_info& acc_info = info.tool_info;
      delete_nodes(decomp, [acc_info](feature* f) {
	  return map_find(f, acc_info).size() == 0;
	});
    }

    clip_top_and_bottom_pairs(dir_info);

    return dir_info;
  }

  std::vector<feature*>
  collect_viable_features(feature_decomposition* decomp,
			  const volume_info_map& vol_info,
			  const fixture& fix) {
    std::vector<feature*> fs = collect_features(decomp);

    delete_if(fs, [vol_info](feature* feat) {
	return map_find(feat, vol_info).volume < 0.00001;
      });

    auto unreachable_feats =
      unreachable_features(fs, fix);

    delete_if(fs, [&unreachable_feats](feature *f) {
	return elem(f, unreachable_feats);
      });

    return fs;
  }
  
  std::vector<fixture_setup>
  select_jobs_and_features(const triangular_mesh& stock,
			   const triangular_mesh& part,
			   const fixtures& f,
			   const std::vector<tool>& tools,
			   const std::vector<point>& norms) {

    vector<direction_process_info> dir_info =
      initial_decompositions(stock, part, tools, norms);

    volume_info_map volume_inf = initial_volume_info(dir_info);

    vice v_pre = f.get_vice();
    double max_plate = max_e(f.parallel_plates(), [](const plate_height p)
			     { return p; });

    vice v(v_pre, max_plate);

    triangular_mesh current_stock = stock;
    vector<fixture_setup> cut_setups;

    double part_volume = volume(part);

    vector<feature*> all_features{};

    while (dir_info.size() > 0) {
      direction_process_info info = select_next_dir(dir_info, volume_inf);

      point n = normal(info.decomp);
      //      auto sfs = outer_surfaces(current_stock);
      auto orients = all_stable_orientations_box(current_stock, v, n);
      auto maybe_orient =
	find_orientation_by_normal_optional(orients, n);

      if (maybe_orient) {
	auto orient = *maybe_orient;
	fixture fix(orient, v);

	auto decomp = info.decomp;
	auto& acc_info = info.tool_info;

	clip_volumes(decomp, volume_inf);
	
	auto t = mating_transform(current_stock, orient, v);
	auto features = collect_viable_features(decomp, volume_inf, fix);

	if (features.size() > 0) {
	  for (auto f : features) {
	    auto vol_data = map_find(f, volume_inf);
	    cout << "delta volume = " << vol_data.volume << endl;

	    vtk_debug_feature(*f);

	    //	  vtk_debug_mesh(vol_data.dilated_mesh);

	    vtk_debug_meshes(vol_data.meshes);

	  }
	  vtk_debug_features(features);
	  concat(all_features, features);

	  cut_setups.push_back(create_setup(t, current_stock, part, features, fix, info.tool_info));

	  auto stock_res = subtract_features(current_stock, features);

	  DBG_ASSERT(stock_res.size() == 1);

	  current_stock = stock_res.front();

	  double stock_volume = volume(current_stock);
	  double volume_ratio = part_volume / stock_volume;
	
	  cout << "Part volume  = " << part_volume << endl;
	  cout << "Stock volume = " << stock_volume << endl;
	  cout << "part / stock = " << volume_ratio << endl;

	  vtk_debug_mesh(current_stock);

	  if (volume_ratio > 0.999) { break; }
	}
      }
    }

    double stock_volume = volume(current_stock);
    double volume_ratio = part_volume / stock_volume;
	
    cout << "Part volume  = " << part_volume << endl;
    cout << "Stock volume = " << stock_volume << endl;
    cout << "part / stock = " << volume_ratio << endl;

    vtk_debug_features(all_features);

    // TODO: Tighten this tolerance once edge features are supportedo
    if (volume_ratio <= 0.99) {

      vtk_debug_mesh(part);
      vtk_debug_mesh(current_stock);

      DBG_ASSERT(false);
    }

    return cut_setups;
  }

  std::vector<fixture_setup> plan_jobs(const triangular_mesh& stock,
				       const triangular_mesh& part,
				       const fixtures& f,
				       const std::vector<tool>& tools) {
    vector<surface> surfs = outer_surfaces(stock);

    DBG_ASSERT(surfs.size() == 6);

    vector<point> norms;
    for (auto ax : surfs) {
      point n = ax.face_orientation(ax.front());
      norms.push_back(n);
    }

    DBG_ASSERT(norms.size() == 6);


    return select_jobs_and_features(stock, part, f, tools, norms);
  }
  
}
