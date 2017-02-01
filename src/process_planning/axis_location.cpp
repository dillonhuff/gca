#include <unordered_map>

#include "feature_recognition/chamfer_detection.h"
#include "geometry/surface.h"
#include "process_planning/axis_location.h"
#include "synthesis/millability.h"
#include "synthesis/visual_debug.h"
#include "utils/check.h"

namespace gca {

  point part_axis(const triangular_mesh& m) {
    cout << "Part axis " << endl;

    vector<surface> regions = inds_to_surfaces(const_orientation_regions(m), m);

    unordered_map<surface*, double> ortho_areas;
    unordered_map<surface*, unsigned> horizontal_counts;
    for (surface& r : regions) {
      ortho_areas[&r] = 0.0;
      horizontal_counts[&r] = 0;
    }

    for (surface& ri : regions) {
      point ni = normal(ri);
      for (surface& rj : regions) {
	if (rj.parallel_to(ni, 1.0)) {
	  DBG_ASSERT(ortho_areas.find(&ri) != end(ortho_areas));

	  ortho_areas[&ri] = ortho_areas[&ri] + rj.surface_area();
	  horizontal_counts[&ri] = horizontal_counts[&ri] + 1;
	}
	
	if (rj.orthogonal_to(ni, 1.0) ||
	    rj.antiparallel_to(ni, 1.0)) {
	  DBG_ASSERT(ortho_areas.find(&ri) != end(ortho_areas));

	  ortho_areas[&ri] = ortho_areas[&ri] + rj.surface_area();
	}

      }
    }

    auto max_region =
      max_element(begin(ortho_areas),
		  end(ortho_areas),
		  [](const std::pair<surface*, double>& l,
		     const std::pair<surface*, double>& r) {
		    return l.second < r.second;
		  });

    cout << "Max area = " << max_region->second << endl;

    point n = normal(*(max_region->first));

    point neg_n = -1*n;

    unsigned horizontal_count = horizontal_counts[max_region->first];
    
    for (auto& r : regions) {
      if (angle_eps(normal(r), neg_n, 0.0, 1.0)) {
	unsigned neg_horizontal_count = horizontal_counts[&r];
	if (neg_horizontal_count > horizontal_count) {
	  return neg_n;
	}
      }
    }
    return n;

    // unsigned d = std::distance(begin(ortho_areas), max_region);

    // cout << "Max area index = " << d << endl;
    
    // return normal(regions[d]);
  }

  point part_axis(const std::vector<index_t>& viable_inds,
		  const triangular_mesh& part) {
    cout << "Part axis " << endl;

    auto vinds = viable_inds;
    auto const_regions = normal_delta_regions(vinds, part, 1.0);
    vector<surface> regions = inds_to_surfaces(const_regions, part);

    unordered_map<surface*, double> ortho_areas;
    unordered_map<surface*, unsigned> horizontal_counts;
    for (surface& r : regions) {
      ortho_areas[&r] = 0.0;
      horizontal_counts[&r] = 0;
    }

    for (surface& ri : regions) {
      point ni = normal(ri);

      for (surface& rj : regions) {

	if (rj.parallel_to(ni, 1.0)) {

	  DBG_ASSERT(ortho_areas.find(&ri) != end(ortho_areas));

	  ortho_areas[&ri] = ortho_areas[&ri] + rj.surface_area();
	  horizontal_counts[&ri] = horizontal_counts[&ri] + 1;
	}

	if (rj.orthogonal_to(ni, 1.0) ||
	    rj.antiparallel_to(ni, 1.0)) {

	  DBG_ASSERT(ortho_areas.find(&ri) != end(ortho_areas));

	  ortho_areas[&ri] = ortho_areas[&ri] + rj.surface_area();
	}

      }
    }

    auto max_region =
      max_element(begin(ortho_areas),
		  end(ortho_areas),
		  [](const std::pair<surface*, double>& l,
		     const std::pair<surface*, double>& r) {
		    return l.second < r.second;
		  });

    cout << "Max area = " << max_region->second << endl;

    point n = normal(*(max_region->first));

    point neg_n = -1*n;

    unsigned horizontal_count = horizontal_counts[max_region->first];

    for (auto& r : regions) {
      if (angle_eps(normal(r), neg_n, 0.0, 1.0)) {
	unsigned neg_horizontal_count = horizontal_counts[&r];
	if (neg_horizontal_count > horizontal_count) {
	  return neg_n;
	}
      }
    }

    return n;

  }

  point part_axis(const std::vector<point>& possible_dirs,
		  const std::vector<index_t>& viable_inds,
		  const triangular_mesh& part) {
    cout << "Part axis " << endl;

    auto vinds = viable_inds;
    auto const_regions = normal_delta_regions(vinds, part, 1.0);
    vector<surface> regions = inds_to_surfaces(const_regions, part);

    unordered_map<surface*, double> ortho_areas;
    unordered_map<surface*, unsigned> horizontal_counts;
    for (surface& r : regions) {
      ortho_areas[&r] = 0.0;
      horizontal_counts[&r] = 0;
    }

    for (surface& ri : regions) {
      point ni = normal(ri);

      for (surface& rj : regions) {

	if (rj.parallel_to(ni, 1.0)) {

	  DBG_ASSERT(ortho_areas.find(&ri) != end(ortho_areas));

	  ortho_areas[&ri] = ortho_areas[&ri] + rj.surface_area();
	  horizontal_counts[&ri] = horizontal_counts[&ri] + 1;
	}

	if (rj.orthogonal_to(ni, 1.0) ||
	    rj.antiparallel_to(ni, 1.0)) {

	  DBG_ASSERT(ortho_areas.find(&ri) != end(ortho_areas));

	  ortho_areas[&ri] = ortho_areas[&ri] + rj.surface_area();
	}

      }
    }

    auto max_region =
      max_element(begin(ortho_areas),
		  end(ortho_areas),
		  [](const std::pair<surface*, double>& l,
		     const std::pair<surface*, double>& r) {
		    return l.second < r.second;
		  });

    auto is_legal_dir = [possible_dirs](const point p) {
      return any_of(begin(possible_dirs), end(possible_dirs),
		    [p](const point d) {
		      return angle_eps(p, d, 0.0, 0.05);
		    });
    };

    point current_norm = normal(*(max_region->first));
    // Search for the first legal direction
    while (!is_legal_dir(current_norm)) {
      cout << "Current norm = " << current_norm << endl;

      ortho_areas.erase(max_region);

      DBG_ASSERT(ortho_areas.size() > 0);

      max_region = max_element(begin(ortho_areas),
			       end(ortho_areas),
			       [](const std::pair<surface*, double>& l,
				  const std::pair<surface*, double>& r) {
				 return l.second < r.second;
			       });

      current_norm = normal(*(max_region->first));
    }

    cout << "Max area = " << max_region->second << endl;

    point n = normal(*(max_region->first));

    point neg_n = -1*n;

    unsigned horizontal_count = horizontal_counts[max_region->first];

    for (auto& r : regions) {
      if (angle_eps(normal(r), neg_n, 0.0, 1.0)) {
	unsigned neg_horizontal_count = horizontal_counts[&r];
	if (neg_horizontal_count > horizontal_count) {
	  return neg_n;
	}
      }
    }

    return n;

  }
  

  point select_next_cut_dir(const std::vector<index_t>& millable_faces,
			    const triangular_mesh& part) {
    std::vector<index_t> inds = part.face_indexes();
    subtract(inds, millable_faces);

    return part_axis(inds, part);
  }

  point select_next_cut_dir(const std::vector<point>& possible_dirs,
			    const std::vector<index_t>& millable_faces,
			    const triangular_mesh& part) {
    std::vector<index_t> inds = part.face_indexes();
    subtract(inds, millable_faces);

    return part_axis(possible_dirs, inds, part);
  }

  
  std::vector<point> possible_mill_directions(const triangular_mesh& part) {
    auto surfs = outer_surfaces(part);
    std::vector<clamp_orientation> orients = all_stable_orientations(surfs);
    vector<point> dirs;
    for (auto orient : orients) {
      dirs.push_back(orient.top_normal());
    }
    return dirs;
  }

  void
  tag_freeform_surfaces(std::vector<direction_info>& norms,
			std::vector<index_t>& all_millable_faces,
			const triangular_mesh& part) {
    for (direction_info& n : norms) {
      n.search_for_freeform_features = true;

      concat(all_millable_faces, millable_faces(n.dir, part));
      all_millable_faces = sort_unique(all_millable_faces);

      if (all_millable_faces.size() == part.face_indexes().size()) {
	break;
      }
    }
  }

  std::vector<direction_info>
  select_cut_directions(const triangular_mesh& stock,
			const triangular_mesh& part,
			const fixtures& f,
			const std::vector<tool>& tools) {
    vector<surface> surfs = outer_surfaces(stock);

    DBG_ASSERT(surfs.size() == 6);

    vector<direction_info> norms;
    for (auto ax : surfs) {
      point n = ax.face_orientation(ax.front());
      norms.push_back({n, false});
    }

    DBG_ASSERT(norms.size() == 6);

    //    vector<double> angles = chamfer_angles(tools);

    vector<index_t> all_millable_faces;
    for (auto n : norms) {
      concat(all_millable_faces, prismatic_millable_faces(n.dir, part));
      concat(all_millable_faces, chamfer_faces(part, n.dir, tools));
    }

    all_millable_faces = sort_unique(all_millable_faces);

    DBG_ASSERT(all_millable_faces.size() <= part.face_indexes().size());

    //vtk_debug_highlight_inds(all_millable_faces, part);

    vector<point> possible_dirs = possible_mill_directions(part);
    delete_if(possible_dirs,
	      [norms](const point p) {
		return any_of(begin(norms), end(norms), [p](const direction_info d) {
		    return angle_eps(d.dir, p, 0.0, 0.5);
		  });
	      });

    while ((all_millable_faces.size() < part.face_indexes().size()) &&
	   (possible_dirs.size() > 0)) {

      for (auto d : possible_dirs) {
	cout << "Possible dir = " << d << endl;
      }


      auto is_legal_dir = [possible_dirs](const point p) {
	return any_of(begin(possible_dirs), end(possible_dirs),
		      [p](const point d) {
			return angle_eps(p, d, 0.0, 0.05);
		      });
      };

      vector<index_t> remaining_faces = part.face_indexes();
      subtract(remaining_faces, all_millable_faces);
      bool some_legal_dir = false;
      for (auto i : remaining_faces) {
	if (is_legal_dir(part.face_orientation(i))) {
	  some_legal_dir = true;
	  cout << part.face_orientation(i) << " is a legal direction" << endl;
	  break;
	}
      }

      if (!some_legal_dir) { break; }

      point next_norm = select_next_cut_dir(possible_dirs, all_millable_faces, part);

      cout << "Next norm = " << next_norm << endl;
      bool viable_dir =
	any_of(begin(possible_dirs), end(possible_dirs),
	       [next_norm](const point d) {
		 return angle_eps(d, next_norm, 0.0, 0.05);
	       });

      delete_if(possible_dirs,
		[next_norm](const point p) {
		  return angle_eps(p, next_norm, 0.0, 0.5);
		});
      
      if (viable_dir) {
	cout << "next_norm = " << next_norm << endl;
	norms.push_back({next_norm, false});

	concat(all_millable_faces, prismatic_millable_faces(next_norm, part));
	concat(all_millable_faces, chamfer_faces(part, next_norm, tools));

	all_millable_faces = sort_unique(all_millable_faces);
      }

      DBG_ASSERT(all_millable_faces.size() <= part.face_indexes().size());
    }

    // Need to search for freeform surfaces
    if (all_millable_faces.size() != part.face_indexes().size()) {

      tag_freeform_surfaces(norms, all_millable_faces, part);
    }


    if (!(all_millable_faces.size() == part.face_indexes().size())) {
      cout << "# of faces          = " << part.face_indexes().size() << endl;
      cout << "# of millable faces = " << all_millable_faces.size() << endl;

      auto unmillable = part.face_indexes();
      subtract(unmillable, all_millable_faces);

      surface s(&part, unmillable);

      double eps = 0.00001;
      double unmillable_area = s.surface_area();
      cout << "Unmillable surface area = " << unmillable_area << endl;
      if (s.surface_area() <= eps) {
	return norms;
      }

      vtk_debug_highlight_inds(all_millable_faces, part);

      DBG_ASSERT(all_millable_faces.size() == part.face_indexes().size());
    }

    cout << "# of cut directions to consider = " << norms.size() << endl;

    return norms;
  }
}
