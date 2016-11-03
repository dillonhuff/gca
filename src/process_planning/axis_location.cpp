#include <unordered_map>

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


  point select_next_cut_dir(const std::vector<index_t>& millable_faces,
			    const triangular_mesh& part) {
    std::vector<index_t> inds = part.face_indexes();
    subtract(inds, millable_faces);

    return part_axis(inds, part);
  }

  std::vector<index_t>
  prismatic_millable_faces(const point n,
			   const triangular_mesh& part) {
    vector<index_t> all_millable_faces = millable_faces(n, part);

    // Just use parallel to or orthogonal_to functions?
    vector<index_t> vert_or_horiz =
      select(all_millable_faces, [n, part](const index_t& s) {
	  return all_parallel_to({s}, part, n, 10.0) ||
	  all_orthogonal_to({s}, part, n, 10.0);
	});

    return vert_or_horiz;
  }

  std::vector<point> select_cut_directions(const triangular_mesh& stock,
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

    vector<index_t> all_millable_faces;
    for (auto n : norms) {
      concat(all_millable_faces, prismatic_millable_faces(n, part));
    }

    all_millable_faces = sort_unique(all_millable_faces);

    DBG_ASSERT(all_millable_faces.size() <= part.face_indexes().size());

    unsigned num_uncut = part.face_indexes().size() - all_millable_faces.size();

    cout << "# of uncut triangles = " << num_uncut << endl;
    vtk_debug_highlight_inds(all_millable_faces, part);

    while (all_millable_faces.size() < part.face_indexes().size()) {
      point next_norm = select_next_cut_dir(all_millable_faces, part);

      cout << "next_norm = " << next_norm << endl;
      norms.push_back(next_norm);

      concat(all_millable_faces, prismatic_millable_faces(next_norm, part));
      all_millable_faces = sort_unique(all_millable_faces);

      DBG_ASSERT(all_millable_faces.size() <= part.face_indexes().size());
    }

    cout << "# of cut directions to consider = " << norms.size() << endl;

    return norms;
  }
}
