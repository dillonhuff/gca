#include "simulators/region.h"
#include "utils/algorithm.h"

namespace gca {

  // bool same_grid_cell(const grid_update l,
  // 		      const grid_update r) {
  //   return l.cell == r.cell; //return (l.x_ind == r.x_ind) && (l.y_ind == r.y_ind);
  // }

  std::vector<grid_update> sum_updates(const std::vector<point_update>& updates) {
    vector<grid_update> total_updates;
    for (auto& update : updates) {
      for (auto& gu : update.grid_updates) {
	total_updates.push_back(gu);
      }
    }

    sort(begin(total_updates), end(total_updates),
	 [](const grid_update l, const grid_update& r) {
	   if (l.cell.x_ind < r.cell.x_ind) { return true; }
	   return l.cell.y_ind < r.cell.y_ind;
	 });

    vector<vector<grid_update> > same_location_updates =
      split_by(total_updates, [](const grid_update l, const grid_update r) {
	  return l.cell == r.cell;
	});

    vector<grid_update> summed_updates;
    for (auto& group : same_location_updates) {

      grid_cell c = group.front().cell;
      double total_height = 0.0;
      for (auto& update : group) {
	total_height += update.height_diff;
      }

      summed_updates.push_back({c, total_height});
    }

    return total_updates;
  }

  // double max_cut_depth_from_updates(const std::vector<point_update>& updates) {
  //   if (updates.size() == 0) { return -1; }
  //   vector<grid_update> total_updates = sum_updates(updates);

  //   if (total_updates.size() == 0) {
  //     return -1;
  //   }
    
  //   grid_update largest_cut = max_e(total_updates, [](const grid_update& g) {
  // 	return g.height_diff;
  //     });

  //   return largest_cut.height_diff;
  // }

  double volume_removed_in_update(const double resolution,
				  const point_update& update) {
    return volume_removed_in_updates(resolution, update.grid_updates);
  }

  double
  volume_removed_in_updates(const double resolution,
			    const std::vector<grid_update>& sum_updates) {
    double volume_removed = 0.0;

    for (auto& g : sum_updates) {
      volume_removed += g.height_diff * resolution*resolution;
    }

    return volume_removed;
  }
  
  
}
