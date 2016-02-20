#include "geometry/point.h"
#include "synthesis/toolpath.h"

namespace gca {
  
  // bool continuous_cuts(const cut& last_cut,
  // 		       const cut& next_cut,
  // 		       double max_orientation_change) {
  //   if (!within_eps(last_cut.end, next_cut.start)) {
  //     return false;
  //   }
  //   double theta = angle_between(last_cut.final_orient(), next_cut.initial_orient());
  //   return theta <= max_orientation_change;
  // }

  struct not_continuous_checker {
    double max_orientation_change;
    
    not_continuous_checker(double pmax_orientation_change) :
      max_orientation_change(pmax_orientation_change) {}
    
    bool operator()(const cut&last_cut, const cut& next_cut) {
      if (!within_eps(last_cut.end, next_cut.start)) {
	return true;
      }
      double theta = angle_between(last_cut.final_orient(), next_cut.initial_orient());
      return theta > max_orientation_change;
    }
  };
  
  void collect_adjacent_cuts(const vector<cut*>& cuts,
			     vector<cut*>& cut_group,
			     unsigned i,
			     double max_orientation_change) {
    cut_group.push_back(cuts[i]);
    if (cuts.size() == i - 1) {
      return;
    }
    unsigned j = i;
    not_continuous_checker cts(max_orientation_change);
    while (j < cuts.size() - 1) {
      cut* last_cut = cuts[j];
      cut* next_cut = cuts[j + 1];
      if (cts(*last_cut, *next_cut)) {
	break;
      }
      cut_group.push_back(cuts[j]);
      j++;
    }
  }

  void group_adjacent_cuts(const vector<cut*>& cuts,
			   vector<cut_group>& cut_groups,
			   double max_orientation_change) {
    unsigned i = 0;
    while (i < cuts.size()) {
      cut_group cut_g;
      collect_adjacent_cuts(cuts, cut_g, i, max_orientation_change);
      assert(cut_g.size() > 0);
      cut_groups.push_back(cut_g);
      i += cut_g.size();
    }
  }

}
