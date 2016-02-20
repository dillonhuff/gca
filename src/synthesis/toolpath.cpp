#include <numeric>

#include "geometry/point.h"
#include "synthesis/toolpath.h"

namespace gca {

  struct cut_appender {
    double max_orientation_change;
    vector<cut_group>& cut_groups;
    cut_appender(double pmax_orientation_change,
		 vector<cut_group>& pcut_groups) :
      max_orientation_change(pmax_orientation_change), cut_groups(pcut_groups) {}

    bool continuous(const cut* last_cut, const cut* next_cut) {
      if (!within_eps(last_cut->end, next_cut->start)) {
      	return false;
      }
      double theta = angle_between(last_cut->final_orient(), next_cut->initial_orient());
      return theta <= max_orientation_change;
    }
    
    void operator()(cut* ct) {
      if (cut_groups.size() == 0 ||
	  !continuous(cut_groups.front().back(), ct)) {
	cut_group cgs;
	cut_groups.push_back(cgs);
      }
      cut_groups.back().push_back(ct);
    }
  };

  void group_adjacent_cuts(const vector<cut*>& cuts,
			   vector<cut_group>& cut_groups,
			   double max_orientation_change) {
    cut_appender ca(max_orientation_change, cut_groups);
    size_t old = cut_groups.size();
    for_each(cuts.begin(), cuts.end(), ca);
  }

}
