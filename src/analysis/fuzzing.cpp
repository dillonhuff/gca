#include "analysis/fuzzing.h"

#include "analysis/gcode_to_cuts.h"
#include "gcode/circular_arc.h"
#include "gcode/circular_helix_cut.h"
#include "gcode/linear_cut.h"
#include "gcode/safe_move.h"
#include "system/file.h"
#include "utils/algorithm.h"

namespace gca {

  template<typename F>
  void apply_to_gprograms(const string& dn, F f) {

    auto func = [&f](const string& dir_name) {
      if (ends_with(dir_name, ".NCF")) {
	cout << dir_name << endl;
	std::ifstream t(dir_name);
	std::string str((std::istreambuf_iterator<char>(t)),
			std::istreambuf_iterator<char>());
	vector<block> p = lex_gprog(str);
	cout << "NUM BLOCKS: " << p.size() << endl;
	f(p, dir_name);
      }
    };
    read_dir(dn, func);
  
  }
  
  bool program_in_HAAS_travel(const std::vector<std::vector<cut*> >& paths) {
    double HAAS_x_travel = 20;
    double HAAS_y_travel = 16;
    double HAAS_z_travel = 20;

    box bound = bound_paths(paths);

    if ((bound.x_len() > HAAS_x_travel) ||
	(bound.y_len() > HAAS_y_travel) ||
	(bound.z_len() > HAAS_z_travel)) {
      cout << "Box" << endl;
      cout << bound << endl;
      cout << "goes beyond HAAS bounds" << endl;

      return true;
    }

    return false;
  }

  bool program_in_GCA_travel(const std::vector<std::vector<cut*> >& paths) {
    double emco_f1_x_travel = 8;
    double emco_f1_y_travel = 6;
    double emco_f1_z_travel = 8;

    box bound = bound_paths(paths);

    if ((bound.x_len() > emco_f1_x_travel) ||
	(bound.y_len() > emco_f1_y_travel) ||
	(bound.z_len() > emco_f1_z_travel)) {
      cout << "Box" << endl;
      cout << bound << endl;
      cout << "goes beyond emco_f1 bounds" << endl;

      return true;
    }

    return false;
  }

  double fRand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  }

  point out_of_bounds_point(const double range) {
    double v = range;
    double x = fRand(0, v);
    v = v - x;
    double y = fRand(0, v);
    v = v - y;
    double z = v;
    return point(x, y, z);
  }

  int random_int(const int min, const int max) {
    return min + (rand() % (int)(max - min + 1));
  }

  point random_point(const point l, const point r) {
    box bb = bound_positions({l, r});

    double x = fRand(bb.x_min, bb.x_max);
    double y = fRand(bb.y_min, bb.y_max);
    double z = fRand(bb.z_min, bb.z_max);

    return point(x, y, z);
  }

  cut* random_cut_to_from(const point start, const point end) {
    int cut_index = random_int(0, 1);
    if (cut_index == 0) {
      return new (allocate<linear_cut>()) linear_cut(start, end);
    } else if (cut_index == 1) {
      return new (allocate<safe_move>()) safe_move(start, end);
    } else {
      DBG_ASSERT(false);
    }
  }

  cut* random_cut_starting_at(const point start, const point bound) {
    int cut_index = random_int(0, 3);
    if (cut_index == 0) {

      return new (allocate<linear_cut>()) linear_cut(start, random_point(start, bound));

    } else if (cut_index == 1) {

      return new (allocate<safe_move>()) safe_move(start, random_point(start, bound));
    }
    else if (cut_index == 2) {

      point end = random_point(start, bound);
      end.z = start.z;
      point centroid = 0.5*(start + end);
      point center = centroid;
      direction dir = random_int(0, 1) == 1 ? COUNTERCLOCKWISE : CLOCKWISE;

      return new (allocate<circular_arc>()) circular_arc(start, end, center - start, dir, XY);
    
    } else {
      point end = random_point(start, bound);
      point centroid = 0.5*(start + end);
      point center = centroid;
      direction dir = random_int(0, 1) == 1 ? COUNTERCLOCKWISE : CLOCKWISE;//COUNTERCLOCKWISE;

      return new (allocate<circular_arc>()) circular_helix_cut(start, end, center - start, dir, XY);

    }
  }

  vector<cut*> random_cuts_to_from(const point start, const point end) {
    int max_cuts = 8;
    int num_cuts = random_int(1, max_cuts);

    point first_point = start;
    //point last_point = first_point;

    vector<cut*> cuts;
    for (int i = 0; i < num_cuts; i++) {

      cut* c;
    
      if (i == (num_cuts - 1)) {
	c = random_cut_to_from(first_point, end);
      } else {
	c = random_cut_starting_at(first_point, end);
      }

      first_point = c->get_end();
      //last_point = c->get_end();

      cuts.push_back(c);

    }
  

    return cuts;
  }

  vector<cut*> random_cut_sequence(const point start_pt, const double range) {
    point start = start_pt;
    point mid = out_of_bounds_point(range);
    point end = start_pt;

    vector<cut*> cuts = random_cuts_to_from(start, mid);
    concat(cuts, random_cuts_to_from(mid, end));
    return cuts;
  }


  bool randomly_mutate(std::vector<std::vector<cut*> >& paths) {
    if (paths.size() == 0) { return false; }

    double r = ((double) rand() / (RAND_MAX));
    if (r < 0.5) {
      cut* last = paths.back().back();
      point last_pt = last->get_end();
      vector<cut*> error_cuts = random_cut_sequence(last_pt, 100);

      concat(paths.back(), error_cuts); //.push_back(error_cut);

      return true;
    }
    return false;
  }

  void print_case_stats(const std::vector<mutated_test_case>& cases) {
    int true_positives = 0;
    int true_negatives = 0;
    int false_positives = 0;
    int false_negatives = 0;

    for (auto& c : cases) {
      if (c.introduced_error && c.found_error) {
	true_positives++;
      }

      if (c.introduced_error && !c.found_error) {
	false_negatives++;
      }

      if (!c.introduced_error && c.found_error) {
	false_positives++;
      }
    
      if (!c.introduced_error && !c.found_error) {
	true_negatives++;
      }

    }

    cout << "False positives = " << false_positives << endl;
    cout << "False negatives = " << false_negatives << endl;
    cout << "True positives  = " << true_positives << endl;
    cout << "True negatives  = " << true_negatives << endl;
  }

  void test_mutated_cases_HAAS(const std::string& dir_name) {
    std::vector<mutated_test_case> cases;

    apply_to_gprograms(dir_name, [&cases](const vector<block>& p, const string& file_name) {
	vector<vector<cut*>> paths;
	auto r = gcode_to_cuts(p, paths);
	if (r == GCODE_TO_CUTS_SUCCESS) {

	  bool introduced_error =
	    randomly_mutate(paths);

	  bool any_travel_errors = program_in_HAAS_travel(paths);

	  if (!introduced_error) {
	    cases.push_back({paths, false, any_travel_errors});
	  } else {
	    cases.push_back({paths, true, any_travel_errors});
	  }
	}
      });

    print_case_stats(cases);
  }

  void test_mutated_cases_GCA(const std::string& dir_name) {
    std::vector<mutated_test_case> cases;

    apply_to_gprograms(dir_name, [&cases](const vector<block>& p, const string& file_name) {
	vector<vector<cut*>> paths;
	auto r = gcode_to_cuts(p, paths);
	if (r == GCODE_TO_CUTS_SUCCESS) {

	  bool introduced_error =
	    randomly_mutate(paths);

	  bool any_travel_errors = program_in_GCA_travel(paths);

	  if (!introduced_error) {
	    cases.push_back({paths, false, any_travel_errors});
	  } else {
	    cases.push_back({paths, true, any_travel_errors});
	  }
	}
      });

    print_case_stats(cases);
  }

}
