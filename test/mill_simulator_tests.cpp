#include <cmath>

#include "analysis/gcode_to_cuts.h"
#include "catch.hpp"
#include "system/arena_allocator.h"
#include "geometry/line.h"
#include "simulators/mill_tool.h"
#include "simulators/region.h"
#include "simulators/sim_mill.h"
#include "synthesis/circular_arc.h"
#include "synthesis/linear_cut.h"
#include "synthesis/safe_move.h"
#include "system/file.h"

namespace gca {
  
  double square(double d) { return d*d; }

  TEST_CASE("Mill simulator") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Run empty program") {
      vector<cut*> lines;
      region r(10, 10, 10, 0.01);
      cylindrical_bit t(1);
      double actual = simulate_mill(lines, r, t);
      REQUIRE(actual == 0.0);
    }

    SECTION("One G1 move down") {
      region r(10, 10, 10, 0.01);
      r.set_height(0, 10, 0, 10, 10);
      r.set_machine_x_offset(5);
      r.set_machine_y_offset(5);
      double tool_diameter = 1.0;
      cylindrical_bit t(tool_diameter);
      vector<cut*> lines{linear_cut::make(point(0, 0, 10), point(0, 0, 5))};
      double actual = simulate_mill(lines, r, t);
      double tr = tool_diameter / 2.0;
      double correct_volume = M_PI*tr*tr*5;
      cout << "-- Correct: " << correct_volume << endl;
      cout << "-- Actual: " << actual << endl;
      REQUIRE(within_eps(actual, correct_volume, 0.05));
    }

    SECTION("Simulation") {
      region r(5, 5, 5, 0.01);
      r.set_machine_x_offset(1);
      r.set_machine_y_offset(3);
      double tool_diameter = 1.0;
      cylindrical_bit t(tool_diameter);
      
      SECTION("Move through whole workpiece") {
    	vector<cut*> lines{linear_cut::make(point(0, 0, 3), point(3, 0, 3))};
      	r.set_height(2, 3, 2, 4, 5);
      	double actual = simulate_mill(lines, r, t);
      	double correct_volume = tool_diameter*2*1;
      	cout << "-- Correct: " << correct_volume << endl;
      	cout << "-- Actual: " << actual << endl;
      	REQUIRE(within_eps(actual, correct_volume, 0.05));
      }

      SECTION("Move through whole workpiece then move back and stop") {
    	vector<cut*> lines{linear_cut::make(point(0, 0, 3), point(3, 0, 3)),
    	    linear_cut::make(point(3, 0, 3), point(0, 0, 3))};
      	r.set_height(2, 3, 2, 4, 5);
      	double actual = simulate_mill(lines, r, t);
      	double correct_volume = tool_diameter*2*1;
      	cout << "-- Correct: " << correct_volume << endl;
      	cout << "-- Actual: " << actual << endl;
      	REQUIRE(within_eps(actual, correct_volume, 0.05));
      }

      SECTION("Diagonal cut over nothing") {
    	vector<cut*> lines{linear_cut::make(point(0, 0, 3), point(3, -0.1, 3))};
    	r.set_height(2, 3, 2, 4, 1);
    	double actual = simulate_mill(lines, r, t);
    	double correct_volume = 0.0;
    	cout << "-- Correct: " << correct_volume << endl;
    	cout << "-- Actual: " << actual << endl;
    	REQUIRE(within_eps(actual, correct_volume, 0.05));
      }

    }

    SECTION("Push down, then make a circle") {
      region r(5, 5, 5, 0.005);
      r.set_height(0, 5, 0, 5, 4.0);
      r.set_machine_x_offset(0);
      r.set_machine_y_offset(0);
      double tool_diameter = 0.125;
      double tool_radius = tool_diameter / 2.0;
      cylindrical_bit t(tool_diameter);

      point p0(3, 3, 5);
      point p1(3, 3, 2);
      point p2(4, 2, 2);
      point p3(3, 1, 2);
      point p4(2, 2, 2);

      point center(3, 2, 2);
      
      cut* push_down = linear_cut::make(p0, p1);
      cut* q1 = circular_arc::make(p1, p2, center - p1, CLOCKWISE, XY);
      cut* q2 = circular_arc::make(p2, p3, center - p2, CLOCKWISE, XY);
      cut* q3 = circular_arc::make(p3, p4, center - p3, CLOCKWISE, XY);
      cut* q4 = circular_arc::make(p4, p1, center - p4, CLOCKWISE, XY);
      
      SECTION("Just push down") {
	vector<cut*> cuts{push_down};
	double actual = simulate_mill(cuts, r, t);
	double correct = M_PI*tool_radius*tool_radius*(4.0 - 2.0);
	cout << "-- Correct = " << correct << endl;
	cout << "-- Actual = " << actual << endl;
	REQUIRE(within_eps(actual, correct, 0.005));
      }

      SECTION("Push down and draw circle of radius 1") {
	vector<cut*> cuts{push_down, q1, q2, q3, q4};
	double actual = simulate_mill(cuts, r, t);
	double correct = 2*M_PI*square(1 + tool_radius) - 2*M_PI*square(1 - tool_radius);
	cout << "-- Correct = " << correct << endl;
	cout << "-- Actual = " << actual << endl;
	REQUIRE(within_eps(actual, correct, 0.05));
      }
    }

    SECTION("Safe move above the workpiece removes nothing") {
      region r(5, 5, 5, 0.005);
      double z_max = 0.499;
      r.set_height(0, 5, 0, 5, z_max);
      r.set_machine_x_offset(0);
      r.set_machine_y_offset(0);
      double tool_diameter = 0.125;
      double tool_radius = tool_diameter / 2.0;
      cylindrical_bit t(tool_diameter);
      auto c = safe_move::make(point(1, 1, 0.5), point(1, 2, 0.5));
      double volume_removed = update_cut(*c, r, t);
      REQUIRE(volume_removed == 0.0);
    }
  }

  TEST_CASE("Inferring safe height") {
    arena_allocator a;
    set_system_allocator(&a);
    
    double tool_diameter = 0.125;
    double tool_radius = tool_diameter / 2.0;
    cylindrical_bit t(tool_diameter);
    vector<vector<cut*>> paths{{}};
    double safe_height = 2.5;
    auto c1 = safe_move::make(point(1, 1, safe_height), point(1, 2, safe_height));
    paths.front().push_back(c1);
    auto c2 = linear_cut::make(point(1, 2, 0.5), point(1, 2, -0.5));
    paths.front().push_back(c2);
    auto r = set_up_region(paths, tool_diameter);
    double volume_removed = update_cut(*c1, r, t);
    REQUIRE(volume_removed == 0.0);
  }

  TEST_CASE("Material removing safe moves extracted from actual program 2") {
    arena_allocator a;
    set_system_allocator(&a);

    string dir_name = "/Users/dillon/CppWorkspace/gca/test/nc-files/BottomALBottom2.NCF";
    std::ifstream td(dir_name);
    std::string str((std::istreambuf_iterator<char>(td)),
		    std::istreambuf_iterator<char>());
    vector<block> p = lex_gprog(str);
    vector<vector<cut*>> paths;
    auto res = gcode_to_cuts(p, paths);
    assert(res == GCODE_TO_CUTS_SUCCESS);
    double tool_diameter = 0.125;
    cylindrical_bit t(tool_diameter);
    auto r = set_up_region(paths, tool_diameter);
    auto c = safe_move::make(point(-0.2191, -1.813, 2.5), point(-0.3893, 0.2266, 2.5));
    double volume_removed = update_cut(*c, r, t);
    if (c->is_safe_move() && !within_eps(volume_removed, 0.0)) {
      if (is_horizontal(c)) {
	cout << *c << endl;
	cout << "CUT INFO" << endl;
	cout << "Execution time: " << cut_execution_time_seconds(c) << endl;
	cout << "Volume removed: " << volume_removed << endl;
	cout << "IS HORIZONTAL" << endl;
      }
    }
    REQUIRE(volume_removed == 0.0);
  }

}
