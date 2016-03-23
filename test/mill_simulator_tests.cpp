#include <cmath>

#include "catch.hpp"
#include "core/arena_allocator.h"
#include "geometry/line.h"
#include "simulators/mill_tool.h"
#include "simulators/region.h"
#include "simulators/sim_mill.h"

namespace gca {

  TEST_CASE("Mill simulator") {
    
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Run empty program") {
      vector<line> lines;
      region r(10, 10, 10, 0.01);
      cylindrical_bit t(1);
      simulate_mill(lines, r, t);
      REQUIRE(r.volume_removed() == 0.0);
    }

    SECTION("One G1 move down") {
      region r(10, 10, 10, 0.01);
      r.set_height(0, 10, 0, 10, 10);
      r.set_machine_x_offset(5);
      r.set_machine_y_offset(5);
      double tool_diameter = 1.0;
      cylindrical_bit t(tool_diameter);
      vector<line> lines{line(point(0, 0, 0), point(0, 0, 5))};
      simulate_mill(lines, r, t);
      double tr = tool_diameter / 2.0;
      double correct_volume = M_PI*tr*tr*5;
      double actual = r.volume_removed();
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
	vector<line> lines{line(point(0, 0, 2), point(3, 0, 2))};
      	r.set_height(2, 3, 2, 4, 5);
      	simulate_mill(lines, r, t);
      	double correct_volume = tool_diameter*2*1;
      	double actual = r.volume_removed();
      	cout << "-- Correct: " << correct_volume << endl;
      	cout << "-- Actual: " << actual << endl;
      	REQUIRE(within_eps(actual, correct_volume, 0.05));
      }

      SECTION("Move through whole workpiece then move back and stop") {
	vector<line> lines{line(point(0, 0, 2), point(3, 0, 2)),
	    line(point(3, 0, 2), point(0, 0, 2))};
      	r.set_height(2, 3, 2, 4, 5);
      	simulate_mill(lines, r, t);
      	double correct_volume = tool_diameter*2*1;
      	double actual = r.volume_removed();
      	cout << "-- Correct: " << correct_volume << endl;
      	cout << "-- Actual: " << actual << endl;
      	REQUIRE(within_eps(actual, correct_volume, 0.05));
      }

      SECTION("Diagonal cut over nothing") {
	vector<line> lines{line(point(0, 0, 2), point(3, -0.1, 2))};
	r.set_height(2, 3, 2, 4, 1);
	simulate_mill(lines, r, t);
	double correct_volume = 0.0;
	double actual = r.volume_removed();
	cout << "-- Correct: " << correct_volume << endl;
	cout << "-- Actual: " << actual << endl;
	REQUIRE(within_eps(actual, correct_volume, 0.05));
      }
    }
  }
}
