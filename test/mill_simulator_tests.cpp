#include <cmath>

#include "catch.hpp"
#include "core/parser.h"
#include "simulators/mill_tool.h"
#include "simulators/region.h"
#include "simulators/sim_mill.h"

namespace gca {

  TEST_CASE("Mill simulator") {
    context c;

    SECTION("Run empty program") {
      gprog* p = parse_gprog(c, "");
      region r(10, 10, 10, 0.01);
      cylindrical_bit t(1);
      simulate_mill(*p, r, t);
      REQUIRE(r.volume_removed() == 0.0);
    }

    SECTION("One G1 move down") {
      gprog* p = parse_gprog(c, "G1 X0 Y0 Z5");
      region r(10, 10, 10, 0.01);
      r.set_height(0, 10, 0, 10, 10);
      r.set_machine_x_offset(5);
      r.set_machine_y_offset(5);
      double tool_diameter = 1.0;
      cylindrical_bit t(tool_diameter);
      simulate_mill(*p, r, t);
      double tr = tool_diameter / 2.0;
      double correct_volume = M_PI*tr*tr*5;
      double actual = r.volume_removed();
      cout << "-- Correct: " << correct_volume << endl;
      cout << "-- Actual: " << actual << endl;
      REQUIRE(within_eps(actual, correct_volume, 0.01));
    }

    SECTION("Move through whole workpiece") {
      gprog* p = parse_gprog(c, "G0 X0 Y0 Z2 G91 G1 X3");
      region r(5, 5, 5, 0.01);
      r.set_height(2, 3, 2, 4, 5);
      r.set_machine_x_offset(1);
      r.set_machine_y_offset(3);
      double tool_diameter = 1.0;
      cylindrical_bit t(tool_diameter);
      simulate_mill(*p, r, t);
      double correct_volume = tool_diameter*2*1;
      double actual = r.volume_removed();
      cout << "-- Correct: " << correct_volume << endl;
      cout << "-- Actual: " << actual << endl;
      REQUIRE(within_eps(actual, correct_volume, 0.05));
    }
  }
}
