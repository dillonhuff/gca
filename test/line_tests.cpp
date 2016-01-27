#include "catch.hpp"
#include "core/context.h"
#include "geometry/line.h"
#include "synthesis/cut.h"
#include "synthesis/output.h"

namespace gca {
  
  TEST_CASE("lines_to_cuts") {    
    arena_allocator a;
    set_system_allocator(&a);
    double cutter_width = 1.0;
    double depth = -0.05;
    double l = 10;
    vector<line> edges;
    vector<cut*> cuts;
    
    SECTION("2 cuts per line") {
      edges.push_back(line(point(0, 0, depth), point(0, l, depth)));
      edges.push_back(line(point(0, l, depth), point(l, l, depth)));
      edges.push_back(line(point(l, l, depth), point(l, 0, depth)));
      edges.push_back(line(point(l, 0, depth), point(0, 0, depth)));
      cuts = lines_to_cuts(edges, cutter_width);
      REQUIRE(cuts.size() == 8);
    }

    SECTION("Lines are extended by cuts") {
      edges.push_back(line(point(0, 0, depth), point(0, l, depth)));
      cuts = lines_to_cuts(edges, cutter_width);
      double w = cutter_width / 2.0;
      cut correct0(point(-w, -w, 0), point(-w, -w, depth));
      cut correct1(point(-w, -w, depth), point(-w, l + w, depth));
      REQUIRE(*(cuts[0]) == correct0);
      REQUIRE(*(cuts[1]) == correct1);
    }
  }

}
