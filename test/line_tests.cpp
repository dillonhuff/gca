#include "catch.hpp"
#include "context.h"
#include "cut.h"
#include "line.h"
#include "output.h"

namespace gca {
  
  TEST_CASE("lines_to_cuts") {
    context c;
    double cutter_width = 1.0;
    double depth = -0.05;
    double l = 10;
    vector<line> edges;
    vector<cut*> cuts;
    
    SECTION("1 cut per line") {
      edges.push_back(line(point(0, 0, depth), point(0, l, depth)));
      edges.push_back(line(point(0, l, depth), point(l, l, depth)));
      edges.push_back(line(point(l, l, depth), point(l, 0, depth)));
      edges.push_back(line(point(l, 0, depth), point(0, 0, depth)));
      cuts = lines_to_cuts(c, edges, cutter_width);
      REQUIRE(cuts.size() == 4);
    }

    SECTION("Lines are extended by cuts") {
      edges.push_back(line(point(0, 0, depth), point(0, l, depth)));
      cuts = lines_to_cuts(c, edges, cutter_width);
      double w = cutter_width / 2.0;
      cut correct(point(-w, -w, depth), point(-w, l + w, depth));
      REQUIRE(*(cuts[0]) == correct);
    }
  }

}
