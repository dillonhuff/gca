#include <vector>

#include "catch.hpp"
#include "system/algorithm.h"

using namespace std;

namespace gca {

  TEST_CASE("greedy_adjacent_chains") {

    SECTION("Empty vector") {
      vector<int> v{};
      greedy_adjacent_chains(v.begin(), v.end(),
			     [](int i, int j) { return j == i + 1; });
      vector<int> correct{};
      REQUIRE(v == correct);
    }

    SECTION("One chain") {
      vector<int> v{13, 8, 14};
      greedy_adjacent_chains(v.begin(), v.end(),
			     [](int i, int j) { return j == i + 1; });
      vector<int> correct{13, 14, 8};
      REQUIRE(v == correct);
    }

    SECTION("Prefix of 3 chains") {
      vector<int> v{3, 13, 8, 4, 14};
      greedy_adjacent_chains(v.begin(), v.end(),
			     [](int i, int j) { return j == i + 1; });
      vector<int> correct{3, 4, 8, 13, 14};
      REQUIRE(v == correct);
    }
    
    SECTION("3 chains") {
      vector<int> v{1, 13, 2, 3, 8, 4, 14};
      // 1 13 2 3 8 4 14
      // 1 2 13 3 8 4 14
      // 1 2 3 13 8 4 14
      // 1 2 3 4 8 13 14
      greedy_adjacent_chains(v.begin(), v.end(),
			     [](int i, int j) { return j == i + 1; });
      vector<int> correct{1, 2, 3, 4, 8, 13, 14};
      REQUIRE(v == correct);
    }
  }

  TEST_CASE("Drop while tests") {

    SECTION("") {
      vector<int> v{};
      drop_while(v, [](int i) { return i < 0; });
      vector<int> correct;
      REQUIRE(v == correct);
    }

    SECTION("Several values") {
      vector<int> v{1, 3, 5, 2, 8, 34, 2};
      drop_while(v, [](int i) { return i < 5; });
      vector<int> correct{5, 2, 8, 34, 2};
      REQUIRE(v == correct);
    }

    SECTION("Drop the entire sequence") {
      vector<int> v{1, 3, 5, 2, 8, 34, 2};
      drop_while(v, [](int i) { return i < 500; });
      vector<int> correct;
      REQUIRE(v == correct);
    }
  }
}
