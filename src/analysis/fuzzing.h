#pragma once

#include "gcode/cut.h"

namespace gca {

  struct mutated_test_case {
    std::vector<std::vector<cut*> > paths;
    bool introduced_error;
    bool found_error;
  };

  void test_mutated_cases_GCA(const std::string& dir_name);

  void test_mutated_cases_HAAS(const std::string& dir_name);

  bool program_in_HAAS_travel(const std::vector<std::vector<cut*> >& paths);

  bool program_in_GCA_travel(const std::vector<std::vector<cut*> >& paths);

  void print_case_stats(const std::vector<mutated_test_case>& cases);

}
