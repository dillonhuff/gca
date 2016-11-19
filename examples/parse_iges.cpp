#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <streambuf>
#include <vector>

#include "system/parse_iges.h"
#include "utils/check.h"

using namespace gca;
using namespace std;

struct iges_record {
  std::string line;
  int number;
};

iges_record segment_record(const std::string& record_line) {
  string line = record_line.substr(0, 73);
  string num = record_line.substr(73, 80);

  DBG_ASSERT(line.size() == 73);

  return iges_record{line, stoi(num)};
}

std::vector<iges_record> lex_records(std::ifstream& iges_file) {
  std::vector<iges_record> records;

  int record_size = 82;
  char record[record_size+1];
  record[record_size] = '\0';

  while (iges_file.read(record, record_size)) {
    string rec_str = record;

    records.push_back(segment_record(rec_str));
  }

  return records;
}

struct iges_sections {
  std::vector<iges_record> start;
  std::vector<iges_record> global;
  std::vector<iges_record> directory_entry;
  std::vector<iges_record> parameter_data;
  std::vector<iges_record> terminate;
};

iges_sections
group_records_into_sections(const std::vector<iges_record>& records) {
  vector<vector<iges_record> > sections;

  vector<iges_record> current_section;
  int current_no = 0;
  for (auto& r : records) {
    if (r.number == (current_no + 1)) {
      current_section.push_back(r);
      current_no = r.number;
    } else if (r.number == 1) {
      sections.push_back(current_section);
      current_no = 1;
      current_section = {r};
    } else {
      cout << "Bad record with number = " << r.number << endl;
      cout << "Text = " << r.line << endl;
      DBG_ASSERT(false);
    }
  }

  sections.push_back(current_section);

  cout << "# of sections = " << sections.size() << endl;

  DBG_ASSERT(sections.size() == 5);
  DBG_ASSERT(sections.back().size() == 1);

  return iges_sections{sections[0],
      sections[1],
      sections[2],
      sections[3],
      sections[4]};
}

void parse_iges(const std::string& file_name) {
  ifstream iges_file(file_name.c_str());

  std::vector<iges_record> records = lex_records(iges_file);

  cout << "# of records = " << records.size() << endl;
  // for (auto& r : records) {
  //   cout << "RECORD # = " << r.number << endl;
  //   cout << "LINE     = " << r.line << endl;
  // }

  iges_sections sections =
    group_records_into_sections(records);
}

int main(int argc, char* argv[]) {
  DBG_ASSERT(argc == 2);

  std::string name = argv[1];

  parse_iges(name);
}
