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

void print_entity_name(const int n) {
  switch(n) {
  case 100:
    cout << "CIRCULAR ARC" << endl;
    break;
  case 102:
    cout << "COMPOSITE CURVE" << endl;
    break;
  case 104:
    cout << "CONIC ARC" << endl;
    break;
  case 110:
    cout << "LINE" << endl;
    break;
  case 120:
    cout << "SURFACE OF REVOLUTION" << endl;
    break;
  case 122:
    cout << "TABULATED CYLINDER" << endl;
    break;
  case 124:
    cout << "TRANSFORMATION MATRIX" << endl;
    break;
  case 126:
    cout << "RATIONAL B-SPLINE CURVE" << endl;
    break;
  case 142:
    cout << "CURVE ON A PARAMETRIC SURFACE" << endl;
    break;
  case 144:
    cout << "TRIMMED PARAMETRIC SURFACE" << endl;
    break;
  case 314:
    cout << "COLOR DEFINITION ENTITY" << endl;
    break;
  default:
    cout << "UNKNOWN ENTITY CODE " << n << endl;
    DBG_ASSERT(false);
  }
}

void print_directory_entry_types(const std::vector<iges_record>& dirents) {
  DBG_ASSERT((dirents.size() % 2) == 0);

  for (unsigned i = 0; i < dirents.size(); i += 2) {
    string s = dirents[i].line.substr(0, 8);
    int n = stoi(s);

    print_entity_name(n);
  }

}

void parse_iges(const std::string& file_name) {
  ifstream iges_file(file_name.c_str());

  std::vector<iges_record> records = lex_records(iges_file);

  cout << "# of records = " << records.size() << endl;

  iges_sections sections =
    group_records_into_sections(records);

  print_directory_entry_types(sections.directory_entry);
}

int main(int argc, char* argv[]) {
  DBG_ASSERT(argc == 2);

  std::string name = argv[1];

  parse_iges(name);
}
