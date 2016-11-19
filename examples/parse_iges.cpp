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

void parse_iges(const std::string& file_name) {
  ifstream iges_file(file_name.c_str());

  std::vector<iges_record> records = lex_records(iges_file);

  cout << "# of records = " << records.size() << endl;
  for (auto& r : records) {
    cout << "RECORD # = " << r.number << endl;
    cout << "LINE     = " << r.line << endl;
  }

  
}

int main(int argc, char* argv[]) {
  DBG_ASSERT(argc == 2);

  std::string name = argv[1];

  parse_iges(name);
}
