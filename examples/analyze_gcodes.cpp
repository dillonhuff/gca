#include <cassert>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>

#include "analysis/propagate_settings.h"
#include "analysis/unfold.h"
#include "core/arena_allocator.h"
#include "core/lexer.h"
#include "core/parser.h"

using namespace gca;
using namespace std;

bool is_canned_cycle(const token* t) {
  vector<token*> canned;
  canned.push_back(icode::make('G', 81));
  canned.push_back(icode::make('G', 82));
  canned.push_back(icode::make('G', 83));
  canned.push_back(icode::make('G', 84));
  canned.push_back(icode::make('G', 85));
  return count_if(canned.begin(), canned.end(), cmp_token_to(t)) > 0;
}

bool is_feedrate(const token* t) {
  if (t->tp() == ICODE) {
    const icode* ic = static_cast<const icode*>(t);
    return ic->c == 'F';
  }
  return false;
}

bool is_spindle_speed(const token* t) {
  if (t->tp() == ICODE) {
    const icode* ic = static_cast<const icode*>(t);
    return ic->c == 'S' || ic->c == 'T';
  } else if (t->tp() == COMMENT) {
    return true;
  }
  return false;
}

bool is_coord_system(const token* t) {
  if (t->tp() == ICODE) {
    const icode* ic = static_cast<const icode*>(t);
    return ic->c == 'G' &&
      (ic->v == ilit(54) ||
       ic->v == ilit(55) ||
       ic->v == ilit(56) ||
       ic->v == ilit(57) ||
       ic->v == ilit(58));
  }
  return false;
}

void print_canned_feedrate(const block& b) {
  if (count_if(b.begin(), b.end(), is_coord_system) > 0) { //is_spindle_speed) > 0) {//is_feedrate) > 0) {//is_canned_cycle) > 0) {
    cout << b << endl;
  }
}

void print_canned_cycle_feedrates(const vector<block>& p) {
  for_each(p.begin(), p.end(), print_canned_feedrate);
}

inline bool ends_with(string const& value, string const& ending) {
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

void read_dir(const string& dir_name) {
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir(dir_name.c_str())) != NULL) {
    while ((ent = readdir(dir)) != NULL) {
      string fname = ent->d_name;
      if (fname != "." && fname != "..") {
	read_dir(dir_name + "/" + fname);
      }
    }
    closedir(dir);
  } else {
    if (ends_with(dir_name, ".NCF")) {
      cout << dir_name << endl;
      std::ifstream t(dir_name);
      std::string str((std::istreambuf_iterator<char>(t)),
		      std::istreambuf_iterator<char>());
      vector<block> p = lex_gprog(str);
      cout << "NUM BLOCKS: " << p.size() << endl;
      vector<block> uf = unfold_gprog(p);
      cout << "UNFOLDED BLOCKS: " << uf.size() << endl;
      vector<block> prop = propagate_settings(uf);
      print_canned_cycle_feedrates(prop);
    }
  }
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: print-knife-hacks <gcode file path>" << endl;
    return 0;
  }

  arena_allocator a;
  set_system_allocator(&a);

  string dir_name = argv[1];
  read_dir(dir_name);
  
  // std::ifstream t("/Users/dillon/Documents//ME318-2/ME318/ME318/Term Project/Part 1/Mount Op 3.NCF");
  // std::string str((std::istreambuf_iterator<char>(t)),
  // 		  std::istreambuf_iterator<char>());
  
  // vector<token> ts = lex_gprog(str);
}
