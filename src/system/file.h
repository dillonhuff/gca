#ifndef GCA_FILE_H
#define GCA_FILE_H

#include <cassert>
#include <ctime>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>

using namespace std;

namespace gca {

  bool ends_with(string const& value, string const& ending);

  template<typename T>
  void read_dir(const string& dir_name, T f) {
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir(dir_name.c_str())) != NULL) {
      while ((ent = readdir(dir)) != NULL) {
	string fname = ent->d_name;
	if (fname != "." && fname != "..") {
	  read_dir(dir_name + "/" + fname, f);
	}
      }
      closedir(dir);
    } else {
      f(dir_name);
    }
  }

}

#endif
