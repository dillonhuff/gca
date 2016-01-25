#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <set>

#include "core/context.h"
#include "core/parser.h"
#include "synthesis/output.h"

using namespace gca;

int* opt2(int* path, double** dis, int cities) {
  START:
  int i, j;
  for (i = 0; i < cities - 2; i++)
  {
    for (j = i + 2; j < cities-1; j++)
    {
      double swap_length = dis[path[i]][path[j]] +
                           dis[path[i + 1]][path[j + 1]];
      double old_length = dis[path[i]][path[i + 1]] + 
                          dis[path[j]][path[j + 1]];
      if (swap_length < old_length)
      {
         for (int x = 0; x < (j - i) / 2; x++)
         {
            int temp = path[i + 1 + x];
            path[i + 1 + x] = path[j - x];
            path[j - x] = temp;
         }
         goto START;
      }
    }
  }
  return path;
}

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: gdiff <gcode_file_path>" << endl;
    return 0;
  }
  string file = argv[1];
  context c;
  gprog* p = read_file(c, file);
  vector<point> hole_locations;
  for (int i = 0; i < p->size(); i++) {
    instr* is = (*p)[i];
    if (is->is_G1()) {
      move_instr* mi = static_cast<move_instr*>(is);
      assert(mi->is_concrete());
      point pt = mi->pos();
      assert(pt.z == 0);
      hole_locations.push_back(pt);
    }
  }

  int* pts = static_cast<int*>(malloc(sizeof(int)*hole_locations.size()));
  for (int i = 0; i < hole_locations.size(); i++) {
    pts[i] = i;
  }

  double** distance = static_cast<double**>(malloc(sizeof(double*)*hole_locations.size()));
  for (int i = 0; i < hole_locations.size(); i++) {
    distance[i] = static_cast<double*>(malloc(sizeof(double*)*hole_locations.size()));
    for (int j = 0; j < hole_locations.size(); j++) {
      distance[i][j] = abs((hole_locations[i] - hole_locations[j]).len());
    }
  }

  int* res = opt2(pts, distance, hole_locations.size());
  gprog* r = c.mk_gprog();
  for (int i = 0; i < hole_locations.size(); i++) {
    point pt = hole_locations[res[i]];
    r->push_back(c.mk_G1(pt.x, pt.y, pt.z, c.mk_omitted()));
    if (i < hole_locations.size() - 1) {
      point next = hole_locations[res[i+1]];
      r->push_back(c.mk_G0(pt.x, pt.y, 0.35));
      r->push_back(c.mk_G0(next.x, next.y, 0.35));
    }
  }
  cout << *r;
  return 0;
}
