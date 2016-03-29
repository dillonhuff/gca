#ifndef GCA_CUT_H
#define GCA_CUT_H

#include <cassert>

#include "analysis/machine_state.h"
#include "core/value.h"
#include "geometry/box.h"
#include "geometry/point.h"
#include "synthesis/machine.h"

namespace gca {

  struct cut {
  private:
    point start;
    point end;
    
  public:
    machine_settings settings;
    tool_name tool_no;
    
    cut(point s, point e) : start(s), end(e), tool_no(NO_TOOL) {}
    cut(point s, point e, tool_name t) : start(s), end(e), tool_no(t) {}

    inline value* get_spindle_speed() const { return settings.spindle_speed; }
    inline value* get_feedrate() const { return settings.feedrate; }
    inline point get_start() const { return start; }
    inline point get_end() const { return end; }

    inline void set_spindle_speed(value* v) { settings.spindle_speed = v; }
    inline void set_feedrate(value* v) { settings.feedrate = v; }
    inline void set_start(point p) { start = p; }
    inline void set_end(point p) { end = p; }
    
    virtual inline bool is_safe_move() const { return false; }
    virtual inline bool is_linear_cut() const { return false; }
    virtual inline bool is_circular_arc() const { return false; }
    virtual inline bool is_circular_helix_cut() const { return false; }
    virtual inline bool is_hole_punch() const { return false; }

    virtual point final_orient() const { assert(false); }
    virtual point initial_orient() const { assert(false); }
    
    virtual bool operator==(const cut& other) const = 0;
    virtual cut* shift(point shift) const = 0;
    virtual cut* scale(double s) const = 0;
    virtual cut* scale_xy(double s) const = 0;
    virtual cut* copy() const = 0;
    virtual void print(ostream& other) const = 0;
  };

  ostream& operator<<(ostream& stream, const cut& c);
  ostream& operator<<(ostream& stream, const vector<cut*>& c);

  bool cmp_cuts(const cut* l, const cut* r);
  bool same_cut_properties(const cut& l, const cut& r);

  box path_bounds(const vector<cut*>& path);  
  bool is_vertical(const cut* c);
  bool is_horizontal(const cut* c);
  bool is_prismatic(vector<cut*>& path);
  // TODO: Make this account for cut shape
  double cut_execution_time(const cut* c);
  double execution_time(const vector<cut*>& path);
}


#endif
