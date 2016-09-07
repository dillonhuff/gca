#ifndef GCA_CUT_H
#define GCA_CUT_H

#include <cassert>

#include "analysis/machine_state.h"
#include "gcode/value.h"
#include "geometry/box.h"
#include "geometry/line.h"
#include "geometry/parametric_curve.h"
#include "geometry/point.h"
#include "gcode/machine.h"
#include "utils/check.h"

namespace gca {

  struct cut {
  protected:
    parametric_curve c;
    
  public:
    machine_settings settings;
    tool_name tool_no;
    
    cut(point s, point e) : c(line(s, e)), tool_no(NO_TOOL) {}
    cut(point s, point e, tool_name t) : c(line(s, e)), tool_no(t) {}

    inline value* get_spindle_speed() const { return settings.spindle_speed; }
    inline value* get_feedrate() const { return settings.feedrate; }
    inline point get_start() const { return c.value(0.0); }
    inline point get_end() const { return c.value(1.0); }
    inline point value_at(double t) const { return c.value(t); }
    inline double length() const { return (get_end() - get_start()).len(); }
    inline value* get_tool_number() const { return settings.active_tool; }

    inline void set_spindle_speed(value* v) { settings.spindle_speed = v; }
    inline void set_feedrate(value* v) { settings.feedrate = v; }
    inline void set_tool_number(value* v) { settings.active_tool = v; }
    inline void set_start(point p) { c = parametric_curve(line(p, get_end())); }
    inline void set_end(point p) { c = parametric_curve(line(get_start(), p)); }
    
    virtual inline bool is_safe_move() const { return false; }
    virtual inline bool is_linear_cut() const { return false; }
    virtual inline bool is_circular_arc() const { return false; }
    virtual inline bool is_circular_helix_cut() const { return false; }
    virtual inline bool is_hole_punch() const { return false; }

    virtual point final_orient() const { DBG_ASSERT(false); }
    virtual point initial_orient() const { DBG_ASSERT(false); }
    
    virtual bool operator==(const cut& other) const = 0;

    virtual cut* shift(point shift) const {
      cut* new_c = copy();
      new_c->c = c.shift(shift);
      return new_c;
    }

    virtual cut* scale(double s) const {
      cut* new_c = copy();
      new_c->c = c.scale(s);
      return new_c;
    }

    virtual cut* scale_xy(double s) const {
      cut* new_c = copy();
      new_c->c = c.scale_xy(s);
      return new_c;
    }

    virtual cut* copy() const = 0;
    virtual void print(ostream& other) const = 0;
  };

  ostream& operator<<(ostream& stream, const cut& c);
  ostream& operator<<(ostream& stream, const vector<cut*>& c);

  bool cmp_cuts(const cut* l, const cut* r);
  bool same_cut_properties(const cut& l, const cut& r);

  double infer_safe_height(const vector<vector<cut*>>& paths);
  double infer_material_height(const vector<vector<cut*>>& paths, double offset);
  box path_bounds(const vector<cut*>& path);
  box bound_paths(const vector<vector<cut*>>& paths);
  bool is_vertical(const cut* c);
  bool is_horizontal(const cut* c);
  bool is_prismatic(vector<cut*>& path);
  // TODO: Make this account for cut shape
  double cut_execution_time_minutes(const cut* c);
  double cut_execution_time_seconds(const cut* c);
  double execution_time_minutes(const vector<cut*>& path);
  int get_active_tool_no(const vector<cut*>& path);
  double get_spindle_speed(const vector<cut*>& path);
  double get_spindle_speed(const cut* c);
}


#endif
