#ifndef GCA_FABRICATION_PLAN_H
#define GCA_FABRICATION_PLAN_H

#include "gcode/gcode_program.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/vice.h"
#include "synthesis/tool.h"
#include "synthesis/workpiece.h"

namespace gca {

  struct fabrication_setup {
    triangular_mesh part;
    vice v;
    gcode_program prog;

    fabrication_setup(const triangular_mesh& m,
		      const vice& p_v,
		      const gcode_program& p)
      : part(m), v(p_v), prog(p) {}
  };

  class fabrication_plan {
  protected:
    std::vector<fabrication_setup> fab_steps;
    std::vector<fabrication_plan*> custom_fixes;

  public:
    fabrication_plan(const std::vector<fabrication_setup>& p_fab_steps)
      : fab_steps(p_fab_steps), custom_fixes{} {}

    fabrication_plan(const std::vector<fabrication_setup>& p_fab_steps,
		     const std::vector<fabrication_plan*>& p_custom_fixes)
      : fab_steps(p_fab_steps), custom_fixes(p_custom_fixes) {}
    
    const std::vector<fabrication_setup>& steps() const { return fab_steps; }
    const std::vector<fabrication_plan*>& custom_fixtures() const
    { return custom_fixes; }
  };

}

#endif
