#ifndef GCA_FABRICATION_PLAN_H
#define GCA_FABRICATION_PLAN_H

#include "gcode/gcode_program.h"
#include "geometry/rigid_arrangement.h"
#include "synthesis/vice.h"
#include "backend/tool.h"
#include "backend/toolpath.h"
#include "synthesis/workpiece.h"

namespace gca {

  class fixtures {
  protected:
    vice v;
    std::vector<plate_height> par_plates;
    
  public:

    fixtures(const vice& p_v)
      : v(p_v) {}
    
    fixtures(const vice& p_v,
	     const std::vector<plate_height>& p_par_plates)
      : v(p_v), par_plates(p_par_plates) {}
    
    inline const vice& get_vice() const { return v; }
    inline const std::vector<plate_height>& parallel_plates() const
    { return par_plates; }
  };

  struct fabrication_inputs {
    fixtures f;
    std::vector<tool> tools;
    std::vector<workpiece> w;

    fabrication_inputs(const fixtures& p_f,
		       const std::vector<tool>& p_tools,
		       const workpiece& p_w) :
      f(p_f), tools(p_tools), w{p_w} {}

    fabrication_inputs(const fixtures& p_f,
		       const std::vector<tool>& p_tools,
		       const std::vector<workpiece>& p_w) :
      f(p_f), tools(p_tools), w(p_w) {}

  };

  class fabrication_setup {
  protected:
    rigid_arrangement a;
    std::vector<toolpath> tps;

  public:
    vice v;

    fabrication_setup(const triangular_mesh& m,
		      const vice& p_v,
		      const std::vector<toolpath>& p_tps)
      : v(p_v), tps(p_tps) {
      a.insert("part", m);
    }

    fabrication_setup(const rigid_arrangement& p_a,
		      const vice& p_v,
		      const std::vector<toolpath>& p_tps)
      : a(p_a), v(p_v), tps(p_tps) {}

    const triangular_mesh& part_mesh() const { return a.mesh("part"); }

    const rigid_arrangement& arrangement() const { return a; }

    const std::vector<toolpath>& toolpaths() const { return tps; }

    template<typename F>
    gcode_program gcode_for_toolpaths(F f) const {
      return build_gcode_program("Surface cut", toolpaths(), f);
    }

  };

  class fabrication_plan {
  protected:
    const triangular_mesh* final_part;
    std::vector<fabrication_setup> fab_steps;
    std::vector<fabrication_plan*> custom_fixes;

  public:
    fabrication_plan(const std::vector<fabrication_setup>& p_fab_steps)
      : final_part(nullptr), fab_steps(p_fab_steps), custom_fixes{} {}

    fabrication_plan(const std::vector<fabrication_setup>& p_fab_steps,
		     const std::vector<fabrication_plan*>& p_custom_fixes)
      : final_part(nullptr), fab_steps(p_fab_steps), custom_fixes(p_custom_fixes) {}

    fabrication_plan(const triangular_mesh* p_final_part,
		     const std::vector<fabrication_setup>& p_fab_steps,
		     const std::vector<fabrication_plan*>& p_custom_fixes)
      : final_part(p_final_part), fab_steps(p_fab_steps), custom_fixes(p_custom_fixes) {}

    const triangular_mesh* final_part_mesh() const { return final_part; }
    const std::vector<fabrication_setup>& steps() const { return fab_steps; }
    const std::vector<fabrication_plan*>& custom_fixtures() const
    { return custom_fixes; }
  };

  fabrication_inputs current_fab_inputs(const workpiece& workpiece_dims);

  std::vector<tool> current_tools();

}

#endif
