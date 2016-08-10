#ifndef GCA_FABRICATION_PLAN_H
#define GCA_FABRICATION_PLAN_H

#include "gcode/gcode_program.h"
#include "geometry/rigid_arrangement.h"
#include "synthesis/vice.h"
#include "synthesis/tool.h"
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
    vector<tool> tools;
    workpiece w;
  };

  class fabrication_setup {
  protected:
    rigid_arrangement a;
    // triangular_mesh part;
    // std::vector<triangular_mesh*> other_meshes;

  public:
    vice v;
    gcode_program prog;

    fabrication_setup(const triangular_mesh& m,
		      const vice& p_v,
		      const gcode_program& p)
      : v(p_v), prog(p) {
      a.insert("part", m);
    }

    // TODO: Actually initialize meshes
    fabrication_setup(const rigid_arrangement& p_a,
		      const vice& p_v,
		      const gcode_program& p)
      : a(p_a), v(p_v), prog(p) {}

    const triangular_mesh& part_mesh() const { return a.mesh("part"); }

    const rigid_arrangement& arrangement() const { return a; }

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

}

#endif
