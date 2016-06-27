#ifndef MESH_TO_GCODE_H
#define MESH_TO_GCODE_H

#include "gcode/gcode_program.h"
#include "geometry/surface.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/fixture_analysis.h"
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
    std::vector<gcode_program> stock_clipping_progs;
    std::vector<fabrication_setup> fab_steps;
    
  public:
    fabrication_plan(const std::vector<gcode_program>& p_stock_clipping_programs,
		     const std::vector<fabrication_setup>& p_fab_steps)
      : stock_clipping_progs(p_stock_clipping_programs),
	fab_steps(p_fab_steps){}

    const std::vector<gcode_program>& stock_clipping_programs() const
    { return stock_clipping_progs; }
    const std::vector<fabrication_setup>& steps() const { return fab_steps; }
  };

  ostream& operator<<(ostream& out, const workpiece& w);

  enum axis { X_AXIS, Y_AXIS, Z_AXIS, A_AXIS, B_AXIS, C_AXIS };

  std::vector<surface> outer_surfaces(const triangular_mesh& m);

  std::vector<gcode_program> mesh_to_gcode(const triangular_mesh& m,
					   const fixtures& v,
					   const vector<tool>& tools,
					   const workpiece w_dims);

  fabrication_plan make_fabrication_plan(const triangular_mesh& m,
					 const fixtures& v,
					 const vector<tool>& tools,
					 const workpiece w_dims);
  
  void remove_SA_surfaces(const std::vector<surface>& surfaces,
			  std::vector<index_t>& indices);

  void classify_part_surfaces(std::vector<surface>& part_surfaces,
			      const workpiece workpiece_mesh);

  workpiece align_workpiece(const std::vector<surface>& part_surfaces,
			    const workpiece w);

  std::pair<triangular_mesh, std::vector<std::vector<index_t>>>
  oriented_part_mesh(const stock_orientation& orient,
		     const surface_list& surfaces,
		     const vice v);
  

  std::vector<triangular_mesh>
  part_arrangements(const triangular_mesh& part_mesh, const vice v);

  std::vector<std::pair<triangular_mesh, surface_list>>
  part_arrangements(const triangular_mesh& part_mesh,
		    const vector<surface>& part_ss,
		    const vice v);

  void cut_secured_meshes(const std::vector<std::pair<triangular_mesh, surface_list>>& meshes,
			  std::vector<gcode_program>& progs,
			  const std::vector<tool>& tools);
  
}

#endif
