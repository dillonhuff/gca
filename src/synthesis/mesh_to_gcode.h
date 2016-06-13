#ifndef MESH_TO_GCODE_H
#define MESH_TO_GCODE_H

#include "gcode/gcode_program.h"
#include "geometry/surface.h"
#include "geometry/triangular_mesh.h"
#include "synthesis/vice.h"
#include "synthesis/tool.h"
#include "synthesis/workpiece.h"

namespace gca {

  ostream& operator<<(ostream& out, const workpiece& w);

  enum axis { X_AXIS, Y_AXIS, Z_AXIS, A_AXIS, B_AXIS, C_AXIS };

  std::vector<surface> outer_surfaces(const triangular_mesh& m);

  std::vector<gcode_program> mesh_to_gcode(const triangular_mesh& m,
					   const vice v,
					   const vector<tool>& tools,
					   const workpiece w_dims);

  void remove_SA_surfaces(const std::vector<surface>& surfaces,
			  std::vector<index_t>& indices);

  void classify_part_surfaces(std::vector<surface>& part_surfaces,
			      const workpiece workpiece_mesh);

  workpiece align_workpiece(const std::vector<surface>& part_surfaces,
			    const workpiece w);
  

  std::vector<triangular_mesh>
  part_arrangements(const triangular_mesh& part_mesh, const vice v);

  std::vector<gcode_program>
  workpiece_clipping_programs(const workpiece aligned_workpiece,
			      const triangular_mesh& part_mesh,
			      const std::vector<tool>& tools,
			      const vice v);

  std::vector<std::pair<triangular_mesh, surface_list>>
  part_arrangements(const triangular_mesh& part_mesh,
		    const vector<surface>& part_ss,
		    const vice v);

  void cut_secured_meshes(const std::vector<std::pair<triangular_mesh, surface_list>>& meshes,
			  std::vector<gcode_program>& progs,
			  const vice v,
			  const std::vector<tool>& tools);
  
}

#endif
