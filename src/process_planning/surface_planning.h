#pragma once

#include <memory>

#include "geometry/surface.h"

namespace gca {

  enum mill_process { FINISH_FACE_MILL,
		      FINISH_PERIPHERAL_MILL,
		      FINISH_FREEFORM };

  struct finish_constraint {
    point access_dir;
    mill_process finish_process;
  };

  struct finish_constraints {
    std::vector<finish_constraint> constraints;
  };

  class surface_milling_constraints {
  protected:
    std::vector<std::vector<surface > > scs;

    // std::vector<unique_ptr<surface> > flat_surfaces;
    // std::unordered_map<surface*, finish_constraints> surface_finish_constraints;
    
  public:

    surface_milling_constraints(const std::vector<std::vector<surface> >& p_scs) :
      scs(p_scs) {}

    bool has_unmillable_inside_corner() const { return true; }

    std::vector<std::vector<surface> >
    hard_corner_groups() const;
    
  };

  surface_milling_constraints
  build_surface_milling_constraints(const triangular_mesh& part);

}
