#pragma once

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <utility>

#include "geometry/surface.h"

namespace gca {

  class surface_graph {
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS> graph;
    typedef std::pair<int, int> graph_edge;

  };

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

  struct mandatory_complex {
    vector<point> legal_access_dirs;
    vector<surface> surfs;
  };

  struct proto_setup {
    plane locating_plane;

    vector<vector<surface> > mandatory_complexes;
    vector<surface> mandatory_access;
    vector<surface> unrestricted;

    inline point access_direction() const { return -1*locating_plane.normal(); }

    inline bool is_empty() const {
      return (mandatory_complexes.size() == 0) &&
	(mandatory_access.size() == 0) &&
	(unrestricted.size() == 0);
    }
  };

  surface_milling_constraints
  build_surface_milling_constraints(const triangular_mesh& part);

  boost::optional<std::vector<proto_setup> >
  surface_plans(const triangular_mesh& part);

  std::vector<surface> select_profile(const triangular_mesh& part);

}
