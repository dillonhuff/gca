#ifndef GCA_MESH_OPERATIONS_H
#define GCA_MESH_OPERATIONS_H

#include <boost/optional.hpp>

#include "geometry/plane.h"
#include "geometry/triangular_mesh.h"

namespace gca {

  std::vector<oriented_polygon>
  mesh_cross_section(const triangular_mesh& m,
		     const plane p);
  
  triangular_mesh
  clip_mesh(const triangular_mesh& m, const plane pl);

  triangular_mesh
  clip_mesh_exact(const triangular_mesh& m,
		  const plane pl);


  boost::optional<triangular_mesh>
  boolean_difference(const triangular_mesh& a, const triangular_mesh& b);

  boost::optional<triangular_mesh>
  boolean_intersection(const triangular_mesh& a, const triangular_mesh& b);

  std::vector<triangular_mesh>
  boolean_difference(const triangular_mesh& a,
		     const std::vector<triangular_mesh>& bs);
  
  void write_mesh_as_stl(const triangular_mesh& m,
			 const std::string& file_name);

  double volume(const triangular_mesh& m);

  std::vector<triangular_mesh>
  boolean_difference(const std::vector<triangular_mesh>& as,
		     const std::vector<triangular_mesh>& bs);

}

#endif
