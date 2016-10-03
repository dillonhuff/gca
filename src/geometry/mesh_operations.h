#ifndef GCA_MESH_OPERATIONS_H
#define GCA_MESH_OPERATIONS_H

#include <boost/optional.hpp>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Nef_polyhedron_3.h>

#include "geometry/plane.h"
#include "geometry/triangular_mesh.h"

namespace gca {

  typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
  typedef CGAL::Polyhedron_3<Kernel>         Polyhedron;
  typedef Polyhedron::HalfedgeDS             HalfedgeDS;
  typedef CGAL::Nef_polyhedron_3<Kernel>  Nef_polyhedron;
  typedef Nef_polyhedron::Plane_3 Plane_3;
  
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

  triangular_mesh nef_to_single_trimesh(const Nef_polyhedron& nef);

  Nef_polyhedron trimesh_to_nef_polyhedron(const triangular_mesh& m);

}

#endif
