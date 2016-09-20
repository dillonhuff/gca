#include "system/parse_stl.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Nef_polyhedron_3.h>

#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

using namespace gca;

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel>         Polyhedron;
typedef Polyhedron::HalfedgeDS             HalfedgeDS;
typedef CGAL::Nef_polyhedron_3<Kernel>  Nef_polyhedron;

template <class HDS>
class build_mesh : public CGAL::Modifier_base<HDS> {
public:
  const triangular_mesh& m;
  build_mesh(const triangular_mesh& p_m) : m(p_m) {}
  
  void operator()( HDS& hds) {
    // Postcondition: hds is a valid polyhedral surface.
    CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);

    unsigned num_verts = m.vertex_indexes().size();
    unsigned num_faces = m.face_indexes().size();

    B.begin_surface( num_verts, num_faces, 6*num_faces);

    typedef typename HDS::Vertex   Vertex;
    typedef typename Vertex::Point Point;

    for (auto i : m.vertex_indexes()) {
      point p = m.vertex(i);
      B.add_vertex(Point(p.x, p.y, p.z));
    }

    for (auto i : m.face_indexes()) {
      triangle_t t = m.triangle_vertices(i);
      B.begin_facet();
      B.add_vertex_to_facet(t.v[0]);
      B.add_vertex_to_facet(t.v[1]);
      B.add_vertex_to_facet(t.v[2]);
      B.end_facet();
    }

    B.end_surface();

  }
};

int main() {

  auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/Part Studio 1 - Part 1(1).stl", 0.0001);  
  
  build_mesh<HalfedgeDS> mesh_builder(mesh);

  Polyhedron P;
  P.delegate(mesh_builder);

  Nef_polyhedron N(P);
  
  if (N.is_simple()) {
    cout << "Simple" << endl;
  } else {
    cout << "Not simple" << endl;
  }

  return 0;
}
