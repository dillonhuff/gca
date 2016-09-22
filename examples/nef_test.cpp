#include "synthesis/fixture_analysis.h"
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
typedef Nef_polyhedron::Plane_3 Plane_3;

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

  auto surfs = outer_surfaces(mesh);
  workpiece w(3.0, 3.0, 3.0, ALUMINUM);
  triangular_mesh stock = align_workpiece(surfs, w);
  
  build_mesh<HalfedgeDS> mesh_builder(mesh);

  Polyhedron P;
  P.delegate(mesh_builder);

  Nef_polyhedron N(P);
  
  if (N.is_simple()) {
    cout << "Part mesh is Simple" << endl;
  } else {
    cout << "Part mesh is Not simple" << endl;
  }

  build_mesh<HalfedgeDS> stock_mesh_builder(mesh);

  Polyhedron S;
  S.delegate(stock_mesh_builder);

  Nef_polyhedron stock_nef(S);
  
  if (stock_nef.is_simple()) {
    cout << "Stock mesh is Simple" << endl;
  } else {
    cout << "Stock mesh is Not simple" << endl;
  }

  Nef_polyhedron negative_space = stock_nef - N;

  if (negative_space.is_simple()) {
    cout << "Negative space mesh is Simple" << endl;
  } else {
    cout << "Negative space mesh is Not simple" << endl;
  }

  plane top = face_plane(stock, point(0, 0, 1));
  plane mid = top.slide(-0.05);

  typedef Nef_polyhedron::Point_3 Point_3;
  typedef Nef_polyhedron::Vector_3 Vector_3;

  
  Plane_3 p(0.0, 0.0, 0.0, 0.0);

  // Plane_3 p(Point_3(mid.pt().x, mid.pt().y, mid.pt().z),
  // 	    Vector_3(mid.normal().x, mid.normal().y, mid.normal().z));

  Nef_polyhedron slice(p);

  auto sliced = negative_space - slice;

  if (sliced.is_simple()) {
    cout << "sliced mesh is Simple" << endl;
  } else {
    cout << "sliced mesh is Not simple" << endl;
  }

  return 0;
}
