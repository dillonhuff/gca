// #include <boost/numeric/ublas/lu.hpp>
// #include <boost/numeric/ublas/io.hpp>

// #include "geometry/matrix.h"
// #include "geometry/surface.h"
// #include "system/parse_stl.h"

// using namespace boost::numeric::ublas;
// using namespace gca;

// surface find_surface_by_normal(const std::vector<surface>& m_surfs,
// 			       const point n) {
//   return *(find_if(begin(m_surfs), end(m_surfs),
// 		   [n](const surface& s)
// 		   { return within_eps(s.face_orientation(s.index_list().front()), n); }));  
// }


int main() {
  // triangular_mesh m = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Box1x1x1.stl", 0.001);
  // triangular_mesh shifted =
  //   m.apply_to_vertices([](const point p)
  // 			{ return p + point(10, 3, 10); });

  // auto m_surfs = outer_surfaces(m);

  // surface a = find_surface_by_normal(m_surfs, point(0, 0, 1));
  // surface b = find_surface_by_normal(m_surfs, point(1, 0, 0));
  // surface c = find_surface_by_normal(m_surfs, point(0, 1, 0));

  // triangle at = a.face_triangle(a.index_list().front());
  // triangle bt = b.face_triangle(b.index_list().front());
  // triangle ct = c.face_triangle(c.index_list().front());

  // auto shifted_surfs = outer_surfaces(shifted);

  // surface ap = find_surface_by_normal(shifted_surfs, point(0, -1, 0));
  // surface bp = find_surface_by_normal(shifted_surfs, point(1, 0, 0));
  // surface cp = find_surface_by_normal(shifted_surfs, point(0, 0, 1));

  // triangle apt = ap.face_triangle(ap.index_list().front());
  // triangle bpt = bp.face_triangle(bp.index_list().front());
  // triangle cpt = cp.face_triangle(cp.index_list().front());

  // const boost::numeric::ublas::matrix<double> r =
  //   plane_basis_rotation(at, apt,
  // 			 bt, bpt,
  // 			 ct, cpt);

  // std::cout << "Matrix r = " << std::endl;
  // std::cout << r << std::endl;
  // std::cout << "Determinant of r = " << determinant(r) << std::endl;

  // boost::numeric::ublas::vector<double> d =
  //   plane_basis_displacement(r,
  // 			     apt.normal, bpt.normal, cpt.normal,
  // 			     apt.v1, bpt.v1, cpt.v1,
  // 			     at.v1, bt.v1, ct.v1);
  
  // std::cout << "Displacement d = " << std::endl;
  // std::cout << d << std::endl;

  // cout << "pt at.v1 = " << endl;
  // auto pt = to_vector(at.v1);
  // cout << pt << endl;

  // cout << "Result" << endl;
  // cout << prod(r, pt) + d << endl;
}
