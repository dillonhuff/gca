#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

#include "geometry/surface.h"
#include "system/parse_stl.h"

using namespace boost::numeric::ublas;

matrix<double> inverse(matrix<double>& a) {
  matrix<double> a_inv = identity_matrix<double>(a.size1());
  permutation_matrix<size_t> pm(a.size1());
  int res = lu_factorize(a, pm);
  if (!res) {
    lu_substitute(a, pm, a_inv);
  } else {
    std::cout << a << std::endl;
    std::cout << "Singular matrix!" << std::endl;
    assert(false);
  }
  return a_inv;
}

matrix<double> inverse(const matrix<double>& a) {
  matrix<double> b = a;
  return inverse(b);
}

using namespace gca;

surface find_surface_by_normal(const std::vector<surface>& m_surfs,
			       const point n) {
  return *(find_if(begin(m_surfs), end(m_surfs),
		   [n](const surface& s)
		   { return within_eps(s.face_orientation(s.index_list().front()), n); }));  
}

boost::numeric::ublas::matrix<double>
plane_basis_rotation(const triangle at, const triangle apt,
		     const triangle bt, const triangle bpt,
		     const triangle ct, const triangle cpt) {
  boost::numeric::ublas::matrix<double> a(3, 3);
  a(0, 0) = at.normal.x;
  a(1, 0) = at.normal.y;
  a(2, 0) = at.normal.z;

  a(0, 1) = bt.normal.x;
  a(1, 1) = bt.normal.y;
  a(2, 1) = bt.normal.z;

  a(0, 2) = ct.normal.x;
  a(1, 2) = ct.normal.y;
  a(2, 2) = ct.normal.z;

  std::cout << "Matrix a = " << std::endl;
  std::cout << a << std::endl;
  
  boost::numeric::ublas::matrix<double> b(3, 3);
  b(0, 0) = apt.normal.x;
  b(1, 0) = apt.normal.y;
  b(2, 0) = apt.normal.z;

  b(0, 1) = bpt.normal.x;
  b(1, 1) = bpt.normal.y;
  b(2, 1) = bpt.normal.z;

  b(0, 2) = cpt.normal.x;
  b(1, 2) = cpt.normal.y;
  b(2, 2) = cpt.normal.z;

  auto a_inv = inverse(a);
  return prod(b, a_inv);
}

int main() {
  triangular_mesh m = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Box1x1x1.stl", 0.001);
  triangular_mesh shifted =
    m.apply_to_vertices([](const point p)
			{ return p + point(10, 3, 10); });

  auto m_surfs = outer_surfaces(m);

  surface a = find_surface_by_normal(m_surfs, point(0, 0, 1));
  surface b = find_surface_by_normal(m_surfs, point(1, 0, 0));
  surface c = find_surface_by_normal(m_surfs, point(0, 1, 0));

  triangle at = a.face_triangle(a.index_list().front());
  triangle bt = b.face_triangle(b.index_list().front());
  triangle ct = c.face_triangle(c.index_list().front());

  auto shifted_surfs = outer_surfaces(shifted);

  surface ap = find_surface_by_normal(m_surfs, point(0, -1, 0));
  surface bp = find_surface_by_normal(m_surfs, point(1, 0, 0));
  surface cp = find_surface_by_normal(m_surfs, point(0, 0, 1));

  triangle apt = ap.face_triangle(ap.index_list().front());
  triangle bpt = bp.face_triangle(bp.index_list().front());
  triangle cpt = cp.face_triangle(cp.index_list().front());

  boost::numeric::ublas::matrix<double> r = plane_basis_rotation(at, apt,
								 bt, bpt,
								 ct, cpt);
  
  // int n = 3;
  // const matrix<double> a = 3 * identity_matrix<double>(n);

  // std::cout << a << std::endl;
  // matrix<double> a_inv = inverse(a);
  // std::cout << a_inv << std::endl;
}
