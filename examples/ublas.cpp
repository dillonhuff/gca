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

boost::numeric::ublas::vector<double>
to_vector(const point p) {
  boost::numeric::ublas::vector<double> v(3);
  v(0) = p.x;
  v(1) = p.y;
  v(2) = p.z;
  return v;
}

boost::numeric::ublas::vector<double>
plane_basis_displacement(const boost::numeric::ublas::matrix<double>& r,
			 const point u1, const point u2, const point u3,
			 const point q1, const point q2, const point q3,
			 const point p1, const point p2, const point p3) {
  boost::numeric::ublas::vector<double> uv1 = to_vector(u1);
  boost::numeric::ublas::vector<double> uv2 = to_vector(u2);
  boost::numeric::ublas::vector<double> uv3 = to_vector(u3);

  boost::numeric::ublas::vector<double> qv1 = to_vector(q1);
  boost::numeric::ublas::vector<double> qv2 = to_vector(q2);
  boost::numeric::ublas::vector<double> qv3 = to_vector(q3);

  boost::numeric::ublas::vector<double> pv1 = to_vector(p1);
  boost::numeric::ublas::vector<double> pv2 = to_vector(p2);
  boost::numeric::ublas::vector<double> pv3 = to_vector(p3);

  boost::numeric::ublas::vector<double> s(3);
  s(0) = inner_prod(qv1, uv1) - inner_prod(prod(r, pv1), uv1);
  s(1) = inner_prod(qv2, uv2) - inner_prod(prod(r, pv2), uv2);
  s(2) = inner_prod(qv3, uv3) - inner_prod(prod(r, pv3), uv3);

  boost::numeric::ublas::matrix<double> u(3, 3);
  u(0, 0) = uv1(0);
  u(0, 1) = uv1(1);
  u(0, 2) = uv1(2);

  u(1, 0) = uv2(0);
  u(1, 1) = uv2(1);
  u(1, 2) = uv2(2);

  u(2, 0) = uv3(0);
  u(2, 1) = uv3(1);
  u(2, 2) = uv3(2);

  auto u_inv = inverse(u);

  return prod(u_inv, s);
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

  surface ap = find_surface_by_normal(shifted_surfs, point(0, -1, 0));
  surface bp = find_surface_by_normal(shifted_surfs, point(1, 0, 0));
  surface cp = find_surface_by_normal(shifted_surfs, point(0, 0, 1));

  triangle apt = ap.face_triangle(ap.index_list().front());
  triangle bpt = bp.face_triangle(bp.index_list().front());
  triangle cpt = cp.face_triangle(cp.index_list().front());

  boost::numeric::ublas::matrix<double> r = plane_basis_rotation(at, apt,
								 bt, bpt,
								 ct, cpt);

  std::cout << "Matrix r = " << std::endl;
  std::cout << r << std::endl;

  boost::numeric::ublas::vector<double> d =
    plane_basis_displacement(r,
			     apt.normal, bpt.normal, cpt.normal,
			     apt.v1, bpt.v1, cpt.v1,
			     at.v1, bt.v1, ct.v1);

  
  std::cout << "Displacement d = " << std::endl;
  std::cout << d << std::endl;

  cout << "pt at.v1 = " << endl;
  auto pt = to_vector(at.v1);
  cout << pt << endl;

  cout << "Result" << endl;
  cout << prod(r, pt) + d << endl;
  // int n = 3;
  // const matrix<double> a = 3 * identity_matrix<double>(n);

  // std::cout << a << std::endl;
  // matrix<double> a_inv = inverse(a);
  // std::cout << a_inv << std::endl;
}
