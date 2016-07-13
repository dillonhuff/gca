#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace boost::numeric::ublas;

int main() {
  int n = 3;
  matrix<double> a = 3 * identity_matrix<double>(n);

  std::cout << a << std::endl;
  matrix<double> a_inv = identity_matrix<double>(a.size1());
  permutation_matrix<size_t> pm(a.size1());
  int res = lu_factorize(a, pm);
  if (!res) {
    lu_substitute(a, pm, a_inv);
    std::cout << a_inv << std::endl;
    std::cout << prod(a, a_inv) << std::endl;
  } else {
    std::cout << "Singular matrix!" << std::endl;
  }
}
