#ifndef GCA_MATRIX_H
#define GCA_MATRIX_H

#include <cstdlib>

#include "geometry/point.h"

namespace gca {

  template<int I, int J>
  class matrix {
  private:
    double coeffs[I*J];
  public:

    matrix() {}
    
    matrix(double* p_coeffs) {
      for (int i = 0; i < I; i++) {
	for (int j = 0; j < J; j++) {
	  coeffs[i*J + j] = p_coeffs[i*J + j];
	}
      }
    }

    inline void set(int i, int j, double v)
    { coeffs[i*J + j] = v; }

    inline double get(int i, int j) const
    { return coeffs[i*J + j]; }
  };
  
  template<int I, int J>
  matrix<I, J> operator+(const matrix<I, J>& a, const matrix<I, J>& b) {
    matrix<I, J> sum;
    for (int i = 0; i < I; i++) {
      for (int j = 0; j < J; j++) {
	sum.set(i, j, a.get(i, j) + b.get(i, j));
      }
    }
    return sum;
  }

  template<int I, int J, int K>
  matrix<I, K> operator*(const matrix<I, J>& a, const matrix<J, K>& b) {
    matrix<I, K> prod;
    for (int k = 0; k < K; k++) {
      for (int i = 0; i < I; i++) {
	prod.set(i, k, 0.0);
	for (int j = 0; j < J; j++) {
	  prod.set(i, k, prod.get(i, k) + a.get(i, j) * b.get(j, k));
	}
      }
    }
    return prod;
  }

  template<int I, int J>
  matrix<I, J> operator*(const double c, const matrix<I, J>& a) {
    matrix<I, J> scalar_prod;
    for (int i = 0; i < I; i++) {
      for (int j = 0; j < J; j++) {
	scalar_prod.set(i, j, c*a.get(i, j));
      }
    }
    return scalar_prod;
  }
  
  template<int I, int J>
  matrix<I, J> identity() {
    double coeffs[I*J];
    for (int i = 0; i < I; i++) {
      for (int j = 0; j < J; j++) {
	if (i == j) {
	  coeffs[i*J + j] = 1.0;
	} else {
	  coeffs[i*J + j] = 0;
	}
      }
    }
    return matrix<I, J>(coeffs);
  }

  matrix<3, 3> rotate_onto(point a, point b);

  point operator*(const matrix<3, 3>& m, const point a);

}

#endif