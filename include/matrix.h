// #ifndef __QS_MATRIX_H
// #define __QS_MATRIX_H

// #include <vector>
// #include <array>

// template <typename T, const unsigned rc, const unsigned cc> class QSMatrix {
//  private:
//   std::vector<std::vector<T> > mat;
//   unsigned rows;
//   unsigned cols;

//  public:
//   QSMatrix(const T& _initial);
//   QSMatrix(const QSMatrix<T, rc, cc>& rhs);
//   virtual ~QSMatrix();

//   // Operator overloading, for "standard" mathematical matrix operations                                                                                                                                                          
//   QSMatrix<T, rc, cc>& operator=(const QSMatrix<T, rc, cc>& rhs);

//   // Matrix mathematical operation, rc, ccs                                                                                                                                                                                               
//   QSMatrix<T, rc, cc> operator+(const QSMatrix<T, rc, cc>& rhs);
//   QSMatrix<T, rc, cc>& operator+=(const QSMatrix<T, rc, cc>& rhs);
//   QSMatrix<T, rc, cc> operator-(const QSMatrix<T, rc, cc>& rhs);
//   QSMatrix<T, rc, cc>& operator-=(const QSMatrix<T, rc, cc>& rhs);

//   template<const unsigned cc2>
//   QSMatrix<T, rc, cc2> operator*(const QSMatrix<T, cc, cc2>& rhs);

//   QSMatrix<T, rc, cc>& operator*=(const QSMatrix<T, rc, cc>& rhs);
//   QSMatrix<T, cc, rc> transpose();

//   // Matrix/scalar operations                                                                                                                                                                                                     
//   QSMatrix<T, rc, cc> operator+(const T& rhs);
//   QSMatrix<T, rc, cc> operator-(const T& rhs);
//   QSMatrix<T, rc, cc> operator*(const T& rhs);
//   QSMatrix<T, rc, cc> operator/(const T& rhs);

//   // Matrix/vector operations                                                                                                                                                                                                     
//   std::vector<T> operator*(const std::vector<T>& rhs);
//   std::vector<T> diag_vec();

//   // Access the individual elements                                                                                                                                                                                               
//   T& operator()(const unsigned& row, const unsigned& col);
//   const T& operator()(const unsigned& row, const unsigned& col) const;

//   // Access the row and column sizes                                                                                                                                                                                              
//   unsigned get_rows() const;
//   unsigned get_cols() const;

// };

// #include "matrix.cpp"

// #endif