// #ifndef __QS_MATRIX_CPP
// #define __QS_MATRIX_CPP

// #include "matrix.h"

// // Parameter Constructor

// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, rowC, colC>::QSMatrix(std::initializer_list<T> l)
// {
//   rows = rowCount;
//   cols = colCount;
//   assert(l.size() == rows * cols);
//   mat.resize(rows);
//   for (unsigned i = 0; i < mat.size(); i++)
//   {
//     for(unsigned j = 0; j < cols; j ++)
//     {
//       mat.push_back(l[i * cols + j]);
//     }
//   }
// }

// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, rowC, colC>::QSMatrix( const T &_initial)
// {
//   rows = rowCount;
//   cols = colCount;
//   mat.resize(rows);
//   for (unsigned i = 0; i < mat.size(); i++)
//   {
//     mat[i].resize(cols, _initial);
//   }
//   rows = rows;
//   cols = cols;
  
// }

// // Copy Constructor
// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, rowC, colC>::QSMatrix(const QSMatrix<T, rowC, colC> &rhs)
// {
//   mat = rhs.mat;
//   rows = rhs.get_rows();
//   cols = rhs.get_cols();
// }

// // (Virtual) Destructor
// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, rowC, colC>::~QSMatrix() {}

// // Assignment Operator
// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, rowC, colC> &QSMatrix<T, rowC, colC>::operator=(const QSMatrix<T, rowC, colC> &rhs)
// {
//   if (&rhs == this)
//     return *this;

//   unsigned new_rows = rhs.get_rows();
//   unsigned new_cols = rhs.get_cols();

//   mat.resize(new_rows);
//   for (unsigned i = 0; i < mat.size(); i++)
//   {
//     mat[i].resize(new_cols);
//   }

//   for (unsigned i = 0; i < new_rows; i++)
//   {
//     for (unsigned j = 0; j < new_cols; j++)
//     {
//       mat[i][j] = rhs(i, j);
//     }
//   }
//   rows = new_rows;
//   cols = new_cols;

//   return *this;
// }

// // Addition of two matrices
// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, rowC, colC> QSMatrix<T, rowC, colC>::operator+(const QSMatrix<T, rowC, colC> &rhs)
// {
//   QSMatrix result(rows, cols, 0.0);

//   for (unsigned i = 0; i < rows; i++)
//   {
//     for (unsigned j = 0; j < cols; j++)
//     {
//       result(i, j) = this->mat[i][j] + rhs(i, j);
//     }
//   }

//   return result;
// }

// // Cumulative addition of this matrix and another
// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, rowC, colC> &QSMatrix<T, rowC, colC>::operator+=(const QSMatrix<T, rowC, colC> &rhs)
// {
//   unsigned rows = rhs.get_rows();
//   unsigned cols = rhs.get_cols();

//   for (unsigned i = 0; i < rows; i++)
//   {
//     for (unsigned j = 0; j < cols; j++)
//     {
//       this->mat[i][j] += rhs(i, j);
//     }
//   }

//   return *this;
// }

// // Subtraction of this matrix and another
// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, rowC, colC> QSMatrix<T, rowC, colC>::operator-(const QSMatrix<T, rowC, colC> &rhs)
// {
//   unsigned rows = rhs.get_rows();
//   unsigned cols = rhs.get_cols();
//   QSMatrix result(rows, cols, 0.0);

//   for (unsigned i = 0; i < rows; i++)
//   {
//     for (unsigned j = 0; j < cols; j++)
//     {
//       result(i, j) = this->mat[i][j] - rhs(i, j);
//     }
//   }

//   return result;
// }

// // Cumulative subtraction of this matrix and another
// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, rowC, colC> &QSMatrix<T, rowC, colC>::operator-=(const QSMatrix<T, rowC, colC> &rhs)
// {
//   unsigned rows = rhs.get_rows();
//   unsigned cols = rhs.get_cols();

//   for (unsigned i = 0; i < rows; i++)
//   {
//     for (unsigned j = 0; j < cols; j++)
//     {
//       this->mat[i][j] -= rhs(i, j);
//     }
//   }

//   return *this;
// }

// // Left multiplication of this matrix and another
// template <Typename T, unsigned RC1
//         , unsigned CRC12>
// template<unsigned CC2>
// QSMatrix<T, RC1, CC2> QSMatrix<T, RC1, CRC12>::operator*(const QSMatrix<T, CRC12, CC2> &rhs)
// {
//   unsigned rows = rhs.get_rows();
//   unsigned cols = rhs.get_cols();
//   QSMatrix result(rows, cols, 0.0);

//   for (unsigned i = 0; i < rows; i++)
//   {
//     for (unsigned j = 0; j < cols; j++)
//     {
//       for (unsigned k = 0; k < rows; k++)
//       {
//         result(i, j) += this->mat[i][k] * rhs(k, j);
//       }
//     }
//   }

//   return result;
// }

// // Cumulative left multiplication of this matrix and another
// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, rowC, colC> &QSMatrix<T, rowC, colC>::operator*=(const QSMatrix<T, rowC, colC> &rhs)
// {
//   QSMatrix result = (*this) * rhs;
//   (*this) = result;
//   return *this;
// }

// // Calculate a transpose of this matrix

// // TODO: FIX THIS THIS PROB DOESNT WORK
// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, colC, C> QSMatrix<T, rowC, colC>::transpose()
// {
//   QSMatrix result(rows, cols, 0.0);

//   for (unsigned i = 0; i < rows; i++)
//   {
//     for (unsigned j = 0; j < cols; j++)
//     {
//       result(i, j) = this->mat[j][i];
//     }
//   }

//   return result;
// }

// // Matrix/scalar addition
// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, rowC, colC> QSMatrix<T, rowC, colC>::operator+(const T &rhs)
// {
//   QSMatrix result(rows, cols, 0.0);

//   for (unsigned i = 0; i < rows; i++)
//   {
//     for (unsigned j = 0; j < cols; j++)
//     {
//       result(i, j) = this->mat[i][j] + rhs;
//     }
//   }

//   return result;
// }

// // Matrix/scalar subtraction
// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, rowC, colC> QSMatrix<T, rowC, colC>::operator-(const T &rhs)
// {
//   QSMatrix result(rows, cols, 0.0);

//   for (unsigned i = 0; i < rows; i++)
//   {
//     for (unsigned j = 0; j < cols; j++)
//     {
//       result(i, j) = this->mat[i][j] - rhs;
//     }
//   }

//   return result;
// }

// // Matrix/scalar multiplication
// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, rowC, colC> QSMatrix<T, rowC, colC>::operator*(const T &rhs)
// {
//   QSMatrix result(rows, cols, 0.0);

//   for (unsigned i = 0; i < rows; i++)
//   {
//     for (unsigned j = 0; j < cols; j++)
//     {
//       result(i, j) = this->mat[i][j] * rhs;
//     }
//   }

//   return result;
// }

// // Matrix/scalar division
// template <Typename T, const unsigned rowC, const unsigned colC>
// QSMatrix<T, rowC, colC> QSMatrix<T, rowC, colC>::operator/(const T &rhs)
// {
//   QSMatrix result(rows, cols, 0.0);

//   for (unsigned i = 0; i < rows; i++)
//   {
//     for (unsigned j = 0; j < cols; j++)
//     {
//       result(i, j) = this->mat[i][j] / rhs;
//     }
//   }

//   return result;
// }

// // Multiply a matrix with a vector
// template <Typename T, const unsigned rowC, const unsigned colC>
// std::vector<T> QSMatrix<T, rowC, colC>::operator*(const std::vector<T> &rhs)
// {
//   std::vector<T> result(rhs.size(), 0.0);

//   for (unsigned i = 0; i < rows; i++)
//   {
//     for (unsigned j = 0; j < cols; j++)
//     {
//       result[i] = this->mat[i][j] * rhs[j];
//     }
//   }

//   return result;
// }

// // Obtain a vector of the diagonal elements
// template <Typename T, const unsigned rowC, const unsigned colC>
// std::vector<T> QSMatrix<T, rowC, colC>::diag_vec()
// {
//   std::vector<T> result(rows, 0.0);

//   for (unsigned i = 0; i < rows; i++)
//   {
//     result[i] = this->mat[i][i];
//   }

//   return result;
// }

// // Access the individual elements
// template <Typename T, const unsigned rowC, const unsigned colC>
// T &QSMatrix<T, rowC, colC>::operator()(const unsigned &row, const unsigned &col)
// {
//   return this->mat[row][col];
// }

// // Access the individual elements (const)
// template <Typename T, const unsigned rowC, const unsigned colC>
// const T &QSMatrix<T, rowC, colC>::operator()(const unsigned &row, const unsigned &col) const
// {
//   return this->mat[row][col];
// }

// // Get the number of rows of the matrix
// template <Typename T, const unsigned rowC, const unsigned colC>
// unsigned QSMatrix<T, rowC, colC>::get_rows() const
// {
//   return this->rows;
// }

// // Get the number of columns of the matrix
// template <Typename T, const unsigned rowC, const unsigned colC>
// unsigned QSMatrix<T, rowC, colC>::get_cols() const
// {
//   return this->cols;
// }

// #endif