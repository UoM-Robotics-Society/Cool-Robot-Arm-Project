#ifndef LA_MATHS
#define LA_MATHS

#include <assert.h>

#include <cmath>
#include <iostream>

namespace LA {

const double Pi = 3.14159265358979323846;

typedef int length_t;
template <length_t L, typename T>
struct vec;
template <length_t R, length_t C, typename T>
struct mat;

// Non-Vector and Non-Matrix Functions
template <typename T>
T min(T a, T b) {
    return a < b ? a : b;
}

template <typename T>
T max(T a, T b) {
    return a > b ? a : b;
}

template <typename T>
T abs(T a) {
    return a < 0 ? -a : a;
}

template <int N, typename T>
struct vec {
   private:
    T m[N];

   public:
    static length_t len() { return N; }
    static length_t width() { return 1; }
    static length_t height() { return N; }
    static length_t columns() { return 1; }
    static length_t rows() { return N; }
    static length_t size() { return N; }

    // --- Constructors --- //
    vec(T scalar = 0) {
        for (int i = 0; i < this->size(); i++) {
            this->operator[](i) = scalar;
        }
    }

    vec(const T arr[N]) {
        for (int i = 0; i < this->size(); i++) {
            this->operator[](i) = arr[N];
        }
    }

    vec(std::initializer_list<T> arr) {
        assert(arr.size() <= N);
        std::copy(arr.begin(), arr.end(), this->m);
    }

    // --- Unary Selector Functions --- //

    // Index Operator
    T& operator[](int i) {
        if (i > this->size() || i < 0) {
            throw std::out_of_range("index out of bounds");
        }
        return this->m[i];
    }

    T const& operator[](int i) const {
        if (i > this->size() || i < 0) {
            throw std::out_of_range("index out of bounds");
        }
        return this->m[i];
    }

    // --- Unary Arithmetic Operator Functions --- //

    vec<N, T>& operator=(const vec<N, T>& v) {
        for (int i = 0; i < this->size(); i++) {
            this->operator[](i) = v[i];
        }
        return *this;
    }

    vec<N, T>& operator=(const T arr[N]) {
        for (int i = 0; i < this->size(); i++) {
            this->operator[](i) = arr[i];
        }
        return *this;
    }

    // Addition and assign
    vec<N, T>& operator+=(T scalar) {
        for (int i = 0; i < this->size(); i++) {
            this->operator[](i) += scalar;
        }
        return *this;
    }
    vec<N, T>& operator+=(const vec<N, T>& v) {
        for (int i = 0; i < this->size(); i++) {
            this->operator[](i) += v.operator[](i);
        }
        return *this;
    }

    // Subtract and assign
    vec<N, T>& operator-=(T scalar) {
        for (int i = 0; i < this->size(); i++) {
            this->operator[](i) -= scalar;
        }
        return *this;
    }
    vec<N, T>& operator-=(const vec<N, T>& v) {
        for (int i = 0; i < this->size(); i++) {
            this->operator[](i) -= v.operator[](i);
        }
        return *this;
    }

    // Scalar multiplication
    vec<N, T>& operator*=(T scalar) {
        for (int i = 0; i < this->size(); i++) {
            this->operator[](i) *= scalar;
        }
        return *this;
    }
    // Component-wise multiplication
    vec<N, T>& operator*=(const vec<N, T>& v) {
        for (int i = 0; i < this->size(); i++) {
            this->operator[](i) *= v[i];
        }
        return *this;
    }

    // Scalar division
    vec<N, T>& operator/=(T scalar) {
        for (int i = 0; i < this->size(); i++) {
            this->operator[](i) /= scalar;
        }
        return *this;
    }
    // Component-wise division
    vec<N, T>& operator/=(const vec<N, T>& v) {
        for (int i = 0; i < this->size(); i++) {
            if (v[i] == 0) {
                throw "ERROR: Divide by 0";
            }
            this->operator[](i) /= v[i];
        }
        return *this;
    }

    // Pre-Fix Increment
    vec<N, T>& operator++() {
        for (int i = 0; i < this->size(); i++) {
            ++this->operator[](i);
        }
        return *this;
    }
    // Pre-Fix Decrement
    vec<N, T>& operator--() {
        for (int i = 0; i < this->size(); i++) {
            --this->operator[](i);
        }
        return *this;
    }
    // Post-Fix Increment
    vec<N, T> operator++(int) {
        for (int i = 0; i < this->size(); i++) {
            this->operator[](i)++;
        }
        return *this;
    }

    // Post-Fix Decrement
    vec<N, T> operator--(int) {
        for (int i = 0; i < this->size(); i++) {
            this->operator[](i)--;
        }
        return *this;
    }

    // ---  Nullary Functions --- //

    // int len() {
    //     return N;
    // }

    // int size() {
    //     return N;
    // }

    // int height() {
    //     return N;
    // }

    // int width() {
    //     return 1;
    // }
};

// Convert 1x1 vector to scalar
template <typename T>
T as_scalar(vec<1, T> const& v) {
    return v[0];
}

// Get length of vector
template <int N, typename T>
double length(vec<N, T> const& v) {
    double t = 0.0;
    for (int i = 0; i < v.size(); i++) {
        t += v[i] * v[i];
    }
    return sqrt(t);
}

// Normalise vector to length 1
template <int N, typename T>
vec<N, T> normalise(vec<N, T> const& v) {
    double l = LA::length(v);
    if (l == 0.0) {
        return v;
    }
    vec<N, T> total;
    for (int i = 0; i < v.size(); i++) {
        total[i] = v[i] / l;
    }
    return total;
}

// --- Binary Arithmetic Operator Functions --- //

// Addition
template <int N, typename T>
vec<N, T> operator+(vec<N, T> const& v, T scalar) {
    vec<N, T> total;
    for (int i = 0; i < v.len(); i++) {
        total[i] = v[i] + scalar;
    }
    return total;
}
template <int N, typename T>
vec<N, T> operator+(T scalar, vec<N, T> const& v) {
    vec<N, T> total;
    for (int i = 0; i < v.len(); i++) {
        total[i] = scalar + v[i];
    }
    return total;
}
template <int N, typename T>
vec<N, T> operator+(vec<N, T> const& v1, vec<N, T> const& v2) {
    vec<N, T> total;
    for (int i = 0; i < v1.len(); i++) {
        total[i] = v1[i] + v2[i];
    }
    return total;
}

// Negative
template <int N, typename T>
vec<N, T> operator-(vec<N, T> const& v) {
    vec<N, T> total;
    for (int i = 0; i < v.len(); i++) {
        total[i] = -v[i];
    }
    return total;
}

// Subtraction
template <int N, typename T>
vec<N, T> operator-(vec<N, T> const& v, T scalar) {
    vec<N, T> total;
    for (int i = 0; i < v.len(); i++) {
        total[i] = v[i] - scalar;
    }
    return total;
}
template <int N, typename T>
vec<N, T> operator-(T scalar, vec<N, T> const& v) {
    vec<N, T> total;
    for (int i = 0; i < v.len(); i++) {
        total[i] = scalar - v[i];
    }
    return total;
}
template <int N, typename T>
vec<N, T> operator-(vec<N, T> const& v1, vec<N, T> const& v2) {
    vec<N, T> total;
    for (int i = 0; i < v1.len(); i++) {
        total[i] = v1[i] - v2[i];
    }
    return total;
}

// Scalar multiplication
template <int N, typename T>
vec<N, T> operator*(vec<N, T> const& v, T scalar) {
    vec<N, T> total;
    for (int i = 0; i < v.len(); i++) {
        total[i] = v[i] * scalar;
    }
    return total;
}
template <int N, typename T>
vec<N, T> operator*(T scalar, vec<N, T> const& v) {
    vec<N, T> total;
    for (int i = 0; i < v.len(); i++) {
        total[i] = scalar * v[i];
    }
    return total;
}
// Component-wise multiplication
template <int N, typename T>
vec<N, T> operator*(vec<N, T> const& v1, vec<N, T> const& v2);

// Scalar division
template <int N, typename T>
vec<N, T> operator/(vec<N, T> const& v, T scalar) {
    if (scalar == 0) {
        throw "ERROR: Divide by 0";
    }
    vec<N, T> total;
    for (int i = 0; i < v.len(); i++) {
        total[i] = v[i] / scalar;
    }
    return total;
}
template <int N, typename T>
vec<N, T> operator/(T scalar, vec<N, T> const& v) {
    vec<N, T> total;
    for (int i = 0; i < v.len(); i++) {
        if (v[i] == 0) {
            throw "ERROR: Divide by 0";
        }
        total[i] = scalar / v[i];
    }
    return total;
}
// Component-wise division
template <int N, typename T>
vec<N, T> operator/(vec<N, T> const& v1, vec<N, T> const& v2) {
    vec<N, T> total;
    for (int i = 0; i < v1.len(); i++) {
        if (v2[i] == 0) {
            throw "ERROR: Divide by 0";
        }
        total[i] = v1[i] / v2[i];
    }
    return total;
}

template <int N, typename T>
vec<N, T> operator%(vec<N, T> const& v, T scalar) {
    vec<N, T> total;
    for (int i = 0; i < v.len(); i++) {
        total[i] = v[i] % scalar;
    }
    return total;
}

template <int N, typename T>
vec<N, T> operator%(T scalar, vec<N, T> const& v) {
    vec<N, T> total;
    for (int i = 0; i < v.len(); i++) {
        total[i] = scalar % v[i];
    }
    return total;
}

template <int N, typename T>
vec<N, T> operator%(vec<N, T> const& v1, vec<N, T> const& v2) {
    vec<N, T> total;
    for (int i = 0; i < v1.len(); i++) {
        total[i] = v1[i] % v2[i];
    }
    return total;
}

template <int N, typename T>
vec<N, T> operator^(vec<N, T> const& v, T scalar) {
    vec<N, T> total;
    for (int i = 0; i < v.len(); i++) {
        total[i] = v[i] ^ scalar;
    }
    return total;
}

template <int N, typename T>
vec<N, T> operator^(T scalar, vec<N, T> const& v) {
    vec<N, T> total;
    for (int i = 0; i < v.len(); i++) {
        total[i] = scalar ^ v[i];
    }
    return total;
}

template <int N, typename T>
vec<N, T> operator^(vec<N, T> const& v1, vec<N, T> const& v2) {
    vec<N, T> total;
    for (int i = 0; i < v1.len(); i++) {
        total[i] = v1[i] ^ v2[i];
    }
    return total;
}

// Equivalence Comparators
template <int N, typename T, int M, typename U>
bool operator==(vec<N, T> const& v1, vec<M, U> const& v2) {
    if (v1.len() != v2.len()) {
        return false;
    }
    for (int i = 0; i < v1.len(); i++) {
        if (v1[i] != v2[i]) {
            return false;
        }
    }
    return true;
}

template <int N, typename T>
bool operator!=(vec<N, T> const& v1, vec<N, T> const& v2) {
    if (v1 == v2) {
        return false;
    } else {
        return true;
    }
}

/// ---------------------------------------------------- ///

template <int N, int M, typename T>
struct mat {
    typedef mat<N, M, T> type;
    typedef mat<M, N, T> transpose_type;
    typedef T value_type;
    typedef vec<N, T> col_type;
    typedef vec<M, T> row_type;

   private:
    col_type data[M];

   public:
    static length_t len() { return M; }
    static length_t width() { return M; }
    static length_t height() { return N; }
    static length_t columns() { return M; }
    static length_t rows() { return N; }
    static length_t size() { return N * M; }

    col_type& operator[](int i) {
        if (i > this->width() || i < 0) {
            throw std::out_of_range("index out of bounds");
        }
        return this->data[i];
    }
    col_type const& operator[](int i) const {
        if (i > this->width() || i < 0) {
            throw std::out_of_range("index out of bounds");
        }
        return this->data[i];
    }

    // -- Constructors --

    // Creates a matrix filled with scalar
    mat(T scalar) {
        for (int i = 0; i < this->width(); i++) {
            this->operator[](i) = vec<N, T>(scalar);
        }
    }

    // Creates a matrix with the leading diagonal filled with 0s (identity)
    mat() : mat(0) {
        for (int i = 0; i < min(this->width(), this->height()); i++) {
            this->operator[](i)[i] = 1;
        }
    }

    mat(std::initializer_list<col_type> arr) {
        assert(arr.size() <= N);
        std::copy(arr.begin(), arr.end(), this->data);
    }

    mat(std::initializer_list<std::initializer_list<T>> arg) {
        assert((int)arg.size() == this->height() &&
               "ERROR: Invalid input (width)");
        int i = 0;
        int j = 0;
        for (std::initializer_list<T> list : arg) {
            assert((int)list.size() == this->width() &&
                   "ERROR: Invalid input (height)");
            i = 0;
            for (T t : list) {
                this->operator[](i)[j] = t;
                i += 1;
            }
            j += 1;
        }
    }

    // template<int I, int J, typename U>
    // template<typename std::enable_if<(N <= I && J <= J), bool> = true>
    // mat(mat<I, J, U> const& m);

    mat<N, M, T>& operator=(mat<N, M, T> const& m) {
        for (int i = 0; i < this->width(); i++) {
            this->data[i] = m[i];
        }
        return *this;
    }

    template <typename U>
    mat<N, M, T>& operator+=(U scalar) {
        for (int i = 0; i < this->width(); i++) {
            this->data[i] += scalar;
        }
        return *this;
    }
    template <typename U>
    mat<N, M, T>& operator+=(mat<N, M, U> const& m) {
        for (int i = 0; i < this->width(); i++) {
            this->data[i] += m[i];
        }
        return *this;
    }

    template <typename U>
    mat<N, M, T>& operator-=(U scalar) {
        for (int i = 0; i < this->width(); i++) {
            this->data[i] -= scalar;
        }
        return *this;
    }
    template <typename U>
    mat<N, M, T>& operator-=(mat<N, M, U> const& m) {
        for (int i = 0; i < this->width(); i++) {
            this->data[i] -= m[i];
        }
        return *this;
    }

    template <typename U>
    mat<N, M, T>& operator*=(U scalar) {
        for (int i = 0; i < this->width(); i++) {
            this->data[i] *= scalar;
        }
        return *this;
    }
    template <typename U>
    mat<N, M, T>& operator*=(mat<N, M, U> const& m) {
        float f = 0.0f;
        for (int i = 0; i < this->width(); i++) {
            for (int j = 0; j < this->height(); j++) {
                f = 0.0f;
                for (int k = 0; k < this->width(); k++) {
                    f += this->operator[](k)[j] * m[i][k];
                }
                this->operator[](j)[i] = f;
            }
        }
        return *this;
    }

    template <typename U>
    mat<N, M, T>& operator/=(U scalar) {
        for (int i = 0; i < this->width(); i++) {
            this->operator[](i) /= scalar;
        }
        return *this;
    }
    template <typename U>
    mat<N, M, T>& operator/=(mat<N, M, U> const& m) {
        return *this *= inverse(m);
    }

    mat<N, M, T>& operator++() {
        for (int i = 0; i < this->width(); i++) {
            ++this->data[i];
        }
        return *this;
    }
    mat<N, M, T>& operator--() {
        for (int i = 0; i < this->width(); i++) {
            --this->data[i];
        }
        return *this;
    }

    mat<N, M, T> operator++(int) {
        for (int i = 0; i < this->width(); i++) {
            this->data[i]++;
        }
        return *this;
    }
    mat<N, M, T> operator--(int) {
        for (int i = 0; i < this->width(); i++) {
            this->data[i]--;
        }
        return *this;
    }
};

// --- Unary Operator --- ///

// Convert 1x1 matrix to scalar
template <typename T>
T as_scalar(mat<1, 1, T> const& m) {
    return m[0][0];
}

// Inverse of a square matrix
template <int N, typename T>
mat<N, N, T> inverse(mat<N, N, T> const& m) {
    double det = determinant(m);
    if (det == 0) {
        throw "ERROR: cannot inverse non-singular matrix";
        return m;
    }
    mat<N, N, T> co = cofactor(m);
    mat<N, N, T> ct = transpose(co);
    return ct * (1 / det);
}

template <int N, typename T>
mat<1, N, T> transpose(vec<N, T> const& v) {
    mat<1, N, T> inv;
    for (int i = 0; i < v.height(); i++) {
        inv[i][0] = v[i];
    }
    return inv;
}

template <int N, int M, typename T>
mat<N, M, T> transpose(mat<M, N, T> const& m) {
    mat<N, M, T> inv;
    for (int i = 0; i < m.width(); i++) {
        for (int j = 0; j < m.height(); j++) {
            inv[j][i] = m[i][j];
        }
    }
    return inv;
}

template <int N, typename T>
double determinant(mat<N, N, T> a) {
    int n = a.len();
    double det = 0;
    mat<N, N, T> m = mat<N, N, T>();

    if (n < 1) {
        throw "ERROR: How the fuck?!?";
    } else if (n == 1) {
        det = a[0][0];
    } else if (n == 2) {
        det = a[0][0] * a[1][1] - a[1][0] * a[0][1];
    } else {
        det = 0;
        for (int j1 = 0; j1 < n; j1++) {
            for (int i = 1; i < n; i++) {
                int j2 = 0;
                for (int j = 0; j < n; j++) {
                    if (j == j1) continue;
                    m[i - 1][j2] = a[i][j];
                    j2++;
                }
            }
            det += pow(-1.0, j1 + 2.0) * a[0][j1] * determinant(m);
        }
    }
    return det;
}

template <int N, typename T>
mat<N, N, T> cofactor(mat<N, N, T> a) {
    int n = a.len();
    double det;
    mat<N, N, T> b, c;

    for (int j = 0; j < n; j++) {
        for (int i = 0; i < n; i++) {
            /* Form the adjoint a_ij */
            int i1 = 0;
            for (int ii = 0; ii < n; ii++) {
                if (ii == i) continue;
                int j1 = 0;
                for (int jj = 0; jj < n; jj++) {
                    if (jj == j) continue;
                    c[i1][j1] = a[ii][jj];
                    j1++;
                }
                i1++;
            }

            /* Calculate the determinate */
            det = determinant(c);

            /* Fill in the elements of the cofactor */
            b[i][j] = pow(-1.0, i + j + 2.0) * det;
        }
    }
    return b;
}

template <int N, int M, typename T>
mat<N, M, T> operator+(mat<N, M, T> const& m, T scalar) {
    mat<N, M, T> total;
    for (int i = 0; i < m.width(); i++) {
        total[i] = m[i] + scalar;
    }
    return total;
}
template <int N, int M, typename T>
mat<N, M, T> operator+(T scalar, mat<N, M, T> const& m) {
    mat<N, M, T> total;
    for (int i = 0; i < m.width(); i++) {
        total[i] = scalar + m[i];
    }
    return total;
}
template <int N, int M, typename T>
mat<N, M, T> operator+(mat<N, M, T> const& m1, mat<N, M, T> const& m2) {
    mat<N, M, T> total;
    for (int i = 0; i < m1.width(); i++) {
        total[i] = m1[i] + m2[i];
    }
    return total;
}

// Negative
template <int N, int M, typename T>
mat<N, M, T> operator-(mat<N, M, T> const& m) {
    mat<N, M, T> total;
    for (int i = 0; i < m.size(); i++) {
        total[i] = -m[i];
    }
    return total;
}

template <int N, int M, typename T>
mat<N, M, T> operator-(mat<N, M, T> const& m, T scalar) {
    mat<N, M, T> total;
    for (int i = 0; i < m.width(); i++) {
        total[i] = m[i] - scalar;
    }
    return total;
}
template <int N, int M, typename T>
mat<N, M, T> operator-(T scalar, mat<N, M, T> const& m) {
    mat<N, M, T> total;
    for (int i = 0; i < m.width(); i++) {
        total[i] = scalar - m[i];
    }
    return total;
}
template <int N, int M, typename T>
mat<N, M, T> operator-(mat<N, M, T> const& m1, mat<N, M, T> const& m2) {
    mat<N, M, T> total;
    for (int i = 0; i < m1.width(); i++) {
        total[i] = m1[i] - m2[i];
    }
    return total;
}

template <int N, int M, typename T>
mat<N, M, T> operator*(mat<N, M, T> const& m, T scalar) {
    mat<N, M, T> total;
    for (int i = 0; i < m.width(); i++) {
        total[i] = m[i] * scalar;
    }
    return total;
}

template <int N, int M, typename T>
mat<N, M, T> operator*(T scalar, mat<N, M, T> const& m) {
    mat<N, M, T> total;
    for (int i = 0; i < m.width(); i++) {
        total[i] = scalar * m[i];
    }
    return total;
}

template <int N, int M, typename T>
vec<N, T> operator*(mat<N, M, T> const& m, vec<M, T> const& v) {
    vec<N, T> total;
    for (int i = 0; i < m.height(); i++) {
        T t = 0;
        for (int k = 0; k < m.width(); k++) {
            t += v[k] * m[k][i];
        }
        total[i] = t;
    }
    return total;
}

template <int N, typename T>
mat<5, 5, T> operator*(vec<N, T> const& v, mat<1, N, T> const& m) {
    mat<5, 5, T> total;
    for (int i = 0; i < total.width(); i++) {
        for (int j = 0; j < total.height(); j++) {
            total[i][j] = v[j] * m[i][0];
        }
    }
    return total;
}

// template<int N, int M, typename T>
// mat<1, M, T> operator*(vec<N, T> const& v, mat<N, M, T> const& m) {
//     typename mat<N, M, T>::row_type total = mat<N, M, T>::row_type();
//     for (int i = 0; i < total.length(); i++) {
//         T t = 0;
//         for (int k = 0; k < total.length(); k++) {
//             t +=  m[i][k] * v[k];
//         }
//         total[i] = t;
//     }
//     return total;
// }

template <int N, int M, int O, typename T>
mat<N, O, T> operator*(mat<N, M, T> const& m1, mat<M, O, T> const& m2) {
    mat<N, O, T> total;
    for (int i = 0; i < m2.width(); i++) {
        for (int j = 0; j < m1.height(); j++) {
            T t = 0;
            for (int k = 0; k < m1.width(); k++) {
                t += m1[k][j] * m2[i][k];
            }
            total[i][j] = t;
        }
    }
    return total;
}

template <int N, int M, typename T>
mat<N, M, T> operator/(mat<N, M, T> const& m, T scalar) {
    mat<N, M, T> total;
    for (int i = 0; i < m.width(); i++) {
        total[i] = m[i] / scalar;
    }
    return total;
}

template <int N, int M, typename T>
mat<N, M, T> operator/(T scalar, mat<N, M, T> const& m) {
    mat<N, M, T> total;
    for (int i = 0; i < m.width(); i++) {
        total[i] = scalar / m[i];
    }
    return total;
}

template <int N, int M, typename T>
vec<N, T> operator/(mat<N, M, T> const& m, vec<N, T> const& v) {
    mat<M, M, T> inv = inverse(m);
    if (m == inv) {
        throw "ERROR: cannot divide using non-singular matrix";
        return v;
    }
    vec<N, T> total = vec<N, T>();
    for (int i = 0; i < total.len(); i++) {
        T t = 0;
        for (int k = 0; k < total.len(); k++) {
            t += v[k] / inv[i][k];
        }
        total[i] = t;
    }
    return total;
}

// template<int N, int M, typename T>
// typename mat<N, M, T>::row_type operator/(typename mat<N, M, T>::row_type
// const& v, mat<N, M, T> const& m) {
//     mat<M, M, T> inv = inverse(m);
//     if (m == inv) {
//         throw "ERROR: cannot divide using non-singular matrix";
//         return v;
//     }
//     typename mat<N, M, T>::row_type total = mat<N, M, T>::row_type();
//     for (int i = 0; i < total.length(); i++) {
//         T t = 0;
//         for (int k = 0; k < total.length(); k++) {
//             t +=  inv[k][i] / v[k];
//         }
//         total[i] = t;
//     }
//     return total;
// }

template <int N, int M, typename T>
mat<N, M, T> operator/(mat<N, M, T> const& m1, mat<M, M, T> const& m2) {
    mat<M, M, T> inv = inverse(m2);
    if (m2 == inv) {
        throw "ERROR: cannot divide using non-singular matrix";
        return m1;
    }
    return m1 * inv;
}

// -- Boolean operators --

template <int N, int M, typename T>
bool operator==(mat<N, M, T> const& m1, mat<N, M, T> const& m2) {
    for (int i = 0; i < m1.width(); i++) {
        if (m1[i] != m2[i]) {
            return false;
        }
    }
    return true;
}

template <int N, int M, typename T>
bool operator!=(mat<N, M, T> const& m1, mat<N, M, T> const& m2) {
    if (m1 == m2) {
        return false;
    } else {
        return true;
    }
}

// print
template <int N, typename T>
std::ostream& operator<<(std::ostream& os, vec<N, T>& obj) {
    for (int i = 0; i < obj.len(); i++) {
        os << obj[i];
    }
    return os << std::endl;
}

template <int N, int M, typename T>
std::ostream& operator<<(std::ostream& os, mat<N, M, T>& obj) {
    for (int i = 0; i < obj.len(); i++) {
        os << obj[i];
    }
    return os << std::endl;
}

template <int N, typename T>
void print(vec<N, T> const& v, bool asRow = false,
           bool hasNewlineEnd = true) {
    char delimmiter = asRow ? '\t' : '\n';
    for (int i = 0; i < v.size(); i++) {
        // std::cout << v[i] << delimmiter;
    }
    if (hasNewlineEnd) {
        // std::cout << std::endl;
    }
}

template <int N, int M, typename T>
void print(mat<N, M, T> const& m, bool hasNewline = true) {
    for (int j = 0; j < m.height(); j++) {
        for (int i = 0; i < m.width(); i++) {
            // std::cout << m[i][j] << "\t";
        }
        // std::cout << std::endl;
    }
    if (hasNewline) {
        // std::cout << std::endl;
    }
}

template <int N, typename T>
T min(vec<N, T> const& v) {
    T t = v[0];
    for (int i = 0; i < v.size(); i++) {
        t = min(t, v[i]);
    }
    return t;
}

template <int N, typename T>
T max(vec<N, T> const& v) {
    T t = v[0];
    for (int i = 1; i < v.size(); i++) {
        t = max(t, v[i]);
    }
    return t;
}

template <int N, typename T>
vec<N, T> abs(vec<N, T> const& v) {
    vec<N, T> total;
    for (int i = 0; i < v.size(); i++) {
        total[i] = abs(v[i]);
    }
    return total;
}

#ifdef LA_OPEN_GL
using vec2 = vec<2, float>;
using vec3 = vec<3, float>;
using vec4 = vec<4, float>;
using mat2 = mat<2, 2, float>;
using mat3 = mat<3, 3, float>;
using mat4 = mat<4, 4, float>;
#else
template <int N>
using vecf = vec<N, float>;
template <int N>
using vecd = vec<N, double>;
template <int N, int M>
using matf = mat<N, M, float>;
template <int N, int M>
using matd = mat<N, M, double>;
#endif

}  // namespace LA

#endif