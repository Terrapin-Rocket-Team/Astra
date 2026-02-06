#include "Matrix.h"

namespace astra
{

    // Default constructor
    Matrix::Matrix() : rows(0), cols(0), array(nullptr) {}

    // Allocating constructor: Creates a zero-initialized matrix
    Matrix::Matrix(uint8_t rows, uint8_t cols) : rows(rows), cols(cols)
    {
        if (rows > 0 && cols > 0) {
            this->array = new double[rows * cols](); // () initializes to 0.0
        } else {
            this->array = nullptr;
        }
    }

    // Constructor from array
    // NOW PERFORMS A DEEP COPY. Safe to use with stack arrays or transient data.
    Matrix::Matrix(uint8_t rows, uint8_t cols, const double *data) : rows(rows), cols(cols)
    {
        if (rows > 0 && cols > 0 && data != nullptr) {
            this->array = new double[rows * cols];
            std::memcpy(this->array, data, rows * cols * sizeof(double));
        } else {
            this->array = nullptr;
        }
    }

    // Destructor
    Matrix::~Matrix()
    {
        delete[] this->array;
    }

    // Copy Constructor
    Matrix::Matrix(const Matrix &other) : rows(other.rows), cols(other.cols)
    {
        if (other.rows > 0 && other.cols > 0) {
            this->array = new double[rows * cols];
            std::memcpy(this->array, other.array, rows * cols * sizeof(double));
        } else {
            this->array = nullptr;
        }
    }

    // Assignment Operator
    Matrix &Matrix::operator=(const Matrix &other)
    {
        if (this != &other)
        {
            delete[] this->array; // Clean up old memory
            
            this->rows = other.rows;
            this->cols = other.cols;
            
            if (other.rows > 0 && other.cols > 0) {
                this->array = new double[this->rows * this->cols];
                std::memcpy(this->array, other.array, this->rows * this->cols * sizeof(double));
            } else {
                this->array = nullptr;
            }
        }
        return *this;
    }

    // Move Constructor (Crucial for performance: A = B * C)
    // "Steals" the pointer from the temporary object 'other'
    Matrix::Matrix(Matrix &&other) noexcept 
        : rows(other.rows), cols(other.cols), array(other.array)
    {
        other.rows = 0;
        other.cols = 0;
        other.array = nullptr;
    }

    // Move Assignment Operator
    Matrix &Matrix::operator=(Matrix &&other) noexcept
    {
        if (this != &other)
        {
            delete[] this->array; // Free our current memory

            // Steal their memory
            this->rows = other.rows;
            this->cols = other.cols;
            this->array = other.array;

            // Nullify them
            other.rows = 0;
            other.cols = 0;
            other.array = nullptr;
        }
        return *this;
    }

    uint8_t Matrix::getRows() const { return this->rows; }
    uint8_t Matrix::getCols() const { return this->cols; }
    double *Matrix::getArr() const { return this->array; }

    double Matrix::get(uint8_t i, uint8_t j) const
    {
        // Add bounds checking if desired
        return this->array[i * this->cols + j];
    }

    Matrix Matrix::operator*(const Matrix &other) const
    {
        return this->multiply(other);
    }

    Matrix Matrix::operator*(double scalar) const
    {
        return this->multiply(scalar);
    }

    Matrix Matrix::multiply(const Matrix &other) const
    {
        if (this->cols != other.rows)
        {
            LOGE("Matrix Mult Dim Error: %d vs %d. Returning Copy.", cols, other.rows);
            return *this; 
        }

        // Create the result matrix (allocates its own memory)
        Matrix result(this->rows, other.cols);

        for (int i = 0; i < this->rows; ++i)
        {
            for (int j = 0; j < other.cols; ++j)
            {
                double sum = 0;
                for (int k = 0; k < this->cols; ++k)
                {
                    sum += this->array[i * this->cols + k] * other.array[k * other.cols + j];
                }
                result(i, j) = sum;
            }
        }

        return result; // Move constructor will handle the return efficiently
    }

    Matrix Matrix::multiply(double scalar) const
    {
        Matrix result(this->rows, this->cols);
        for (int i = 0; i < this->rows * this->cols; ++i)
        {
            result.array[i] = this->array[i] * scalar;
        }
        return result;
    }

    Matrix Matrix::operator+(const Matrix &other) const
    {
        return this->add(other);
    }

    Matrix Matrix::add(const Matrix &other) const
    {
        if (this->rows != other.rows || this->cols != other.cols)
        {
            LOGE("Matrix Add Dim Error. Returning Copy.");
            return *this;
        }

        Matrix result(this->rows, this->cols);
        for (int i = 0; i < this->rows * this->cols; ++i)
        {
            result.array[i] = this->array[i] + other.array[i];
        }
        return result;
    }

    Matrix Matrix::operator-(const Matrix &other) const
    {
        return this->subtract(other);
    }

    Matrix Matrix::subtract(const Matrix &other) const
    {
        if (this->rows != other.rows || this->cols != other.cols)
        {
            LOGE("Matrix Sub Dim Error. Returning Copy.");
            return *this;
        }

        Matrix result(this->rows, this->cols);
        for (int i = 0; i < this->rows * this->cols; ++i)
        {
            result.array[i] = this->array[i] - other.array[i];
        }
        return result;
    }

    Matrix Matrix::T() const
    {
        return this->transpose();
    }

    Matrix Matrix::transpose() const
    {
        Matrix result(this->cols, this->rows);
        for (int i = 0; i < this->rows; ++i)
        {
            for (int j = 0; j < this->cols; ++j)
            {
                result(j, i) = this->array[i * this->cols + j];
            }
        }
        return result;
    }

    void Matrix::luDecompositionWithPartialPivoting(double *A, int *pivot, int n) const
    {
        // Implementation remains same, just marked const function
        for (int i = 0; i < n; ++i) pivot[i] = i;

        for (int i = 0; i < n; ++i)
        {
            double maxVal = std::abs(A[i * n + i]);
            int maxRow = i;
            for (int k = i + 1; k < n; ++k)
            {
                if (std::abs(A[k * n + i]) > maxVal)
                {
                    maxVal = std::abs(A[k * n + i]);
                    maxRow = k;
                }
            }

            if (maxVal == 0.0) LOGE("Matrix Singular!");

            // Swap rows
            for (int k = 0; k < n; ++k) std::swap(A[i * n + k], A[maxRow * n + k]);
            std::swap(pivot[i], pivot[maxRow]);

            for (int j = i + 1; j < n; ++j)
            {
                A[j * n + i] /= A[i * n + i];
                for (int k = i + 1; k < n; ++k)
                {
                    A[j * n + k] -= A[j * n + i] * A[i * n + k];
                }
            }
        }
    }

    void Matrix::solveLU(double *A, int *pivot, double *b, double *x, int n) const
    {
        // Forward subst
        for (int i = 0; i < n; ++i)
        {
            x[i] = b[pivot[i]];
            for (int j = 0; j < i; ++j) x[i] -= A[i * n + j] * x[j];
        }
        // Backward subst
        for (int i = n - 1; i >= 0; --i)
        {
            for (int j = i + 1; j < n; ++j) x[i] -= A[i * n + j] * x[j];
            x[i] /= A[i * n + i];
        }
    }

    Matrix Matrix::inv() const { return this->inverse(); }

    Matrix Matrix::inverse() const
    {
        if (this->rows != this->cols)
        {
            LOGE("Matrix Inverse Error: Non-square. Returning Copy.");
            return *this;
        }

        int n = this->rows;
        Matrix result(n, n); // Holds the inverse

        // Helper buffers (still using new/delete internally, but contained)
        double *A = new double[n * n];
        std::memcpy(A, this->array, n * n * sizeof(double));
        
        int *pivot = new int[n];
        double *b = new double[n];
        double *temp = new double[n];

        luDecompositionWithPartialPivoting(A, pivot, n);

        for (int i = 0; i < n; ++i)
        {
            std::fill(b, b + n, 0.0);
            b[i] = 1.0;
            solveLU(A, pivot, b, temp, n);
            
            // Fill result matrix column-by-column
            for (int j = 0; j < n; ++j)
            {
                result(j, i) = temp[j];
            }
        }

        delete[] A;
        delete[] pivot;
        delete[] b;
        delete[] temp;

        return result;
    }

    double Matrix::trace() const
    {
        if (rows != cols) return 0.0;
        double tr = 0;
        for (int i = 0; i < rows; ++i) tr += array[i * cols + i];
        return tr;
    }

    Matrix Matrix::ident(uint8_t n)
    {
        Matrix result(n, n); // Zeroed by default
        for (int i = 0; i < n; ++i)
        {
            result(i, i) = 1.0;
        }
        return result;
    }

    void Matrix::disp() const
    {
        // Debug display logic here
    }

    // Accessors
    double &Matrix::operator()(uint8_t row, uint8_t col)
    {
        if (row < rows && col < cols) return array[row * cols + col];
        return array[0]; // Fail-safe (risky but matches original intent)
    }

    const double &Matrix::operator()(uint8_t row, uint8_t col) const
    {
        if (row < rows && col < cols) return array[row * cols + col];
        return array[0];
    }
}