#ifndef MATRIX_H
#define MATRIX_H

#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstring> // For memcpy
#include <stdint.h>
#include "RecordData/Logging/EventLogger.h"

namespace astra
{
    class Matrix
    {
    public:
        // Default constructor (empty)
        Matrix();

        // New: Allocating constructor (creates zeroed matrix)
        Matrix(uint8_t rows, uint8_t cols);

        // Copy from array constructor
        // DEEP COPIES 'data'. Does NOT take ownership of the pointer.
        Matrix(uint8_t rows, uint8_t cols, const double *data);

        // Destructor
        ~Matrix();

        // Copy Constructor (Deep Copy)
        Matrix(const Matrix &other);

        // Assignment Operator (Deep Copy)
        Matrix &operator=(const Matrix &other);

        // Move Constructor (C++11) - Efficiently transfers ownership from temporaries
        Matrix(Matrix &&other) noexcept;

        // Move Assignment Operator (C++11)
        Matrix &operator=(Matrix &&other) noexcept;

        uint8_t getRows() const;
        uint8_t getCols() const;
        double *getArr() const; // Be careful modifying this directly
        double get(uint8_t i, uint8_t j) const;

        // Math operations
        // Note: Arguments are now const Matrix& to prevent unnecessary copying on call
        Matrix operator*(const Matrix &other) const;
        Matrix multiply(const Matrix &other) const;

        Matrix operator*(double scalar) const;
        Matrix multiply(double scalar) const;

        Matrix operator+(const Matrix &other) const;
        Matrix add(const Matrix &other) const;

        Matrix operator-(const Matrix &other) const;
        Matrix subtract(const Matrix &other) const;

        Matrix T() const;
        Matrix transpose() const;
        
        Matrix inv() const;
        Matrix inverse() const;
        
        double trace() const;
        static Matrix ident(uint8_t n);
        void disp() const;

        // Mutable access
        double &operator()(uint8_t row, uint8_t col);
        // Const access
        const double &operator()(uint8_t row, uint8_t col) const;

    private:
        uint8_t rows;
        uint8_t cols;
        double *array;

        // Helper functions remain private
        void luDecompositionWithPartialPivoting(double *A, int *pivot, int n) const;
        void solveLU(double *A, int *pivot, double *b, double *x, int n) const;
    };
} // namespace astra
#endif