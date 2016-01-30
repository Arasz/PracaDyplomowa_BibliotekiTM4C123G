#ifndef MATRIXCALCULATION_H_
#define MATRIXCALCULATION_H_

#include <stdint.h>
#include <stdbool.h>
//#include "driverlib/fpu.h"
//#include "driverlib/rom.h"


typedef struct matrix
{
	uint32_t m;
	uint32_t n;
	uint32_t size;
	double elements[4];
}Matrix;

//inline void InitMatrixLib();

inline void InitMatrix(Matrix* A, uint32_t m, uint32_t n, double* values);

inline void SetSize(Matrix* A, uint32_t m, uint32_t n);

inline double GetElement(Matrix* A,uint32_t i, uint32_t j);

inline void SetElement(Matrix* A, uint32_t i, uint32_t j, double value);

inline bool AreDimensionsEqual(Matrix* A,Matrix* B );

int Add(Matrix* A, Matrix* B, Matrix* C );

int MultiplyByScalar(Matrix* A, double b, Matrix* B);

int Multiply(Matrix* A, Matrix* B, Matrix* C);

void Transpose(Matrix* A, Matrix* B);

void DeepMatrixCopy(Matrix* A, Matrix* B);

int Inverse(Matrix* A, Matrix* B);

#endif
