#ifndef MATRIXCALCULATION_H_
#define MATRIXCALCULATION_H_

#include <stdbool.h>

typedef unsigned int uint;

typedef struct matrix
{
	uint m;
	uint n;
	uint size;
	double elements[4];
}Matrix;

inline void InitMatrix(Matrix* A, uint m, uint n, double* values);

inline void SetSize(Matrix* A, uint m, uint n);

inline double GetElement(Matrix* A,uint i, uint j);

inline void SetElement(Matrix* A, uint i, uint j, double value);

inline bool AreDimensionsEqual(Matrix* A,Matrix* B );

int Add(Matrix* A, Matrix* B, Matrix* C );

int MultiplyByScalar(Matrix* A, double b, Matrix* B);

int Multiply(Matrix* A, Matrix* B, Matrix* C);

void Transpose(Matrix* A);

void CopyMatrix(Matrix* A, Matrix* B);

#endif
