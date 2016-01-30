/*
 * MatrixCaluculation.c
 *
 *  Created on: 21 sty 2016
 *      Author: Rafal
 */


#include "MatrixCalculation.h"

void InitMatrixLib()
{
	// Enable FPU for fast calculations
	ROM_FPULazyStackingEnable();
	ROM_FPUEnable();
}

void SetSize(Matrix* A, uint32_t m, uint32_t n)
{
	A->m = m;
	A->n = n;
	A->size = m*n;
}

void InitMatrix(Matrix* A, uint32_t m, uint32_t n, double* values)
{
	SetSize(A, m, n);

	for(int i=0; i< A->size; i++)
	{
		A->elements[i]=values[i];
	}
}


double GetElement(Matrix* A,uint32_t i, uint32_t j)
{
	// get element i,j element from flat table
	return A->elements[(i*A->n)+j];
}

void SetElement(Matrix* A, uint32_t i, uint32_t j, double value)
{
	// set element i,j element from flat table
	A->elements[(i*A->n)+j]=value;
}

bool AreDimensionsEqual(Matrix* A,Matrix* B )
{
	if((A->m == B->m)&&(A->n == B->n))
		return true;
	else
		return false;
}

int Add(Matrix* A, Matrix* B, Matrix* C )
{
	if(AreDimensionsEqual(A,B))
	{
		for(int i=0; i< A->m*A->n; i++)
		{
			C->elements[i]=A->elements[i]+B->elements[i];
		}
        SetSize(C, A->m, A->n);
		return 0;
	}
	return -1;
}

int MultiplyByScalar(Matrix* A, double b, Matrix* B)
{
	if(A->size == B->size)
	{
	    for(int i=0; i<A->size; i++)
	    {
	        B->elements[i] = A->elements[i]*b;
	    }
	    return 0;
	}
	return -1;
}

int Multiply(Matrix* A, Matrix* B, Matrix* C)
{
	if(A->n==B->m)
	{
		int m = A->m;
		int n = A->n; // Amxn*Bnxp=Cmxp
		int p = B->n;

		Matrix result;
		//SetSize(&tmp, m, p);
        SetSize(C, m, p);

		for(int i=0; i<m; i++)
		{
			for(int j=0; j<p; j++)
			{
				double tmp = 0.0;
				for(int r=0; r<n; r++)
				{
					tmp += GetElement(A, i, r)*GetElement(B, r, j);
				}
				SetElement(&result, i, j, tmp);
			}
		}
		DeepMatrixCopy(&result, C);
		return 0;
	}
	return -1;
}

/**
 * @brief Copy matrix A to matrix B
 */
void DeepMatrixCopy(Matrix* A, Matrix* B)
{
    SetSize(B, A->m, A->n);

    for(int i=0; i<A->size; i++)
    {
        B->elements[i]=A->elements[i];
    }
}

void Transpose(Matrix* A, Matrix* B)
{
	SetSize(B,A->n, A->m);

	for(int i=0; i<A->m; i++)
	{
		for(int j=0; j<A->n; j++)
		{
            SetElement(B, j ,i , GetElement(A, i, j) );
		}
	}
}

int Inverse(Matrix* A, Matrix* B)
{
	if(A->m==A->n)
	{
		double det = (GetElement(A,1,1)*GetElement(A,2,2))-(GetElement(A,1,2)*GetElement(A,2,1));
		SetElement(B, 1, 1, GetElement(A, 2, 2));
		SetElement(B, 1, 2, -GetElement(A, 1, 2));
		SetElement(B, 2, 1, -GetElement(A, 2, 1));
		SetElement(B, 2, 2, GetElement(A, 1, 1));
		MultiplyByScalar(B, det, B);
		return 0;
	}
	else
		return -1;
}

