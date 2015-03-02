/*
The MIT License (MIT)

Copyright (c) 2015-? suhetao

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "Matrix.h"
#include "FastMath.h"

arm_status mat_identity(float32_t *pData, uint16_t numRows, uint16_t numCols, float32_t value)
{
	uint16_t pos = 0;
	float32_t *A = pData;
	if (numRows != numCols){
		return ARM_MATH_LENGTH_ERROR;
	}
	do{
		A[pos * numRows + pos] = value;
		pos++;
	} while (pos < numRows);
	return ARM_MATH_SUCCESS;
}

void arm_mat_zero_f32(arm_matrix_instance_f32* s)
{
	uint16_t pos = 0;
	uint16_t blockSize = s->numRows * s->numCols;
	float32_t *A = s->pData;

	do{
		A[pos] = 0.0f;
		pos++;
	} while (pos < blockSize);
}

arm_status arm_mat_identity_f32(arm_matrix_instance_f32* s, float32_t value)
{
	uint16_t pos = 0;
	float32_t *A = s->pData;
	int n = s->numRows;
	if (n != s->numCols){
		return ARM_MATH_LENGTH_ERROR;
	}
	do{
		A[pos * n + pos] = value;
		pos++;
	} while (pos < n);
	return ARM_MATH_SUCCESS;
}

arm_status arm_mat_remainlower_f32(arm_matrix_instance_f32* s)
{
	float32_t *A = s->pData;
	int nrows = s->numRows;
	int ncols = s->numCols;
	int k, p;
	float *p_Uk0; // pointer to U[k][0]
	float *p_Ukp; // pointer to U[k][p]  

	if (nrows != ncols){
		return ARM_MATH_SIZE_MISMATCH;
	}
	for (k = 0, p_Uk0 = A; k < nrows; p_Uk0 += ncols, k++){
		for (p = k + 1; p < ncols; p++){
			p_Ukp = p_Uk0 + p;
			*p_Ukp = 0.0f;
		}
	}
	return ARM_MATH_SUCCESS;
}

arm_status arm_mat_fill_f32(arm_matrix_instance_f32* s, float32_t *pData, uint32_t blockSize)
{
	uint16_t pos = 0;
	float32_t *A = s->pData;
	if (s->numRows * s->numCols < blockSize){
		return ARM_MATH_SIZE_MISMATCH;
	}

	do{
		A[pos] = pData[pos];
		pos++;
	} while (pos < blockSize);
	return ARM_MATH_SUCCESS;
}

//Returns the lower triangular matrix L in the
//bottom of A so that L*L' = A.

arm_status arm_mat_chol_f32(arm_matrix_instance_f32* s)
{
	int i, k, p;
	float32_t *p_Lk0; // pointer to L[k][0]
	float32_t *p_Lkp; // pointer to L[k][p]  
	float32_t *p_Lkk; // pointer to diagonal element on row k.
	float32_t *p_Li0; // pointer to L[i][0]
	float32_t reciprocal;
	float32_t *A = s->pData;
	int n = s->numRows;

	if (n != s->numCols){
		return ARM_MATH_SIZE_MISMATCH;
	}

	for (k = 0, p_Lk0 = A; k < n; p_Lk0 += n, k++) {
		p_Lkk = p_Lk0 + k;
		for (p = 0, p_Lkp = p_Lk0; p < k; p_Lkp += 1,  p++){
			*p_Lkk -= *p_Lkp * *p_Lkp;
		}
		if (*p_Lkk <= 0.0f){
			return ARM_MATH_SINGULAR;
		}
		arm_sqrt_f32(*p_Lkk, p_Lkk);
		reciprocal = 1.0f / *p_Lkk;

		p_Li0 = p_Lk0 + n;
		for (i = k + 1; i < n; p_Li0 += n, i++) {
			for (p = 0; p < k; p++){
				*(p_Li0 + k) -= *(p_Li0 + p) * *(p_Lk0 + p);
			}
			*(p_Li0 + k) *= reciprocal;
			*(p_Lk0 + i) = *(p_Li0 + k);
		}  
	}
	return ARM_MATH_SUCCESS;
}

void arm_mat_getsubmatrix_f32(arm_matrix_instance_f32* s, arm_matrix_instance_f32 *a, int row, int col)
{
	int k;
	int mrows = s->numRows;
	int mcols = s->numCols;
	int ncols = a->numCols;
	
	float32_t *S = s->pData;
	float32_t *A = a->pData;

	for (A += row * ncols + col; mrows > 0; A += ncols, S+= mcols, mrows--){
		for(k = 0; k < mcols; k++){
			*(S + k) = *(A + k);
		}
	}
}

void arm_mat_setsubmatrix_f32(arm_matrix_instance_f32* a, arm_matrix_instance_f32 *s, int row, int col)
{
	int mrows = s->numRows;
	int mcols = s->numCols;
	int ncols = a->numCols;
	int i,j;
	
	float32_t *S = s->pData;
	float32_t *A = a->pData;

	for(A += row * ncols + col, i = 0; i < mrows; A += ncols, i++){
      for(j = 0; j < mcols; j++){
				*(A + j) = *S++;
			}
	}
}

void arm_mat_getcolumn_f32(arm_matrix_instance_f32* s, float32_t *x, uint32_t col)
{
	int nrows = s->numRows, ncols = s->numCols;
	float32_t *S = s->pData;
	int i = 0;
	for (S += col; i < nrows; S += ncols, i++) x[i] = *S;
}

void arm_mat_setcolumn_f32(arm_matrix_instance_f32* s, float32_t *x, uint32_t col)
{
	int nrows = s->numRows, ncols = s->numCols;
	float32_t *S = s->pData;
	int i = 0;
	for (S += col; i < nrows; S += ncols, i++) *S = x[i];
}
