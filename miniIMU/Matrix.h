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

#ifndef _MAXTRIX_H_
#define _MAXTRIX_H_

#include "FastMath.h"

//////////////////////////////////////////////////////////////////////////
//

__inline void Matrix_Zero(float *A, int numRows, int numCols)
{
	float *pIn = A;
	unsigned int numSamples = numRows * numCols;
	unsigned int blkCnt = numSamples >> 2u;

	//cortex-m3's speed optimization
	while(blkCnt > 0u){
		(*pIn++) = 0.0f;
		(*pIn++) = 0.0f;
		(*pIn++) = 0.0f;
		(*pIn++) = 0.0f;
		blkCnt--;
	}
	blkCnt = numSamples & 0x03u;

	while(blkCnt > 0u){
		(*pIn++) = 0.0f;
		blkCnt--;
	}
}

__inline void Matrix_Copy(float *pSrc, int numRows, int numCols, float *pDst)
{
	float *pIn = pSrc;
	float *pOut = pDst;
	unsigned int numSamples = numRows * numCols;
	unsigned int blkCnt = numSamples >> 2u;

	//cortex-m3's speed optimization
	while(blkCnt > 0u){
		(*pOut++) = (*pIn++);
		(*pOut++) = (*pIn++);
		(*pOut++) = (*pIn++);
		(*pOut++) = (*pIn++);
		blkCnt--;
	}
	blkCnt = numSamples & 0x03u;

	while(blkCnt > 0u){
		(*pOut++) = (*pIn++);
		blkCnt--;
	}
}

__inline int Maxtrix_Add(float *pSrcA, unsigned short numRows, unsigned short numCols, float *pSrcB, float *pDst)
{
	float *pIn1 = pSrcA;
	float *pIn2 = pSrcB;
	float *pOut = pDst;

	unsigned int numSamples = numRows * numCols;
	unsigned int blkCnt = numSamples >> 2u;

	//cortex-m3's speed optimization
	while(blkCnt > 0u){
		// C(m,n) = A(m,n) + B(m,n)
		*(pOut++) = *(pIn1++) + (*pIn2++);
		*(pOut++) = *(pIn1++) + (*pIn2++);
		*(pOut++) = *(pIn1++) + (*pIn2++);
		*(pOut++) = *(pIn1++) + (*pIn2++);

		blkCnt--;
	}
	blkCnt = numSamples & 0x03u;

	while(blkCnt > 0u){
		// C(m,n) = A(m,n) + B(m,n)
		*pOut++ = (*pIn1++) + (*pIn2++);
		blkCnt--;
	}
	return 0;
}

__inline int Maxtrix_Sub(float *pSrcA, unsigned short numRows, unsigned short numCols, float *pSrcB, float *pDst)
{
	float *pIn1 = pSrcA;
	float *pIn2 = pSrcB;
	float *pOut = pDst;

	unsigned int numSamples = numRows * numCols;
	unsigned int blkCnt = numSamples >> 2u;

	while(blkCnt > 0u){
		// C(m,n) = A(m,n) - B(m,n)
		*(pOut++) = *(pIn1++) - (*pIn2++);
		*(pOut++) = *(pIn1++) - (*pIn2++);
		*(pOut++) = *(pIn1++) - (*pIn2++);
		*(pOut++) = *(pIn1++) - (*pIn2++);

		blkCnt--;
	}
	blkCnt = numSamples & 0x03u;

	while(blkCnt > 0u){
		// C(m,n) = A(m,n) - B(m,n)
		*pOut++ = (*pIn1++) - (*pIn2++);
		blkCnt--;
	}
	return 0;
}

__inline int Maxtrix_Mul(float *A, int nrows, int ncols, float *B, int mcols, float *C)
{
	float *pB;
	float *p_B;
	int i,j,k;

	for (i = 0; i < nrows; A += ncols, i++){
		for (p_B = B, j = 0; j < mcols; C++, p_B++, j++) {
			pB = p_B;
			*C = 0.0; 
			for (k = 0; k < ncols; pB += mcols, k++) 
				*C += *(A+k) * *pB;
		}
	}
	return 0;
}

__inline void Maxtrix_Mul_With_Transpose(float *A, int nrows, int ncols, float *B, int mrows, float *C) 
{
	int i,j,k;
	float *pA;
	float *pB;

	for (i = 0; i < nrows; A += ncols, i++){
		for (pB = B, j = 0; j < mrows; C++, j++){
			for (pA = A, *C = 0.0, k  = 0; k < ncols; k++){
				*C += *pA++ * *pB++;
			}
		}
	}
}

__inline void Maxtrix_Transpose(float *pSrc, unsigned short nRows, unsigned short nCols, float *pDst)
{
	float *pIn = pSrc;
	float *pOut = pDst;
	float *px;

	unsigned short blkCnt, i = 0u, row = nRows;

	do{
		blkCnt = nCols >> 2;
		px = pOut + i;

		while(blkCnt > 0u){
			*px = *pIn++;
			px += nRows;

			*px = *pIn++;
			px += nRows;

			*px = *pIn++;
			px += nRows;

			*px = *pIn++;
			px += nRows;

			blkCnt--;
		}
		blkCnt = nCols & 0x03u;
		while(blkCnt > 0u){
			*px = *pIn++;
			px += nRows;
			blkCnt--;
		}

		i++;
		row--;
	} while(row > 0u);
}

__inline int Maxtrix_Chol_Decomposition(float *A, unsigned int n)
{
	int i, k, p;
	//////////////////////////////////////////////////////////////////////////
	//choleski lu decomposition 
	float *p_Lk0; // pointer to L[k][0]
	float *p_Lkp; // pointer to L[k][p]  
	float *p_Lkk; // pointer to diagonal element on row k.
	float *p_Li0; // pointer to L[i][0]
	float reciprocal;

	//////////////////////////////////////////////////////////////////////////
	//choleski lu decomposition 
	for (k = 0, p_Lk0 = A; k < n; p_Lk0 += n, k++) {
		p_Lkk = p_Lk0 + k;
		for (p = 0, p_Lkp = p_Lk0; p < k; p_Lkp += 1,  p++){
			*p_Lkk -= *p_Lkp * *p_Lkp;
		}
		if (*p_Lkk <= 0.0f){
			return -1;
		}
		reciprocal = FastSqrtI(*p_Lkk);
		p_Li0 = p_Lk0 + n;
		for (i = k + 1; i < n; p_Li0 += n, i++) {
			for (p = 0; p < k; p++){
				*(p_Li0 + k) -= *(p_Li0 + p) * *(p_Lk0 + p);
			}
			*(p_Li0 + k) *= reciprocal;
			*(p_Lk0 + i) = *(p_Li0 + k);
		}  
	}
	return 0;

}

__inline int Maxtrix_Inverse(float *A, unsigned int n)
{
	int i, j, k;
	//////////////////////////////////////////////////////////////////////////
	//choleski lu inverse
	float *p_i, *p_j, *p_k;
	float *L = A;
	float *LU = A;
	float sum;
	//////////////////////////////////////////////////////////////////////////
	//lower triangular inverse
	for (k = 0, p_k = L; k < n; p_k += (n + 1), k++) {
		if (*p_k == 0.0f){
			return -1;
		}
		else{
			*p_k = 1.0f / *p_k;
		}
	}
	for (i = 1, p_i = L + n; i < n; i++, p_i += n) {
		for (j = 0, p_j = L; j < i; p_j += n, j++) {
			sum = 0.0f;
			for (k = j, p_k = p_j; k < i; k++, p_k += n){
				sum += *(p_i + k) * *(p_k + j);
			}
			*(p_i + j) = - *(p_i + i) * sum;
		}
	}
	//choleski lu inverse
	for (i = 0, p_i = LU; i < n; i++, p_i += n) {
		for (j = 0, p_j = LU; j <= i; j++, p_j += n) {
			sum = 0.0;
			for (k = i, p_k = p_i; k < n; k++, p_k += n){
				sum += *(p_k + i) * *(p_k + j);
			}
			*(p_i + j) = sum;
			*(p_j + i) = sum;
		}
	}

	return 0;
}

__inline int Matrix_LU_Decomposition(float* pSourceDestLU, int* pPerm, unsigned int n)
{
  int	i, j, k, iC;
  int	iMax;
  int	retNumPerm = 0;
  short int sTmp;
	// Pivot Variables
  float fP1, fP2;
	float fLK, fKK;

  iC = n;

  for (i = 0; i < iC; ++i){
		pPerm[i] = i;
	}

  // Partial Pivoting
  for (k = 0; k < iC; ++k){
    for (i = k, iMax = k, fP1 = 0.0f; i < iC; ++i){
      // Local ABS
			fLK = pSourceDestLU[pPerm[i] * iC + k];
      if (fLK > 0){
        fP2 = fLK;
			}
      else{
        fP2 = - fLK;
			}
      if (fP2 > fP1){
        fP1 = fP2;
        iMax = i;
      }
    }
    // Row exchange, update permutation vector
    if (k != iMax){
      retNumPerm++;
      sTmp = pPerm[k];
      pPerm[k] = pPerm[iMax];
      pPerm[iMax] = sTmp;
    }

    // Suspected Singular Matrix
		fKK = *(pSourceDestLU + pPerm[k] * iC + k);
    if (fKK == 0.0f){
      return -1;
		}

    for (i = k + 1; i < iC; ++i){
      // Calculate Mat [i][j]
			fLK = *(pSourceDestLU + pPerm[i] * iC + k);
      *(pSourceDestLU + pPerm[i] * iC + k) = fLK / fKK;

      // Elimination
      for (j = k + 1; j < iC; ++j){
        *(pSourceDestLU + pPerm[i] * iC + j) -=
          *(pSourceDestLU + pPerm[i] * iC + k) * *(pSourceDestLU + pPerm[k] * iC + j);
      }
    }
  }
  return retNumPerm;
}

__inline void Maxtrix_BackSubs(float* pSourceLU, float* pSourceDestColumn,
	int* pPerm,  int iCols, int iResultCol, float* pDest)
{
  int i, j, k, iC;
	int permK, permI;
  float fSum, fTmp;

  iC = iCols;

  for (k = 0; k < iC; ++k){
		permK = pPerm[k] * iC;
    for (i = k + 1; i < iC; ++i){
			permI = pPerm[i] * iC;
      pSourceDestColumn[permI] -= pSourceLU[permI + k] * pSourceDestColumn[permK];
		}
	}
	i = (iC - 1);
	j = i * iC;
	k = pPerm[i] * iC;
  pDest[j + iResultCol] = pSourceDestColumn[k] /pSourceLU[k + i];

  for (k = iC - 2; k >= 0; k--){
    fSum = 0.0f;
		permK = pPerm[k] * iC;
    for (j = k + 1; j < iC; ++j){
      fSum += pSourceLU[permK + j] * pDest[j * iC + iResultCol];
		}
    fTmp = pSourceDestColumn[permK] - fSum;
    pDest[k * iC + iResultCol] = fTmp / pSourceLU[permK + k];
  }
}

__inline int Maxtrix_Inv(float *A, float *B, int* P, unsigned int n, float* pDest)
{
	int i, j, k;
	int jC;
	// LU Decomposition and check for Singular Matrix
  if (Matrix_LU_Decomposition(A, P, n) == -1){
		return -1;
  }
	for (i = 0; i < n; ++i){
    for (j = 0; j < n; j++){
			jC = j * n;
      for (k = 0; j < n; j++){
          B[jC + k] = 0.0f;
			}
		}
    B[i * n] = 1.0f;
    Maxtrix_BackSubs(A, B, P, n, i, pDest);
  }
	return 0;
}

//need optimization!!
__inline int Maxtrix_QR_DecompositionT(float *A, unsigned int m, unsigned int n,
									  float *Q, unsigned int numRowsQ, unsigned int numColsQ,
									  float *R, unsigned int numRowsR, unsigned int numColsR)
{
	int minor;
	int row, col;
	int min;
	float xNormSqr = 0.0f;
	float alpha = 0.0f;
	float a;

	// clear R
	Matrix_Zero(R, numRowsR, numColsR);

	min = ((m < n) ? m : n);

	for (minor = 0; minor < min; minor++) {
		xNormSqr = 0.0f;
		for (row = minor; row < m; row++){
			xNormSqr += A[minor * m + row] * A[minor * m + row];
		}

		a = FastSqrt(xNormSqr);
		if (A[minor * m + minor] > 0.0f){
			a = -a;
		}

		if (a != 0.0f) {
			R[minor * numColsR + minor] = a;
			A[minor * m + minor] -= a;
			for (col = minor + 1; col < n; col++) {
				alpha = 0.0f;
				for (row = minor; row < m; row++){
					alpha -= A[col * m + row] * A[minor * m + row];
				}

				alpha /= a * A[minor * m + minor];
				for (row = minor; row < m; row++){
					A[col * m + row] -= alpha * A[minor * m + row];
				}
			}
		}
		else{
			return 0;
		}
	}

	// Form the matrix R of the QR-decomposition.
	// R is supposed to be m x n, but only calculate n x n
	// copy the upper triangle of A
	for (row = min-1; row >= 0; row--){
		for (col = row+1; col < n; col++){
			R[row * numColsR + col] = A[col * m + row];
		}
	}

	// Form the matrix Q of the QR-decomposition.
	// Q is supposed to be m x m
	if (Q) {
		Matrix_Zero(Q, numRowsQ, numColsQ);
		for (minor = m - 1; minor >= min; minor--){
			Q[minor * m + minor] = 1.0f;
		}

		for (minor = min-1; minor >= 0; minor--) {
			Q[minor * m + minor] = 1.0f;

			if (A[minor * m + minor] != 0.0f) {
				for (col = minor; col < m; col++) {
					alpha = 0.0f;
					for (row = minor; row < m; row++){
						alpha -= Q[row * m + col] * A[minor * m + row];
					}

					alpha /= R[minor * numColsR + minor] * A[minor * m + minor];

					for (row = minor; row < m; row++){
						Q[row * m + col] -= alpha * A[minor * m + row];
					}
				}
			}
		}
	}

	return 1;
}

__inline void Matrix_Div(float *X, float *A, unsigned int numRowsA, unsigned int numColsA,
						 float *B, unsigned int numRowsB, unsigned int numColsB,
						 float *Q, float *R, float *AQ) {
	int i, j, k;
	int m, n;

	unsigned int numRowsQ = numRowsB;
	unsigned int numColsQ = numColsB;
	unsigned int numRowsR = numRowsB;
	unsigned int numColsR = numColsB;
	/*
	unsigned int numRowsAQ = numRowsA;
	unsigned int numColsAQ = numRowsB;
	*/

	m = numRowsA;
	n = numColsB;

	Maxtrix_QR_DecompositionT(B, numRowsB, numColsB, Q, numRowsQ, numColsQ, R, numRowsR, numColsR);
	Maxtrix_Mul(A, numRowsA, numColsA, Q, numColsQ, AQ);

	// solve for X by backsubstitution
	for (i = 0; i < m; i++) {
		for (j = n-1; j >= 0; j--) {
			for (k = j+1; k < n; k++){
				AQ[i*n + j] -= R[j*n + k] * X[i*n + k];
			}
			X[i*n + j] = AQ[i*n + j] / R[j*n + j];
		}
	}
}

#endif
