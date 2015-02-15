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

__inline void Matrix_Copy(float *pSrc, int numRows, int numCols, float *pDst)
{
	float *pIn = pSrc;
	float *pOut = pDst;
	unsigned int numSamples = numRows * numCols;
	unsigned int blkCnt = numSamples >> 2u;
	
	//cortex-m3's speed optimization
	while(blkCnt > 0u){
		pOut[0] = pIn[0];
		pOut[1] = pIn[1];
		pOut[2] = pIn[2];
		pOut[3] = pIn[3];
		
		pOut += 4u;
		pIn += 4u;
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

__inline int Maxtrix_Inverse(float *A, unsigned int n)
{
	int i, j, k, p;
	//////////////////////////////////////////////////////////////////////////
	//choleski lu decomposition 
	float *p_Lk0; // pointer to L[k][0]
	float *p_Lkp; // pointer to L[k][p]  
	float *p_Lkk; // pointer to diagonal element on row k.
	float *p_Li0; // pointer to L[i][0]
	float reciprocal;

	//////////////////////////////////////////////////////////////////////////
	//choleski lu inverse
	float *p_i, *p_j, *p_k;
	float *L = A;
	float *LU = A;
	float sum;

	//////////////////////////////////////////////////////////////////////////
	//choleski lu decomposition 
	for (k = 0, p_Lk0 = A; k < n; p_Lk0 += n, k++) {
		p_Lkk = p_Lk0 + k;
		for (p = 0, p_Lkp = p_Lk0; p < k; p_Lkp += 1,  p++)
			*p_Lkk -= *p_Lkp * *p_Lkp;
		if (*p_Lkk <= 0.0f){
			return -1;
		}
		reciprocal = FastSqrtI(*p_Lkk);
		p_Li0 = p_Lk0 + n;
		for (i = k + 1; i < n; p_Li0 += n, i++) {
			for (p = 0; p < k; p++)
				*(p_Li0 + k) -= *(p_Li0 + p) * *(p_Lk0 + p);
			*(p_Li0 + k) *= reciprocal;
			*(p_Lk0 + i) = *(p_Li0 + k);
		}  
	}
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
			for (k = j, p_k = p_j; k < i; k++, p_k += n)
				sum += *(p_i + k) * *(p_k + j);
			*(p_i + j) = - *(p_i + i) * sum;
		}
	}
	//choleski lu inverse
	for (i = 0, p_i = LU; i < n; i++, p_i += n) {
		for (j = 0, p_j = LU; j <= i; j++, p_j += n) {
			sum = 0.0;
			for (k = i, p_k = p_i; k < n; k++, p_k += n)
				sum += *(p_k + i) * *(p_k + j);
			*(p_i + j) = sum;
			*(p_j + i) = sum;
		}
	}

	return 0;
}
#endif
