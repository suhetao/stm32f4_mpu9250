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

#ifndef _FP_MAXTRIX_H_
#define _FP_MAXTRIX_H_

#include "FP_Math.h"

//////////////////////////////////////////////////////////////////////////
//

__inline void FP_Matrix_Copy(Q16 *pSrc, int numRows, int numCols, Q16 *pDst)
{
	Q16 *pIn = pSrc;
	Q16 *pOut = pDst;
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

__inline int FP_Maxtrix_Add(Q16 *pSrcA, unsigned short numRows, unsigned short numCols, Q16 *pSrcB, Q16 *pDst)
{
	Q16 *pIn1 = pSrcA;
	Q16 *pIn2 = pSrcB;
	Q16 *pOut = pDst;

	Q16 inA1, inA2, inB1, inB2, out1, out2;

	unsigned int numSamples;
	unsigned int blkCnt;

	numSamples = (unsigned int) numRows * numCols;

	blkCnt = numSamples >> 2u;
	while(blkCnt > 0u){
		//C(m,n) = A(m,n) + B(m,n)
		inA1 = pIn1[0];
		inB1 = pIn2[0];
		inA2 = pIn1[1];
		out1 = FP_ADDS(inA1, inB1);

		inB2 = pIn2[1];
		inA1 = pIn1[2];
		out2 = FP_ADDS(inA2, inB2);

		inB1 = pIn2[2];
		pOut[0] = out1;
		pOut[1] = out2;

		inA2 = pIn1[3];
		inB2 = pIn2[3];
		out1 = FP_ADDS(inA1, inB1);
		out2 = FP_ADDS(inA2, inB2);

		pOut[2] = out1;
		pOut[3] = out2;

		pIn1 += 4u;
		pIn2 += 4u;
		pOut += 4u;

		blkCnt--;
	}
	blkCnt = numSamples & 0x03u;

	while(blkCnt > 0u){
		//C(m,n) = A(m,n) + B(m,n)
		*pOut++ = FP_ADDS((*pIn1++), (*pIn2++));

		blkCnt--;
	}
	return 0;
}

__inline int FP_Maxtrix_Sub(Q16 *pSrcA, unsigned short numRows, unsigned short numCols, Q16 *pSrcB, Q16 *pDst)
{
	Q16 *pIn1 = pSrcA;
	Q16 *pIn2 = pSrcB;
	Q16 *pOut = pDst;

	Q16 inA1, inA2, inB1, inB2, out1, out2;

	unsigned int numSamples;
	unsigned int blkCnt;

	numSamples = (unsigned int) numRows * numCols;

	blkCnt = numSamples >> 2u;
	while(blkCnt > 0u){
		//C(m,n) = A(m,n) - B(m,n)
		inA1 = pIn1[0];
		inB1 = pIn2[0];
		inA2 = pIn1[1];
		out1 = FP_SUBS(inA1, inB1);

		inB2 = pIn2[1];
		inA1 = pIn1[2];
		out2 = FP_SUBS(inA2, inB2);

		inB1 = pIn2[2];
		pOut[0] = out1;
		pOut[1] = out2;

		inA2 = pIn1[3];
		inB2 = pIn2[3];
		out1 = FP_SUBS(inA1, inB1);
		out2 = FP_SUBS(inA2, inB2);

		pOut[2] = out1;
		pOut[3] = out2;

		pIn1 += 4u;
		pIn2 += 4u;
		pOut += 4u;

		blkCnt--;
	}
	blkCnt = numSamples & 0x03u;

	while(blkCnt > 0u){
		//C(m,n) = A(m,n) - B(m,n)
		*pOut++ = FP_SUBS((*pIn1++), (*pIn2++));

		blkCnt--;
	}
	return 0;
}

__inline int FP_Maxtrix_Mul(Q16 *A, int nrows, int ncols, Q16 *B, int mcols, Q16 *C)
{
	Q16 *pB;
	Q16 *p_B;
	int i,j,k;
	int __al, __ah;

	for (i = 0; i < nrows; A += ncols, i++){
		for (p_B = B, j = 0; j < mcols; C++, p_B++, j++) {
			pB = p_B;
			//*C = 0;
			__al = 0; __ah = 0;
			for (k = 0; k < ncols; pB += mcols, k++){
				//*C += *(A+k) * *pB;
				__asm{
					smlal __al, __ah, *(A+k), *pB;
				}
			}
			__asm{
				lsls __ah, __ah, #16;
				orr *C, __ah, __al, lsr #16;
			}
		}
	}
	return 0;
}

__inline void FP_Maxtrix_Mul_With_Transpose(Q16 *A, int nrows, int ncols, Q16 *B, int mrows, Q16 *C) 
{
	int i,j,k;
	Q16 *pA;
	Q16 *pB;
	int __al, __ah;

	for (i = 0; i < nrows; A += ncols, i++){
		for (pB = B, j = 0; j < mrows; C++, j++){
			__al = 0; __ah = 0;
			for (pA = A, /* ,*C = 0 */k  = 0; k < ncols; k++){
				//*C += *pA++ * *pB++;
				__asm{
					smlal __al, __ah, *pA++, *pB++;
				}
			}
			__asm{
				lsls __ah, __ah, #16;
				orr *C, __ah, __al, lsr #16;
			}
		}
	}
}

__inline int FP_Matrix_LU_Decomposition(Q16* pSourceDestLU, int* pPerm, int n)
{
  int	i, j, k, iC;
  int	iMax;
  int	retNumPerm = 0;
  short int sTmp;
	// Pivot Variables
  Q16 qP1, qP2;
	Q16 qLK, qKK;

  iC = n;

  for (i = 0; i < iC; ++i){
		pPerm[i] = i;
	}

  // Partial Pivoting
  for (k = 0; k < iC; ++k){
    for (i = k, iMax = k, qP1 = 0; i < iC; ++i){
      // Local ABS
			qLK = pSourceDestLU[pPerm[i] * iC + k];
      if (qLK > 0){
        qP2 = qLK;
			}
      else{
        qP2 = -qLK;
			}
      if (qP2 > qP1){
        qP1 = qP2;
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
		qKK = *(pSourceDestLU + pPerm[k] * iC + k);
    if (qKK == 0){
      return -1;
		}

    for (i = k + 1; i < iC; ++i){
      // Calculate Mat [i][j]
			qLK = *(pSourceDestLU + pPerm[i] * iC + k);
      *(pSourceDestLU + pPerm[i] * iC + k) = FP_SDIV(qLK, qKK);

      // Elimination
      for (j = k + 1; j < iC; ++j){
        *(pSourceDestLU + pPerm[i] * iC + j) -=
          FP_SMUL(*(pSourceDestLU + pPerm[i] * iC + k), *(pSourceDestLU + pPerm[k] * iC + j));
      }
    }
  }
  return retNumPerm;
}

__inline void FP_Maxtrix_BackSubs(Q16* pSourceLU, Q16* pSourceDestColumn,
	int* pPerm,  int iCols, int iResultCol, Q16* pDest)
{
  int i, j, k, iC;
	int permK, permI;
  Q16 qSum, qTmp;
	int __al, __ah;

  iC = iCols;

  for (k = 0; k < iC; ++k){
		permK = pPerm[k] * iC;
    for (i = k + 1; i < iC; ++i){
			permI = pPerm[i] * iC;
      pSourceDestColumn[permI] -= FP_SMUL(pSourceLU[permI + k], pSourceDestColumn[permK]);
		}
	}
	i = (iC - 1);
	j = i * iC;
	k = pPerm[i] * iC;
  pDest[j + iResultCol] = FP_SDIV(pSourceDestColumn[k], pSourceLU[k + i]);

  for (k = iC - 2; k >= 0; k--){
		__al = 0;__ah = 0;
		permK = pPerm[k] * iC;
    for (j = k + 1; j < iC; ++j){
      //qSum += pSourceLU[permK + j] * pDest[j * iC + iResultCol];
			__asm{
					smlal __al, __ah, pSourceLU[permK + j], pDest[j * iC + iResultCol];
				}
		}
		__asm{
			lsls __ah, __ah, #16;
			orr qSum, __ah, __al, lsr #16;
		}
    qTmp = FP_SUBS(pSourceDestColumn[permK], qSum);
    pDest[k * iC + iResultCol] = FP_SDIV(qTmp, pSourceLU[permK + k]);
  }
}

__inline int FP_Maxtrix_Inv(Q16 *A, Q16 *B, int* P, int n, Q16* pDest)
{
	int i, j, k;
	int jC;
	// LU Decomposition and check for Singular Matrix
  if(FP_Matrix_LU_Decomposition(A, P, n) == -1){
		return -1;
  }
	for (i = 0; i < n; ++i){
    for (j = 0; j < n; j++){
			jC = j * n;
      for (k = 0; j < n; j++){
          B[jC + k] = 0;
			}
		}
    B[i * n] = Q16_One;
    FP_Maxtrix_BackSubs(A, B, P, n, i, pDest);
  }
	return 0;
}

#endif
