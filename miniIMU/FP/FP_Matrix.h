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
		out1 = FP_ADD(inA1, inB1);

		inB2 = pIn2[1];
		inA1 = pIn1[2];
		out2 = FP_ADD(inA2, inB2);

		inB1 = pIn2[2];
		pOut[0] = out1;
		pOut[1] = out2;

		inA2 = pIn1[3];
		inB2 = pIn2[3];
		out1 = FP_ADD(inA1, inB1);
		out2 = FP_ADD(inA2, inB2);

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
		*pOut++ = FP_ADD((*pIn1++), (*pIn2++));

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
		out1 = FP_SUB(inA1, inB1);

		inB2 = pIn2[1];
		inA1 = pIn1[2];
		out2 = FP_SUB(inA2, inB2);

		inB1 = pIn2[2];
		pOut[0] = out1;
		pOut[1] = out2;

		inA2 = pIn1[3];
		inB2 = pIn2[3];
		out1 = FP_SUB(inA1, inB1);
		out2 = FP_SUB(inA2, inB2);

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
		*pOut++ = FP_SUB((*pIn1++), (*pIn2++));

		blkCnt--;
	}
	return 0;
}

__inline int FP_Maxtrix_Mul(Q16 *pSrcA, unsigned short numRowsA, unsigned short numColsA, Q16 *pSrcB, unsigned short numColsB, Q16 *pDst)
{
	Q16 *pIn1 = pSrcA;
	Q16 *pIn2 = pSrcB;
	Q16 *pInA = pSrcA;
	Q16 *pOut = pDst;
	Q16 *px;

	Q16 *pInB = pSrcB;
	uint16_t col, i = 0u, row = numRowsA, colCnt;
	int __al, __ah;

	do{
		px = pOut + i;
		col = numColsB;
		pIn2 = pSrcB;
		do{
		
			__al = __ah = 0;
			pIn1 = pInA;
			colCnt = numColsA;
			while(colCnt > 0u){
				//c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n)
				//cortex-m3's instruction assembly optimization
				//sum += *pIn1++ * (*pIn2);
				__asm{
					smlal __al, __ah, *pIn1++, *pIn2;
				}
				pIn2 += numColsB;
				colCnt--;
			}
			//cortex-m3's instruction assembly optimization
			//*px++ = sum;
			__asm{
				lsls __ah, __ah, #16;
				orr *px++, __ah, __al, lsr #16;
			}
			col--;
			pIn2 = pInB + (numColsB - col);

		} while(col > 0u);
		i = i + numColsB;
		pInA = pInA + numColsA;
		row--;

	} while(row > 0u);
	return 0;
}

__inline int FP_Maxtrix_Trans(Q16 *pSrc, unsigned short nRows, unsigned short nCols, Q16 *pDst)
{
	Q16 *pIn = pSrc;
	Q16 *pOut = pDst;
	Q16 *px;

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
	return 0;
}

__inline int FP_Maxtrix_Inverse(Q16 *pSrc, unsigned int numRows, unsigned int numCols, Q16 *pDst)
{
	Q16 *pIn = pSrc;
	Q16 *pOut = pDst;
	Q16 *pInT1, *pInT2;
	Q16 *pInT3, *pInT4;
	Q16 *pPivotRowIn, *pPRT_in, *pPivotRowDst, *pPRT_pDst;

	Q16 maxC;
	int status = 0;
	Q16 Xchg, in = 0, in1;
	unsigned int i, rowCnt, flag = 0u, j, loopCnt, k, l;

	pInT2 = pOut;
	rowCnt = numRows;

	while(rowCnt > 0u){
		j = numRows - rowCnt;
		while(j > 0u){
			*pInT2++ = 0;
			j--;
		}
		*pInT2++ = Q16_One;
		j = rowCnt - 1u;
		while(j > 0u){
			*pInT2++ = 0;
			j--;
		}
		rowCnt--;
	}
	loopCnt = numCols;
	l = 0u;

	while(loopCnt > 0u){
		pInT1 = pIn + (l * numCols);
		pInT3 = pOut + (l * numCols);
		in = *pInT1;
		k = 1u;
		maxC = 0;
		for (i = 0; i < numRows; i++){
			maxC = *pInT1 > 0 ? (*pInT1 > maxC ? *pInT1 : maxC) : (-*pInT1 > maxC ? -*pInT1 : maxC);
			pInT1 += numCols;
		}
		if(maxC == 0){
			status = -1;
			break;
		}
		pInT1 -= numRows * numCols;
		if( (in > 0 ? in : -in) != maxC){
			i = numRows - (l + 1u);
			while(i > 0u){
				pInT2 = pInT1 + (numCols * l);
				pInT4 = pInT3 + (numCols * k);

				if((*pInT2 > 0 ? *pInT2: -*pInT2) == maxC){
					j = numCols - l;

					while(j > 0u){
						Xchg = *pInT2;
						*pInT2++ = *pInT1;
						*pInT1++ = Xchg;
						j--;
					}
					j = numCols;

					while(j > 0u){
						Xchg = *pInT4;
						*pInT4++ = *pInT3;
						*pInT3++ = Xchg;
						j--;
					}
					flag = 1u;
					break;
				}
				k++;
				i--;
			}
		}
		if((flag != 1u) && (in == 0)){
			status = -1;
			break;
		}

		pPivotRowIn = pIn + (l * numCols);
		pPivotRowDst = pOut + (l * numCols);

		pInT1 = pPivotRowIn;
		pInT2 = pPivotRowDst;
		in = *pPivotRowIn;
		j = (numCols - l);

		while(j > 0u){
			in1 = *pInT1;
			//*pInT1++ = in1 / in;
			*pInT1++ = FP_SDIV(in1, in);
			j--;
		}
		j = numCols;

		while(j > 0u){
			in1 = *pInT2;
			//*pInT2++ = in1 / in;
			*pInT2++ = FP_SDIV(in1, in);
			j--;
		}
		pInT1 = pIn;
		pInT2 = pOut;

		i = 0u;
		k = numRows;

		while(k > 0u){
			if(i == l){
				pInT1 += numCols - l;

				pInT2 += numCols;
			}
			else{
				in = *pInT1;

				pPRT_in = pPivotRowIn;
				pPRT_pDst = pPivotRowDst;

				j = (numCols - l);

				while(j > 0u){
					in1 = *pInT1;
					//*pInT1++ = in1 - (in * *pPRT_in++);
					*pInT1++ = in1 - FP_SMUL(in, *pPRT_in++);
					j--;
				}
				j = numCols;

				while(j > 0u){
					in1 = *pInT2;
					//*pInT2++ = in1 - (in * *pPRT_pDst++);
					*pInT2++ = in1 - FP_SMUL(in, *pPRT_pDst++);
					j--;
				}

			}
			pInT1 = pInT1 + l;
			k--;
			i++;
		}
		pIn++;
		loopCnt--;
		l++;
	}
	status = 0;

	if((flag != 1u) && (in == 0.0f)){
		status = -1;
	}

	return status;
}


#endif
