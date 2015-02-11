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

//////////////////////////////////////////////////////////////////////////
//
__inline int Maxtrix_Add(float *pSrcA, unsigned short numRows, unsigned short numCols, float *pSrcB, float *pDst)
{
	float *pIn1 = pSrcA;
	float *pIn2 = pSrcB;
	float *pOut = pDst;

	float inA1, inA2, inB1, inB2, out1, out2;

	unsigned int numSamples;
	unsigned int blkCnt;

	numSamples = (unsigned int) numRows * numCols;

	blkCnt = numSamples >> 2u;
	while(blkCnt > 0u){
		/* C(m,n) = A(m,n) + B(m,n) */
		inA1 = pIn1[0];
		inB1 = pIn2[0];
		inA2 = pIn1[1];
		out1 = inA1 + inB1;

		inB2 = pIn2[1];
		inA1 = pIn1[2];
		out2 = inA2 + inB2;

		inB1 = pIn2[2];
		pOut[0] = out1;
		pOut[1] = out2;

		inA2 = pIn1[3];
		inB2 = pIn2[3];
		out1 = inA1 + inB1;
		out2 = inA2 + inB2;

		pOut[2] = out1;
		pOut[3] = out2;

		pIn1 += 4u;
		pIn2 += 4u;
		pOut += 4u;

		blkCnt--;
	}
	blkCnt = numSamples & 0x03u;

	while(blkCnt > 0u){
		/* C(m,n) = A(m,n) + B(m,n) */
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

	float inA1, inA2, inB1, inB2, out1, out2;

	unsigned int numSamples;
	unsigned int blkCnt;

	numSamples = (unsigned int) numRows * numCols;

	blkCnt = numSamples >> 2u;
	while(blkCnt > 0u){
		/* C(m,n) = A(m,n) - B(m,n) */
		inA1 = pIn1[0];
		inB1 = pIn2[0];
		inA2 = pIn1[1];
		out1 = inA1 - inB1;

		inB2 = pIn2[1];
		inA1 = pIn1[2];
		out2 = inA2 - inB2;

		inB1 = pIn2[2];
		pOut[0] = out1;
		pOut[1] = out2;

		inA2 = pIn1[3];
		inB2 = pIn2[3];
		out1 = inA1 - inB1;
		out2 = inA2 - inB2;

		pOut[2] = out1;
		pOut[3] = out2;

		pIn1 += 4u;
		pIn2 += 4u;
		pOut += 4u;

		blkCnt--;
	}
	blkCnt = numSamples & 0x03u;

	while(blkCnt > 0u){
		/* C(m,n) = A(m,n) - B(m,n) */
		*pOut++ = (*pIn1++) + (*pIn2++);

		blkCnt--;
	}
	return 0;
}

__inline int Maxtrix_Mult(float *pSrcA, unsigned short numRowsA, unsigned short numColsA, float *pSrcB, unsigned short numColsB, float *pDst)
{
	float *pIn1 = pSrcA;
	float *pIn2 = pSrcB;
	float *pInA = pSrcA;
	float *pOut = pDst;
	float *px;
	float sum;

	float in1, in2, in3, in4;
	unsigned short col, i = 0u, j, row = numRowsA, colCnt;
	do{
		px = pOut + i;
		col = numColsB;
		pIn2 = pSrcB;
		j = 0u;

		do{
			sum = 0.0f;
			pIn1 = pInA;
			colCnt = numColsA >> 2u;
			while(colCnt > 0u){
				/* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
				in3 = *pIn2;
				pIn2 += numColsB;
				in1 = pIn1[0];
				in2 = pIn1[1];
				sum += in1 * in3;
				in4 = *pIn2;
				pIn2 += numColsB;
				sum += in2 * in4;

				in3 = *pIn2;
				pIn2 += numColsB;
				in1 = pIn1[2];
				in2 = pIn1[3];
				sum += in1 * in3;
				in4 = *pIn2;
				pIn2 += numColsB;
				sum += in2 * in4;
				pIn1 += 4u;

				colCnt--;
			}
			colCnt = numColsA & 0x03u;

			while(colCnt > 0u){
				/* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
				sum += *pIn1++ * (*pIn2);
				pIn2 += numColsB;

				colCnt--;
			}
			*px++ = sum;
			j++;
			pIn2 = pSrcB + j;

			col--;

		} while(col > 0u);

		i = i + numColsB;
		pInA = pInA + numColsA;
		row--;
	} while(row > 0u);
	return 0;
}

__inline int Maxtrix_Trans(float *pSrc, unsigned short nRows, unsigned short nCols, float *pDst)
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
	return 0;
}

__inline int Maxtrix_Inverse(float *pSrc, unsigned int numRows, unsigned int numCols, float *pDst)
{
	float *pIn = pSrc;
	float *pOut = pDst;
	float *pInT1, *pInT2;
	float *pInT3, *pInT4;
	float *pPivotRowIn, *pPRT_in, *pPivotRowDst, *pPRT_pDst;

	float maxC;
	int status = 0;
	float Xchg, in = 0.0f, in1;
	unsigned int i, rowCnt, flag = 0u, j, loopCnt, k, l;

	pInT2 = pOut;
	rowCnt = numRows;

	while(rowCnt > 0u){
		j = numRows - rowCnt;
		while(j > 0u){
			*pInT2++ = 0.0f;
			j--;
		}
		*pInT2++ = 1.0f;
		j = rowCnt - 1u;
		while(j > 0u){
			*pInT2++ = 0.0f;
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
		if(maxC == 0.0f){
			status = -1;
			break;
		}
		pInT1 -= numRows * numCols;
		if( (in > 0.0f ? in : -in) != maxC){
			i = numRows - (l + 1u);
			while(i > 0u){
				pInT2 = pInT1 + (numCols * l);
				pInT4 = pInT3 + (numCols * k);

				if((*pInT2 > 0.0f ? *pInT2: -*pInT2) == maxC){
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
		if((flag != 1u) && (in == 0.0f)){
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
			*pInT1++ = in1 / in;
			j--;
		}
		j = numCols;

		while(j > 0u){
			in1 = *pInT2;
			*pInT2++ = in1 / in;
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
					*pInT1++ = in1 - (in * *pPRT_in++);
					j--;
				}
				j = numCols;

				while(j > 0u){
					in1 = *pInT2;
					*pInT2++ = in1 - (in * *pPRT_pDst++);
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
