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

#include "FP_Matrix.h"

//////////////////////////////////////////////////////////////////////////
//

void FP_Matrix_Copy(Q16 *pSrc, int numRows, int numCols, Q16 *pDst)
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

int FP_Maxtrix_Add(Q16 *pSrcA, unsigned short numRows, unsigned short numCols, Q16 *pSrcB, Q16 *pDst)
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

int FP_Maxtrix_Sub(Q16 *pSrcA, unsigned short numRows, unsigned short numCols, Q16 *pSrcB, Q16 *pDst)
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

int FP_Matrix_Multiply(Q16 *A, int nrows, int ncols, Q16 *B, int mcols, Q16 *C)
{
	Q16 *pB;
	Q16 *p_B;
	int i,j,k;

	for (i = 0; i < nrows; A += ncols, i++){
		for (p_B = B, j = 0; j < mcols; C++, p_B++, j++) {
			pB = p_B;
			*C = 0;
			for (k = 0; k < ncols; pB += mcols, k++){
				//*C += *(A+k) * *pB;
				*C = FP_ADDS(*C, FP_SMUL(*(A+k),*pB));
			}
		}
	}
	return 0;
}

void FP_Matrix_Multiply_With_Transpose(Q16 *A, int nrows, int ncols, Q16 *B, int mrows, Q16 *C) 
{
	int i,j,k;
	Q16 *pA;
	Q16 *pB;

	for (i = 0; i < nrows; A += ncols, i++){
		for (pB = B, j = 0; j < mrows; C++, j++){
			*C = 0;
			for (pA = A, /* ,*C = 0 */k  = 0; k < ncols; k++){
				//*C += *pA++ * *pB++;
				*C = FP_ADDS(*C, FP_SMUL(*pA++, *pB++));
			}
		}
	}
}

int FP_Matrix_Inverse(Q16* pSrc, unsigned short n, Q16* pDst)
{
	Q16 *pIn = pSrc; // input data matrix pointer
	Q16 *pOut = pDst; // output data matrix pointer
	Q16 *pInT1, *pInT2; // Temporary input data matrix pointer
	Q16 *pOutT1, *pOutT2; // Temporary output data matrix pointer
	Q16 *pPivotRowIn, *pPRT_in, *pPivotRowDst, *pPRT_pDst; // Temporary input and output data matrix pointer
	Q16 maxC; // maximum value in the column

	Q16 Xchg, in = 0, in1; // Temporary input values 
	unsigned int i, rowCnt, flag = 0u, j, loopCnt, k, l; // loop counters
	int status; // status of matrix inverse

	// Working pointer for destination matrix
	pOutT1 = pOut;

	// Loop over the number of rows
	rowCnt = n;

	// Making the destination matrix as identity matrix
	while(rowCnt > 0u){
		// Writing all zeroes in lower triangle of the destination matrix
		j = n - rowCnt;
		while(j > 0u){
			*pOutT1++ = 0;
			j--;
		}

		// Writing all ones in the diagonal of the destination matrix
		*pOutT1++ = Q16_One;

		// Writing all zeroes in upper triangle of the destination matrix
		// j = rowCnt - 1u;
		while(j > 0u){
			*pOutT1++ = 0;
			j--;
		}

		// Decrement the loop counter
		rowCnt--;
	}

	// Loop over the number of columns of the input matrix.    
	// All the elements in each column are processed by the row operations
	loopCnt = n;

	// Index modifier to navigate through the columns
	l = 0u;

	while(loopCnt > 0u){
		// Check if the pivot element is zero..    
		// If it is zero then interchange the row with non zero row below.    
		// If there is no non zero element to replace in the rows below,    
		// then the matrix is Singular.

		// Working pointer for the input matrix that points    
		// to the pivot element of the particular row 
		pInT1 = pIn + (l * n);

		// Working pointer for the destination matrix that points    
		// to the pivot element of the particular row 
		pOutT1 = pOut + (l * n);

		// Temporary variable to hold the pivot value
		in = *pInT1;

		// Grab the most significant value from column l
		maxC = 0;
		for (i = l; i < n; i++){
			maxC = *pInT1 > 0 ? (*pInT1 > maxC ? *pInT1 : maxC) : (-*pInT1 > maxC ? -*pInT1 : maxC);
			pInT1 += n;
		}

		// Update the status if the matrix is singular
		if(maxC == 0){
			return -1;
		}

		// Restore pInT1 
		pInT1 = pIn;

		// Destination pointer modifier
		k = 1u;

		// Check if the pivot element is the most significant of the column
		if( (in > 0 ? in : -in) != maxC){
			// Loop over the number rows present below
			i = n - (l + 1u);

			while(i > 0u){
				// Update the input and destination pointers
				pInT2 = pInT1 + (n * l);
				pOutT2 = pOutT1 + (n * k);

				// Look for the most significant element to    
				// replace in the rows below
				if((*pInT2 > 0 ? *pInT2: -*pInT2) == maxC){
					// Loop over number of columns    
					// to the right of the pilot element
					j = n - l;

					while(j > 0u){
						// Exchange the row elements of the input matrix
						Xchg = *pInT2;
						*pInT2++ = *pInT1;
						*pInT1++ = Xchg;

						// Decrement the loop counter
						j--;
					}

					// Loop over number of columns of the destination matrix
					j = n;

					while(j > 0u){
						// Exchange the row elements of the destination matrix
						Xchg = *pOutT2;
						*pOutT2++ = *pOutT1;
						*pOutT1++ = Xchg;

						// Decrement the loop counter
						j--;
					}

					// Flag to indicate whether exchange is done or not
					flag = 1u;

					// Break after exchange is done
					break;
				}

				// Update the destination pointer modifier
				k++;

				// Decrement the loop counter
				i--;
			}
		}

		// Update the status if the matrix is singular
		if((flag != 1u) && (in == 0)){
			return -1;
		}

		// Points to the pivot row of input and destination matrices
		pPivotRowIn = pIn + (l * n);
		pPivotRowDst = pOut + (l * n);

		// Temporary pointers to the pivot row pointers
		pInT1 = pPivotRowIn;
		pInT2 = pPivotRowDst;

		// Pivot element of the row
		in = *pPivotRowIn;

		// Loop over number of columns    
		// to the right of the pilot element
		j = (n - l);

		while(j > 0u){
			// Divide each element of the row of the input matrix    
			// by the pivot element
			in1 = *pInT1;
			//*pInT1++ = in1 / in;
			*pInT1++ = FP_SDIV(in1, in);

			// Decrement the loop counter
			j--;
		}

		// Loop over number of columns of the destination matrix
		j = n;

		while(j > 0u){
			// Divide each element of the row of the destination matrix    
			// by the pivot element
			in1 = *pInT2;
			//*pInT2++ = in1 / in;
			*pInT2++ = FP_SDIV(in1, in);

			// Decrement the loop counter
			j--;
		}

		// Replace the rows with the sum of that row and a multiple of row i    
		// so that each new element in column i above row i is zero.*/

		// Temporary pointers for input and destination matrices
		pInT1 = pIn;
		pInT2 = pOut;

		// index used to check for pivot element
		i = 0u;

		// Loop over number of rows
		//  to be replaced by the sum of that row and a multiple of row i
		k = n;

		while(k > 0u){
			// Check for the pivot element
			if(i == l){
				// If the processing element is the pivot element,    
				// only the columns to the right are to be processed
				pInT1 += n - l;

				pInT2 += n;
			}
			else{
				// Element of the reference row
				in = *pInT1;

				// Working pointers for input and destination pivot rows
				pPRT_in = pPivotRowIn;
				pPRT_pDst = pPivotRowDst;

				// Loop over the number of columns to the right of the pivot element,    
				// to replace the elements in the input matrix
				j = (n - l);

				while(j > 0u){
					// Replace the element by the sum of that row    
					// and a multiple of the reference row 
					in1 = *pInT1;
					//*pInT1++ = in1 - (in * *pPRT_in++);
					*pInT1++ = FP_SUBS(in1, FP_SMUL(in, *pPRT_in++));

					// Decrement the loop counter
					j--;
				}

				// Loop over the number of columns to    
				// replace the elements in the destination matrix
				j = n;

				while(j > 0u){
					// Replace the element by the sum of that row    
					// and a multiple of the reference row 
					in1 = *pInT2;
					//*pInT2++ = in1 - (in * *pPRT_pDst++);
					*pInT2++ = FP_SUBS(in1, FP_SMUL(in, *pPRT_pDst++));

					// Decrement the loop counter
					j--;
				}

			}

			// Increment the temporary input pointer
			pInT1 = pInT1 + l;

			// Decrement the loop counter
			k--;

			// Increment the pivot index
			i++;
		}

		// Increment the input pointer
		pIn++;

		// Decrement the loop counter
		loopCnt--;

		// Increment the index modifier
		l++;
	}

	// Set status as SUCCESS
	status = 0;

	if((flag != 1u) && (in == 0)){
		status = -1;
	}

	// Return to application
	return (status);
}
