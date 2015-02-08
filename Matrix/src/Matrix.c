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

arm_status mat_identity(float32_t *pData, uint16_t numRows, uint16_t numCols, float32_t value)
{
	uint16_t pos = 0;
	if (numRows != numCols){
		return ARM_MATH_LENGTH_ERROR;
	}
	do{
		pData[pos * numRows + pos] = value;
		pos++;
	} while (pos < numRows);
	return ARM_MATH_SUCCESS;
}

void arm_mat_zero_f32(arm_matrix_instance_f32* s)
{
	uint16_t pos = 0;
	uint16_t blockSize = s->numRows * s->numCols;
	
	do{
		s->pData[pos] = 0.0f;
		pos++;
	} while (pos < blockSize);
}

arm_status arm_mat_identity_f32(arm_matrix_instance_f32* s, float32_t value)
{
	uint16_t pos = 0;
	if (s->numRows != s->numCols){
		return ARM_MATH_LENGTH_ERROR;
	}
	do{
		s->pData[pos * s->numRows + pos] = value;
		pos++;
	} while (pos < s->numRows);
	return ARM_MATH_SUCCESS;
}

arm_status arm_mat_fill_f32(arm_matrix_instance_f32* s, float32_t *pData, uint32_t blockSize)
{
	uint16_t pos = 0;
	if (s->numRows * s->numCols < blockSize){
		return ARM_MATH_LENGTH_ERROR;
	}

	do{
		s->pData[pos] = pData[pos];
		pos++;
	} while (pos < blockSize);
	return ARM_MATH_SUCCESS;
}
