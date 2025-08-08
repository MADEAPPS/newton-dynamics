/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndBrainStdafx.h"
#include "ndBrainMatrix.h"

#define D_BRAIN_MATRIX_ALIGNMENT 16

ndBrainMatrix::ndBrainMatrix()
	:ndArray<ndBrainMemVector>()
	,m_memory(nullptr)
{
}

ndBrainMatrix::ndBrainMatrix(ndInt32 rows, ndInt32 columns)
	:ndArray<ndBrainMemVector>()
	,m_memory(nullptr)
{
	Init(rows, columns);
}

ndBrainMatrix::ndBrainMatrix(const ndBrainMatrix& src)
	:ndArray<ndBrainMemVector>()
	,m_memory(nullptr)
{
	Init(src.GetRows(), src.GetColumns());
	Set(src);
}

ndBrainMatrix::~ndBrainMatrix()
{
	ndBrainMatrix& me = *this;
	for (ndInt32 i = ndInt32(GetCount() - 1); i >= 0; --i)
	{
		ndBrainMemVector& row = me[i];
		row.~ndBrainMemVector();
	}
	m_array = nullptr;
	if (m_memory)
	{
		ndMemory::Free(m_memory);
	}
}

void ndBrainMatrix::Init(ndInt32 rows, ndInt32 columns)
{
	m_size = rows;
	m_capacity = rows + 1;

	size_t strideInBytes = size_t((ndInt64(columns * sizeof(ndBrainFloat)) + D_BRAIN_MATRIX_ALIGNMENT - 1) & -D_BRAIN_MATRIX_ALIGNMENT);
	size_t size = size_t(rows * sizeof(ndBrainMemVector) + 256);
	size += strideInBytes * rows;
	m_memory = (ndBrainFloat*)ndMemory::Malloc(size_t(size));
	m_array = (ndBrainMemVector*)m_memory;

	size_t bytes = size_t((rows * sizeof(ndBrainMemVector) + D_BRAIN_MATRIX_ALIGNMENT - 1) & -D_BRAIN_MATRIX_ALIGNMENT);
	ndInt8* ptr = (ndInt8*)m_memory + bytes;

	ndBrainMatrix& me = *this;
	for (ndInt32 i = 0; i < rows; ++i)
	{
		ndBrainMemVector& row = me[i];
		row.SetSize(columns);
		row.SetPointer((ndBrainFloat*) ptr);
		ptr += strideInBytes;
		ndAssert((bytes + strideInBytes * i) < size);
	}

	Set(ndBrainFloat(0.0f));
}

void ndBrainMatrix::Set(ndBrainFloat value)
{
	ndBrainMatrix& matrix = *this;

	ndInt32 strideInBytes = (ndInt32(GetColumns() * sizeof(ndBrainFloat)) + D_BRAIN_MATRIX_ALIGNMENT - 1) & -D_BRAIN_MATRIX_ALIGNMENT;
	ndInt32 size = ndInt32(GetCount()) * strideInBytes / ndInt32(sizeof(ndBrainFloat));
	ndBrainFloat* const ptr = &matrix[0][0];
	ndMemSet(ptr, value, size);
}

void ndBrainMatrix::Set(const ndBrainMatrix& src)
{
	ndAssert(src.GetRows() == GetRows());
	ndAssert(src.GetColumns() == GetColumns());
	ndBrainMatrix& matrix = *this;

	ndInt32 strideInBytes = (ndInt32(GetColumns() * sizeof(ndBrainFloat)) + D_BRAIN_MATRIX_ALIGNMENT - 1) & -D_BRAIN_MATRIX_ALIGNMENT;
	ndInt32 size = ndInt32(GetCount()) * strideInBytes / ndInt32(sizeof(ndBrainFloat));
	ndBrainFloat* const dstData = &matrix[0][0];
	const ndBrainFloat* const srcData = &src[0][0];
	ndMemCpy(dstData, srcData, size);
}

void ndBrainMatrix::Scale(ndBrainFloat scale)
{
	ndBrainMatrix& matrix = *this;

	ndInt32 strideInBytes = (ndInt32(GetColumns() * sizeof(ndBrainFloat)) + D_BRAIN_MATRIX_ALIGNMENT - 1) & -D_BRAIN_MATRIX_ALIGNMENT;
	ndInt32 size = ndInt32(GetCount()) * strideInBytes / ndInt32(sizeof(ndBrainFloat));
	ndBrainFloat* const ptr = &matrix[0][0];
	ndScale(size, ptr, scale);
}

void ndBrainMatrix::ScaleAdd(const ndBrainMatrix& src, ndBrainFloat scale)
{
	ndBrainMatrix& matrix = *this;

	ndInt32 strideInBytes = (ndInt32(GetColumns() * sizeof(ndBrainFloat)) + D_BRAIN_MATRIX_ALIGNMENT - 1) & -D_BRAIN_MATRIX_ALIGNMENT;
	ndInt32 size = ndInt32(GetCount()) * strideInBytes / ndInt32(sizeof(ndBrainFloat));
	ndBrainFloat* const dstData = &matrix[0][0];
	const ndBrainFloat* const srcData = &src[0][0];
	ndScaleAdd(size, dstData, srcData, scale);
}

void ndBrainMatrix::Add(const ndBrainMatrix& src)
{
	ndAssert(src.GetRows() == GetRows());
	ndAssert(src.GetColumns() == GetColumns());
	ndBrainMatrix& matrix = *this;

	ndInt32 strideInBytes = (ndInt32(GetColumns() * sizeof(ndBrainFloat)) + D_BRAIN_MATRIX_ALIGNMENT - 1) & -D_BRAIN_MATRIX_ALIGNMENT;
	ndInt32 size = ndInt32(GetCount()) * strideInBytes / ndInt32(sizeof(ndBrainFloat));
	ndBrainFloat* const dstData = &matrix[0][0];
	const ndBrainFloat* const srcData = &src[0][0];
	ndAdd(size, dstData, srcData);
}

void ndBrainMatrix::Mul(const ndBrainMatrix& src)
{
	ndAssert(src.GetRows() == GetRows());
	ndAssert(src.GetColumns() == GetColumns());
	ndBrainMatrix& matrix = *this;

	ndInt32 strideInBytes = (ndInt32(GetColumns() * sizeof(ndBrainFloat)) + D_BRAIN_MATRIX_ALIGNMENT - 1) & -D_BRAIN_MATRIX_ALIGNMENT;
	ndInt32 size = ndInt32(GetCount()) * strideInBytes / ndInt32(sizeof(ndBrainFloat));
	ndBrainFloat* const dstData = &matrix[0][0];
	const ndBrainFloat* const srcData = &src[0][0];
	ndMul(size, dstData, srcData);
}

void ndBrainMatrix::Blend(const ndBrainMatrix& src, ndBrainFloat blend)
{
	ndAssert(src.GetRows() == GetRows());
	ndAssert(src.GetColumns() == GetColumns());
	ndBrainMatrix& matrix = *this;

	ndInt32 strideInBytes = (ndInt32(GetColumns() * sizeof(ndBrainFloat)) + D_BRAIN_MATRIX_ALIGNMENT - 1) & -D_BRAIN_MATRIX_ALIGNMENT;
	ndInt32 size = ndInt32(GetCount()) * strideInBytes / ndInt32(sizeof(ndBrainFloat));
	ndBrainFloat* const dstPtr = &matrix[0][0];
	const ndBrainFloat* const srcPtr = &src[0][0];
	ndBrainMemVector dstVector(dstPtr, size);
	const ndBrainMemVector srcVector(srcPtr, size);
	dstVector.Blend(srcVector, blend);
}

void ndBrainMatrix::FlushToZero()
{
	ndBrainMatrix& matrix = *this;

	ndInt32 strideInBytes = (ndInt32(GetColumns() * sizeof(ndBrainFloat)) + D_BRAIN_MATRIX_ALIGNMENT - 1) & -D_BRAIN_MATRIX_ALIGNMENT;
	ndInt32 size = ndInt32(GetCount()) * strideInBytes / ndInt32(sizeof(ndBrainFloat));
	ndBrainFloat* const ptr = &matrix[0][0];
	ndBrainMemVector tmp(ptr, size);
	tmp.FlushToZero();
}

void ndBrainMatrix::Mul(const ndBrainVector& input, ndBrainVector& output) const
{
	const ndBrainMatrix& me = *this;
	ndInt32 columns = ndInt32(input.GetCount());
	ndAssert(columns == GetColumns());
	ndAssert(output.GetCount() == GetCount());
	
	for (ndInt32 i = ndInt32(GetCount() - 1); i >= 0; --i)
	{
		const ndBrainVector& row = me[i];
		output[i] = ndDotProduct(columns, &row[0], &input[0]);
		// exploding gradients
		ndAssert(output[i] <= ndFloat32(1000.0f));
		ndAssert(output[i] >= ndFloat32(-1000.0f));
	}
}

void ndBrainMatrix::TransposeMul(const ndBrainVector& input, ndBrainVector& output) const
{
	const ndBrainMatrix& me = *this;
	ndAssert(input.GetCount() == GetCount());
	ndAssert(output.GetCount() == GetColumns());

	output.Set(me[input.GetCount() - 1]);
	output.Scale(input[input.GetCount() - 1]);
	for (ndInt32 j = ndInt32(me.GetCount() - 2); j >= 0; --j)
	{
		ndBrainFloat scale = input[j];
		const ndBrainVector& row = me[j];
		output.ScaleAdd(row, scale);
	}
}

void ndBrainMatrix::InitGaussianWeights(ndBrainFloat variance)
{
	ndBrainMatrix& me = *this;
	variance = ndBrainFloat (ndAbs(variance)) + ndBrainFloat(1.0e-3f);
	for (ndInt32 i = ndInt32(me.GetCount() - 1); i >= 0; --i)
	{
		me[i].InitGaussianWeights(variance);
	}
}