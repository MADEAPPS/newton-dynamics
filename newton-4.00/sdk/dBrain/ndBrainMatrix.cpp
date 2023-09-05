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
#include "ndBrainTypes.h"
#include "ndBrainMatrix.h"

ndBrainMatrix::ndBrainMatrix()
	:ndArray<ndDeepBrainMemVector>()
	,m_memory(nullptr)
{
	ndAssert(0);
}

ndBrainMatrix::ndBrainMatrix(ndInt32 rows, ndInt32 columns)
	:ndArray<ndDeepBrainMemVector>()
	,m_memory(nullptr)
{
	Init(rows, columns);
}

ndBrainMatrix::ndBrainMatrix(const ndBrainMatrix& src)
	:ndArray<ndDeepBrainMemVector>()
	,m_memory(nullptr)
{
	Init(src.GetRows(), src.GetColumns());
	Set(src);
}

ndBrainMatrix::~ndBrainMatrix()
{
	ndAssert(0);
	ndBrainMatrix& me = *this;
	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainMemVector& row = me[i];
		row.~ndDeepBrainMemVector();
	}
	m_array = nullptr;
	ndAssert(m_memory);
	ndMemory::Free(m_memory);
}

void ndBrainMatrix::Init(ndInt32 rows, ndInt32 columns)
{
	m_size = rows;
	m_capacity = rows + 1;

	ndInt32 padding = D_DEEP_BRAIN_DATA_ALIGMENT * 4;
	ndInt32 columSizeInBytes = (ndInt32(columns * sizeof(ndReal)) + padding - 1) & -padding;
	ndInt32 size = ndInt32 (rows * sizeof(ndDeepBrainMemVector) + 256);
	size += columSizeInBytes * rows;
	m_memory = (ndReal*)ndMemory::Malloc(size_t(size));
	m_array = (ndDeepBrainMemVector*)m_memory;

	ndInt32 bytes = ndInt32((rows * sizeof(ndDeepBrainMemVector) + padding - 1) & -padding);
	ndInt8* ptr = (ndInt8*)m_memory + bytes;

	ndBrainMatrix& me = *this;
	for (ndInt32 i = 0; i < rows; ++i)
	{
		ndDeepBrainMemVector& row = me[i];
		row.SetSize(columns);
		row.SetPointer((ndReal*) ptr);
		ptr += columSizeInBytes;
		ndAssert((bytes + columSizeInBytes * i) < size);
	}
}

void ndBrainMatrix::Set(ndReal value)
{
	ndBrainMatrix& me = *this;
	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		me[i].Set(value);
	}
}

void ndBrainMatrix::Set(const ndBrainMatrix& src)
{
	ndAssert(src.GetRows() == GetRows());
	ndAssert(src.GetColumns() == GetColumns());
	ndBrainMatrix& matrix = *this;
	for (ndInt32 i = 0; i < src.GetRows(); ++i)
	{
		matrix[i].Set(src[i]);
	}
}

void ndBrainMatrix::Blend(const ndBrainMatrix& src, ndReal blend)
{
	ndAssert(src.GetRows() == GetRows());
	ndAssert(src.GetColumns() == GetColumns());
	ndBrainMatrix& matrix = *this;
	for (ndInt32 i = 0; i < src.GetRows(); ++i)
	{
		matrix[i].Blend(src[i], blend);
	}
}

void ndBrainMatrix::Mul(const ndBrainVector& input, ndBrainVector& output) const
{
	const ndBrainMatrix& me = *this;
	ndInt32 columns = input.GetCount();
	ndAssert(columns == GetColumns());
	ndAssert(output.GetCount() == GetCount());
	
	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		const ndBrainVector& row = me[i];
		output[i] = ndDotProduct(columns, &row[0], &input[0]);
	}
}

void ndBrainMatrix::TransposeMul(const ndBrainVector& input, ndBrainVector& output) const
{
	const ndBrainMatrix& me = *this;
	ndAssert(input.GetCount() == GetCount());
	ndAssert(output.GetCount() == GetColumns());

	output.Set(me[input.GetCount() - 1]);
	output.Scale(input[input.GetCount() - 1]);
	for (ndInt32 j = me.GetCount() - 2; j >= 0; --j)
	{
		ndReal scale = input[j];
		const ndBrainVector& row = me[j];
		output.ScaleAdd(row, scale);
	}
}