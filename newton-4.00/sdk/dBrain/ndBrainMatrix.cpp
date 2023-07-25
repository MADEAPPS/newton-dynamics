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
	:ndArray<ndBrainVector>()
{
}

ndBrainMatrix::ndBrainMatrix(ndInt32 rows, ndInt32 columns)
	:ndArray<ndBrainVector>()
{
	Init(rows, columns);
}

ndBrainMatrix::ndBrainMatrix(const ndBrainMatrix& src)
	:ndArray<ndBrainVector>(src)
{
	ndBrainMatrix& me = *this;
	for (ndInt32 i = 0; i < src.GetRows(); ++i)
	{
		ndBrainVector& row = me[i];
		row.ResetMembers();
		row.SetCount(src.GetColumns());
		row.Set(src[i]);
	}
}

ndBrainMatrix::~ndBrainMatrix()
{
	if (m_array)
	{
		ndBrainMatrix& me = *this;
		for (ndInt32 i = GetCount() - 1; i >= 0; --i)
		{
			ndBrainVector& row = me[i];
			row.~ndBrainVector();
		}
	}
}

void ndBrainMatrix::Init(ndInt32 rows, ndInt32 columns)
{
	SetCount(rows);
	ndBrainMatrix& me = *this;
	for (ndInt32 i = 0; i < rows; ++i)
	{
		ndBrainVector& row = me[i];
		row.ResetMembers();
		row.SetCount(columns);
	}
}

ndUnsigned8* ndBrainMatrix::SetPointer(ndUnsigned8* const mem)
{
	m_array = (ndDeepBrainMemVector*)mem;
	return mem + GetCount() * sizeof (ndDeepBrainMemVector);
}

ndReal* ndBrainMatrix::SetFloatPointers(ndReal* const mem, ndInt32 columns)
{
	ndInt32 count = 0;
	for (ndInt32 i = 0; i < GetCount(); ++i)
	{
		m_array[i].SetMembers(columns, &mem[count]);
		count += (m_array[i].GetCount() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	}
	return &mem[count];
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