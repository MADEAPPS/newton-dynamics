/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#include "ndDeepBrainStdafx.h"
#include "ndDeepBrainMatrix.h"

ndDeepBrainMatrix::ndDeepBrainMatrix(ndInt32 rows, ndInt32 columns)
	:ndArray<ndDeepBrainVector>()
{
	SetCount(rows);
	ndDeepBrainMatrix& me = *this;
	for (ndInt32 i = 0; i < rows; ++i)
	{
		ndDeepBrainVector& row = me[i];
		row.ResetMembers();
		row.SetCount(columns);
	}
}

ndDeepBrainMatrix::ndDeepBrainMatrix(const ndDeepBrainMatrix& src)
	:ndArray<ndDeepBrainVector>(src)
{
	ndDeepBrainMatrix& me = *this;
	for (ndInt32 i = 0; i < src.GetRows(); ++i)
	{
		ndDeepBrainVector& row = me[i];
		row.ResetMembers();
		row.SetCount(src.GetColumns());
		row.Set(src[i]);
	}
}

ndDeepBrainMatrix::~ndDeepBrainMatrix()
{
	ndDeepBrainMatrix& me = *this;
	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		ndDeepBrainVector& row = me[i];
		row.~ndDeepBrainVector();
	}
}

void ndDeepBrainMatrix::Set(ndReal value)
{
	ndDeepBrainMatrix& me = *this;
	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		me[i].Set(value);
	}
}

void ndDeepBrainMatrix::SetTranspose(const ndDeepBrainMatrix& src)
{
	ndDeepBrainMatrix& matrix = *this;
	ndAssert(src.GetColumns() == GetRows());
	ndAssert(src.GetRows() == GetColumns());
	for (ndInt32 i = 0; i < src.GetRows(); ++i)
	{
		for (ndInt32 j = 0; j < src.GetColumns(); ++j)
		{
			ndReal val = src[i][j];
			matrix[j][i] = val;
		}
	}
}

void ndDeepBrainMatrix::InitGaussianWeights(ndReal mean, ndReal variance)
{
	ndDeepBrainMatrix& me = *this;
	for (ndInt32 i = GetCount() - 1; i >= 0 ; --i)
	{
		me[i].InitGaussianWeights(mean, variance);
	}
}

void ndDeepBrainMatrix::Mul(const ndDeepBrainVector& input, ndDeepBrainVector& output) const
{
	const ndDeepBrainMatrix& me = *this;
	ndInt32 columns = input.GetCount();
	ndAssert(columns == GetColumns());
	ndAssert(output.GetCount() == GetCount());

	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		const ndDeepBrainVector& row = me[i];
		output[i] = ndDotProduct(columns, &row[0], &input[0]);
	}
}